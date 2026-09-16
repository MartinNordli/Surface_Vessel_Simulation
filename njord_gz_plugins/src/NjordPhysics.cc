// SPDX-License-Identifier: MIT
#include "njord/Hydrostatics.hh"
#include <chrono>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <mutex>
namespace njord {
class Physics final : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
  using V = gz::math::Vector3d;
  using Clock = std::chrono::steady_clock;
  struct Thruster {
    V position, axis;
    double forward, reverse, tau, force{};
  };
  struct Coefficient {
    double angle, cx, cy, cn;
  };
  gz::sim::Link link;
  std::vector<hydro::Mesh> meshes;
  V com, wind;
  double density{}, level{}, areaX{}, areaY{}, length{}, timeout{};
  std::vector<Thruster> motors;
  std::vector<Coefficient> table;
  gz::transport::Node node;
  gz::transport::Node::Publisher telemetry;
  std::chrono::steady_clock::duration lastTelemetry{};
  std::mutex mutex;
  Clock::time_point received{};
  double commands[2]{};
  bool valid{};

public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &s,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    link = gz::sim::Link(gz::sim::Model(entity).LinkByName(
        ecm, s->Get<std::string>("link_name")));
    if (!link.Valid(ecm))
      throw std::invalid_argument("Njord physics link missing");
    link.EnableVelocityChecks(ecm);
    com = s->Get<V>("center_of_mass");
    wind = s->Get<V>("wind_velocity");
    density = s->Get<double>("water_density");
    level = s->Get<double>("water_level");
    timeout = s->Get<double>("timeout_s");
    auto mutableS = s->Clone();
    for (auto volume = mutableS->GetElement("buoyancy_volume"); volume;
         volume = volume->GetNextElement("buoyancy_volume")) {
      hydro::Mesh mesh;
      for (auto vertex = volume->GetElement("vertex"); vertex;
           vertex = vertex->GetNextElement("vertex")) {
        auto p = vertex->Get<V>();
        mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
      }
      for (auto face = volume->GetElement("triangle"); face;
           face = face->GetNextElement("triangle")) {
        auto f = face->Get<V>();
        for (int i = 0; i < 3; i++)
          if (f[i] < 0 || f[i] != std::floor(f[i]))
            throw std::invalid_argument("invalid triangle index");
        mesh.faces.push_back({static_cast<unsigned>(f.X()),
                              static_cast<unsigned>(f.Y()),
                              static_cast<unsigned>(f.Z())});
      }
      hydro::Validate(mesh);
      meshes.push_back(mesh);
    }
    if (meshes.empty())
      throw std::invalid_argument("buoyancy volumes missing");
    // Conservative separation: overlapping bounds are unsupported, even when
    // meshes might not intersect. This never silently double counts
    // displacement.
    for (unsigned i = 0; i < meshes.size(); i++)
      for (unsigned j = 0; j < i; j++) {
        auto bounds = [](const hydro::Mesh &m) {
          V lo(1e100, 1e100, 1e100), hi(-1e100, -1e100, -1e100);
          for (auto p : m.vertices) {
            V v(p.x, p.y, p.z);
            lo.Min(v);
            hi.Max(v);
          }
          return std::make_pair(lo, hi);
        };
        auto a = bounds(meshes[i]), b = bounds(meshes[j]);
        bool separated = false;
        for (int k = 0; k < 3; k++)
          if (a.second[k] < b.first[k] - 1e-9 ||
              b.second[k] < a.first[k] - 1e-9)
            separated = true;
        if (!separated)
          throw std::invalid_argument(
              "overlapping/touching buoyancy bounds unsupported");
      }
    areaX = s->Get<double>("wind_area_x");
    areaY = s->Get<double>("wind_area_y");
    length = s->Get<double>("wind_length");
    for (auto t = mutableS->GetElement("thruster"); t;
         t = t->GetNextElement("thruster"))
      motors.push_back({t->Get<V>("position"), t->Get<V>("axis"),
                        t->Get<double>("forward"), t->Get<double>("reverse"),
                        t->Get<double>("response_time")});
    if (motors.size() != 2)
      throw std::invalid_argument("exactly two thrusters required");
    for (auto c = mutableS->GetElement("wind_coefficient"); c;
         c = c->GetNextElement("wind_coefficient"))
      table.push_back({c->Get<double>("angle"), c->Get<double>("cx"),
                       c->Get<double>("cy"), c->Get<double>("cn")});
    std::sort(table.begin(), table.end(),
              [](auto a, auto b) { return a.angle < b.angle; });
    if (table.size() < 2)
      throw std::invalid_argument("wind table requires two angles");
    node.Subscribe("/njord/actuator_forces", &Physics::Command, this);
    telemetry = node.Advertise<gz::msgs::Twist>("/njord/actuator_applied");
  }
  void Command(const gz::msgs::Twist &m) {
    std::lock_guard<std::mutex> lock(mutex);
    commands[0] = m.linear().x();
    commands[1] = m.linear().y();
    valid = std::isfinite(commands[0]) && std::isfinite(commands[1]);
    received = Clock::now();
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    if (info.dt < decltype(info.dt)::zero()) {
      std::lock_guard<std::mutex> lock(mutex);
      valid = false;
      lastTelemetry = info.simTime;
      for (auto &t : motors)
        t.force = 0;
      return;
    }
    auto pose = gz::sim::worldPose(link.Entity(), ecm);
    auto rot = pose.Rot();
    auto worldCom = pose.Pos() + rot.RotateVector(com);
    V force, torque;
    for (const auto &mesh : meshes) {
      auto submerged = mesh;
      for (auto &p : submerged.vertices) {
        auto q = pose.Pos() + rot.RotateVector(V(p.x, p.y, p.z));
        p = {q.X(), q.Y(), q.Z()};
      }
      auto d = hydro::Submerged(submerged, level);
      V f(0, 0, density * 9.81 * d.volume);
      force += f;
      torque +=
          (V(d.centroid.x, d.centroid.y, d.centroid.z) - worldCom).Cross(f);
    }
    // Compute at COM, then transport the wrench to the link origin below.
    {
      std::lock_guard<std::mutex> lock(mutex);
      bool live =
          valid &&
          std::chrono::duration<double>(Clock::now() - received).count() <=
              timeout;
      double targets[2]{};
      for (unsigned i = 0; i < motors.size(); i++) {
        auto &t = motors[i];
        double target =
            live ? std::clamp(commands[i], -t.reverse, t.forward) : 0;
        targets[i] = target;
        t.force = hydro::Response(
            t.force, target,
            std::max(0., std::chrono::duration<double>(info.dt).count()),
            t.tau);
        auto f = rot.RotateVector(t.axis * t.force);
        force += f;
        torque += rot.RotateVector(t.position - com).Cross(f);
      }
      // Evaluation-only force telemetry, never an autonomy input. The state
      // includes this step's response integration, hence its end-step stamp.
      auto stamp = info.simTime + info.dt;
      if (stamp - lastTelemetry >= std::chrono::milliseconds(20)) {
        gz::msgs::Twist msg;
        auto ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count();
        msg.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
        msg.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
        msg.mutable_linear()->set_x(motors[0].force);
        msg.mutable_linear()->set_y(motors[1].force);
        msg.mutable_angular()->set_x(targets[0]);
        msg.mutable_angular()->set_y(targets[1]);
        msg.mutable_angular()->set_z(live ? 1 : 0);
        telemetry.Publish(msg);
        lastTelemetry = stamp;
      }
    }
    auto velocity = link.WorldLinearVelocity(ecm, com);
    if (velocity) {
      auto air = rot.RotateVectorReverse(wind - *velocity);
      double angle = std::atan2(air.Y(), air.X()) * 180 / M_PI;
      angle = std::fmod(angle + 360., 360.);
      auto upper = std::upper_bound(
          table.begin(), table.end(), angle,
          [](double a, const Coefficient &b) { return a < b.angle; });
      auto b = upper == table.end() ? table.front() : *upper;
      auto a = upper == table.begin() ? table.back() : *(upper - 1);
      double aa = a.angle, bb = b.angle;
      if (bb <= aa)
        bb += 360;
      if (angle < aa)
        angle += 360;
      double f = (angle - aa) / (bb - aa);
      double pressure = .5 * 1.225 * (air.X() * air.X() + air.Y() * air.Y());
      force +=
          rot.RotateVector(V(pressure * areaX * (a.cx + f * (b.cx - a.cx)),
                             pressure * areaY * (a.cy + f * (b.cy - a.cy)), 0));
      torque += rot.RotateVector(
          V(0, 0, pressure * areaY * length * (a.cn + f * (b.cn - a.cn))));
    }
    link.AddWorldWrench(ecm, force, torque, com);
  }
};
} // namespace njord
GZ_ADD_PLUGIN(njord::Physics, gz::sim::System, njord::Physics::ISystemConfigure,
              njord::Physics::ISystemPreUpdate)
