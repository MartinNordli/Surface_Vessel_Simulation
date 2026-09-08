// SPDX-License-Identifier: MIT
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/double.pb.h>

namespace njord {
// A single atomic Twist transports two forces: linear.x = left, linear.y = right.
// This is an internal transport envelope, never a kinematic velocity command.
class ActuatorWatchdog final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  using Clock = std::chrono::steady_clock;
  gz::transport::Node node;
  gz::transport::Node::Publisher leftPub, rightPub;
  std::mutex mutex;
  Clock::time_point received{};
  double left{0}, right{0}, timeout{0.5}, maxForce{500};
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    timeout = sdf->Get<double>("timeout_s", 0.5).first;
    maxForce = sdf->Get<double>("max_force_n", 500).first;
    if (!std::isfinite(timeout) || !std::isfinite(maxForce) || timeout <= 0 || maxForce <= 0)
      throw std::invalid_argument("watchdog timeout_s and max_force_n must be positive finite values");
    leftPub = node.Advertise<gz::msgs::Double>("/wamv/thrusters/left/thrust");
    rightPub = node.Advertise<gz::msgs::Double>("/wamv/thrusters/right/thrust");
    node.Subscribe("/njord/actuator_forces", &ActuatorWatchdog::Command, this);
  }
  void Command(const gz::msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(mutex);
    const double l = msg.linear().x(), r = msg.linear().y();
    left = std::isfinite(l) && std::isfinite(r) ? std::clamp(l, -maxForce, maxForce) : 0;
    right = std::isfinite(l) && std::isfinite(r) ? std::clamp(r, -maxForce, maxForce) : 0;
    received = Clock::now();
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &) override {
    std::lock_guard<std::mutex> lock(mutex);
    const bool valid = !info.paused && std::chrono::duration<double>(Clock::now()-received).count() <= timeout;
    gz::msgs::Double l, r;
    l.set_data(valid ? left : 0); r.set_data(valid ? right : 0);
    leftPub.Publish(l); rightPub.Publish(r);
  }
};
}
GZ_ADD_PLUGIN(njord::ActuatorWatchdog, gz::sim::System,
              njord::ActuatorWatchdog::ISystemConfigure,
              njord::ActuatorWatchdog::ISystemPreUpdate)
