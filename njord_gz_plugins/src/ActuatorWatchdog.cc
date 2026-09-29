// SPDX-License-Identifier: MIT
//
// njord::ActuatorWatchdog -- Gazebo model plugin used by the WAM-V reference
// profile (attached by njord_sim/vessel.py). It is the only path from ROS
// thrust commands to the upstream VRX thruster plugins.
//
// SDF parameters:
//   <timeout_s>    simulation seconds a command stays valid (required, set
//                  from constants.COMMAND_TIMEOUT_S)
//   <liveness_timeout_s> steady wall-clock seconds without a command before
//                  thrust is zeroed even if simulation time is stalled
//                  (required, set from constants.PROCESS_LIVENESS_S)
//   <max_force_n>  symmetric thrust limit in newtons (default 500, set from
//                  the vessel's max_thrust_n)
//
// Topics (Gazebo transport):
//   subscribes /njord/actuator_forces          gz.msgs.Float_V, two forces in
//                                              newtons: port, starboard
//   publishes  /wamv/thrusters/left/thrust     gz.msgs.Double, newtons
//   publishes  /wamv/thrusters/right/thrust    gz.msgs.Double, newtons
//
// Failure behavior: a non-finite value or a command without exactly two values
// zeroes both thrusters; a command received more than timeout_s of simulation
// time or liveness_timeout_s of steady time ago, no command yet, or a paused
// simulation all publish zero thrust. Zero thrust is a command to the VRX thruster model,
// not a stop: the hull keeps its momentum and is slowed only by hydrodynamics.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/double.pb.h>

namespace njord {
// A single atomic Float_V transports both forces: data[0] = left (thruster_1),
// data[1] = right (thruster_2), in the order of constants.WAMV_THRUSTERS.
class ActuatorWatchdog final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  // Command freshness is measured in simulation time, so a command acts for
  // the same simulated duration at any real-time factor. Steady wall time is
  // only the liveness watchdog for a publisher that died while /clock stalls.
  using Clock = std::chrono::steady_clock;
  gz::transport::Node node;
  gz::transport::Node::Publisher leftPub, rightPub;
  // Guards the latest command, written from the transport callback thread
  // and read in the simulation thread.
  std::mutex mutex;
  // Epoch default: before the first command the age is huge, so thrust is 0.
  Clock::time_point received{};
  // Simulation time of the first step after the last command arrived.
  std::chrono::steady_clock::duration receivedSim{};
  bool stampPending{}, commanded{};
  double left{0}, right{0}, timeout{0}, liveness{0}, maxForce{500};
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    timeout = sdf->Get<double>("timeout_s", 0.0).first;
    liveness = sdf->Get<double>("liveness_timeout_s", 0.0).first;
    maxForce = sdf->Get<double>("max_force_n", 500).first;
    if (!std::isfinite(timeout) || !std::isfinite(liveness) || !std::isfinite(maxForce) ||
        timeout <= 0 || liveness <= 0 || maxForce <= 0)
      throw std::invalid_argument(
          "watchdog timeout_s, liveness_timeout_s and max_force_n must be positive finite values");
    leftPub = node.Advertise<gz::msgs::Double>("/wamv/thrusters/left/thrust");
    rightPub = node.Advertise<gz::msgs::Double>("/wamv/thrusters/right/thrust");
    node.Subscribe("/njord/actuator_forces", &ActuatorWatchdog::Command, this);
  }
  // Transport callback: store the clamped left/right forces (N) and the
  // steady receive time. A wrong length or a non-finite value zeroes both.
  void Command(const gz::msgs::Float_V &msg) {
    std::lock_guard<std::mutex> lock(mutex);
    const bool ok = msg.data_size() == 2 && std::isfinite(msg.data(0)) && std::isfinite(msg.data(1));
    left = ok ? std::clamp<double>(msg.data(0), -maxForce, maxForce) : 0;
    right = ok ? std::clamp<double>(msg.data(1), -maxForce, maxForce) : 0;
    received = Clock::now();
    stampPending = commanded = true;
  }
  // Every simulation step: republish the stored forces, or zero when the
  // command is stale or the simulation is paused.
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &) override {
    std::lock_guard<std::mutex> lock(mutex);
    if (stampPending) {
      receivedSim = info.simTime;
      stampPending = false;
    }
    // A simulation reset moves simTime before receivedSim: treat as stale.
    const double simAge = std::chrono::duration<double>(info.simTime - receivedSim).count();
    const bool valid = !info.paused && commanded && simAge >= 0 && simAge <= timeout &&
                       std::chrono::duration<double>(Clock::now()-received).count() <= liveness;
    gz::msgs::Double l, r;
    l.set_data(valid ? left : 0); r.set_data(valid ? right : 0);
    leftPub.Publish(l); rightPub.Publish(r);
  }
};
}
GZ_ADD_PLUGIN(njord::ActuatorWatchdog, gz::sim::System,
              njord::ActuatorWatchdog::ISystemConfigure,
              njord::ActuatorWatchdog::ISystemPreUpdate)
