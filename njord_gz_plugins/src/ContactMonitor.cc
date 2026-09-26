// SPDX-License-Identifier: MIT
// Harmonic's stock contact sensor only emits nonempty messages. Publish a
// heartbeat from physics ContactSensorData so silence cannot imply no contact.
//
// njord::ContactMonitor -- Gazebo world plugin added by njord_sim/scenario.py.
// Every course marker and obstacle model carries a contact sensor; this
// plugin gathers their contacts and publishes them together, empty or not.
//
// SDF parameters:
//   <marker>NAME</marker>  repeated; model names whose contacts are monitored
//
// Topic (Gazebo transport, bridged to ROS as ros_gz_interfaces/Contacts):
//   publishes /njord/contacts  gz.msgs.Contacts, at most every 50 ms of
//             simulation time, header stamp = simulation time
//
// Failure behavior: nothing is published until every listed marker has both
// a contact sensor and a physics-populated ContactSensorData component, so
// the evaluator's freshness check (evaluator_node.contact_fresh) fails and a
// run cannot be scored as collision-free without working contact sensing.
// Nothing is published while paused. A simulation-time reset clears the
// accumulated contacts.
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <gz/msgs/contacts.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace njord {
class ContactMonitor final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate {
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::set<std::string> expectedModels;  // from the <marker> elements
  // Unordered collision-id pairs already in `pending`, so a contact that
  // persists across several physics steps is reported once per heartbeat.
  std::set<std::pair<uint64_t, uint64_t>> collectedPairs;
  gz::msgs::Contacts pending;  // contacts accumulated since the last publish
  // Simulation time of the last publish (UpdateInfo::simTime uses the
  // steady_clock duration type, but the value is simulation time).
  std::chrono::steady_clock::duration lastPublish{};

  // Name of the model that owns `link` (the link's parent entity), or empty.

  std::string ModelName(const gz::sim::Entity link,
                       const gz::sim::EntityComponentManager &ecm) const {
    const auto parent = ecm.Component<gz::sim::components::ParentEntity>(link);
    if (!parent) return {};
    const auto name = ecm.Component<gz::sim::components::Name>(parent->Data());
    return name ? name->Data() : std::string{};
  }

 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    // Clone because Element iteration is mutable in sdformat's API.
    auto config = sdf->Clone();
    if (config->HasElement("marker")) {
      for (auto marker = config->GetElement("marker"); marker;
           marker = marker->GetNextElement("marker"))
        expectedModels.insert(marker->Get<std::string>());
    }
    publisher = node.Advertise<gz::msgs::Contacts>("/njord/contacts");
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || expectedModels.empty()) return;
    // Simulation time went backwards (world reset): start a new window.
    if (info.simTime < lastPublish) {
      lastPublish = info.simTime;
      pending.Clear();
      collectedPairs.clear();
    }
    // Which monitored models have a contact sensor entity, and which have
    // contact data populated by the physics system this step.
    std::set<std::string> sensors, dataModels;
    ecm.Each<gz::sim::components::ContactSensor, gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &, const gz::sim::components::ContactSensor *,
          const gz::sim::components::ParentEntity *parent) {
        const auto name = ModelName(parent->Data(), ecm);
        if (expectedModels.count(name)) sensors.insert(name);
        return true;
      });
    ecm.Each<gz::sim::components::Collision, gz::sim::components::ContactSensorData,
             gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &, const gz::sim::components::Collision *,
          const gz::sim::components::ContactSensorData *data,
          const gz::sim::components::ParentEntity *parent) {
        const auto name = ModelName(parent->Data(), ecm);
        if (!expectedModels.count(name)) return true;
        dataModels.insert(name);
        // Accumulate every physics step, even when the heartbeat is throttled.
        for (const auto &contact : data->Data().contact()) {
          const auto first = contact.collision1().id(), second = contact.collision2().id();
          if (collectedPairs.insert({std::min(first, second), std::max(first, second)}).second)
            pending.add_contact()->CopyFrom(contact);
        }
        return true;
      });
    // No heartbeat until every configured marker has an actual sensor and a
    // physics-populated contact component; never fabricate healthy sensor data.
    if (sensors != expectedModels || dataModels != expectedModels) {
      pending.Clear();
      collectedPairs.clear();
      return;
    }
    // Throttle to 20 Hz of simulation time; contacts keep accumulating.
    if (info.simTime - lastPublish < std::chrono::milliseconds(50)) return;
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - seconds);
    pending.mutable_header()->mutable_stamp()->set_sec(seconds.count());
    pending.mutable_header()->mutable_stamp()->set_nsec(nanos.count());
    publisher.Publish(pending);
    pending.Clear();
    collectedPairs.clear();
    lastPublish = info.simTime;
  }
};
}
GZ_ADD_PLUGIN(njord::ContactMonitor, gz::sim::System,
              njord::ContactMonitor::ISystemConfigure,
              njord::ContactMonitor::ISystemPostUpdate)
