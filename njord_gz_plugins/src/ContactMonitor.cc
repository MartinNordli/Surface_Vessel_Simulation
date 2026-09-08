// SPDX-License-Identifier: MIT
// Harmonic's stock contact sensor only emits nonempty messages. Publish a
// heartbeat from physics ContactSensorData so silence cannot imply no contact.
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
  std::set<std::string> expectedModels;
  std::set<std::pair<uint64_t, uint64_t>> collectedPairs;
  gz::msgs::Contacts pending;
  std::chrono::steady_clock::duration lastPublish{};

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
    if (info.simTime < lastPublish) {
      lastPublish = info.simTime;
      pending.Clear();
      collectedPairs.clear();
    }
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
