#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_base.h"

#include <map>
#include <vector>

namespace esphome::rubicson {

class RubicsonComponent : public Component,
                           public remote_base::RemoteReceiverListener {
 public:
  void setup() override;
  void loop() override;

  bool on_receive(remote_base::RemoteReceiveData data) override;

  void set_temperature_sensor(sensor::Sensor *sensor) {
    default_temperature_sensor_ = sensor;
  }

  void set_battery_sensor(binary_sensor::BinarySensor *sensor) {
    default_battery_sensor_ = sensor;
  }

 protected:

  struct Frame {
    uint8_t id = 0;
    uint8_t channel = 0;
    bool battery_ok = false;
    float temperature = 0.0f;
  };

  struct DeviceState {
    std::vector<Frame> frames;
    float last_temperature = NAN;
    uint32_t last_update = 0;
    int vote_score = 0;

    sensor::Sensor *temperature_sensor = nullptr;
    binary_sensor::BinarySensor *battery_sensor = nullptr;
  };

  std::map<uint32_t, DeviceState> devices_;

  sensor::Sensor *default_temperature_sensor_{nullptr};
  binary_sensor::BinarySensor *default_battery_sensor_{nullptr};

  uint32_t key_(uint8_t id, uint8_t channel);

  bool decode_(remote_base::RemoteReceiveData data, Frame &out);

  void process_frame_(const Frame &frame);
  void update_device_(DeviceState &dev, const Frame &frame);
  void flush_device_(DeviceState &dev, uint32_t key);
};

}  // namespace esphome::rubicson
