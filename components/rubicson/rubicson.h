#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_receiver.h"

#include <vector>

namespace esphome::rubicson {

class RubicsonComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_remote_receiver(remote_base::RemoteReceiverComponent *recv);
  void set_temperature_sensor(sensor::Sensor *s);
  void set_battery_sensor(binary_sensor::BinarySensor *s);

 protected:
  remote_base::RemoteReceiverComponent *recv_{nullptr};

  sensor::Sensor *temperature_{nullptr};
  binary_sensor::BinarySensor *battery_{nullptr};

  static constexpr size_t REPEATS = 12;
  
  struct FrameBits {
    std::vector<int> bits;
  };

  std::vector<FrameBits> buffer_;

  bool decode_raw_(const std::vector<int32_t> &raw, FrameBits &out);
  bool build_voted_frame_(std::vector<int> &out_bits);
  int align_shift_(const std::vector<int> &ref, const std::vector<int> &cand);

  bool crc_ok_(uint64_t data);
};

}  // namespace esphome::rubicson
