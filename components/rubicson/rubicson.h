/**
 * @file rubicson.h
 * @brief ESPHome decoder for Rubicson 433 MHz OOK-PWM thermometers.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome {
namespace rubicson {

class RubicsonComponent : public Component,
                          public remote_base::RemoteReceiverListener {
 public:
  // ── Setters called from generated Python code ─────────────────────────────
  void set_temperature_sensor(sensor::Sensor *s)          { temperature_sensor_ = s; }
  void set_battery_low_sensor(binary_sensor::BinarySensor *s) { battery_low_sensor_ = s; }

  /// Accept only packets from this 8-bit sensor ID.  Pass -1 to accept any.
  void set_sensor_id(int id) { sensor_id_ = id; }

  /// Accept only packets from this channel (1–3).  Pass -1 to accept any.
  void set_channel(int ch)   { channel_ = ch; }

  // ── ESPHome overrides ─────────────────────────────────────────────────────
  float get_setup_priority() const override { return setup_priority::DATA; }

  /// Called by remote_receiver for every captured burst.
  bool on_receive(remote_base::RemoteReceiveData data) override;

 protected:
  sensor::Sensor              *temperature_sensor_{nullptr};
  binary_sensor::BinarySensor *battery_low_sensor_{nullptr};

  int sensor_id_{-1};  ///< -1 = accept any sensor ID
  int channel_{-1};    ///< -1 = accept any channel

  /// Attempt to decode one 36-bit Rubicson packet starting at raw[start].
  bool try_decode_(const remote_base::RawTimings &raw, size_t start);
};

} // namespace rubicson
} // namespace esphome
