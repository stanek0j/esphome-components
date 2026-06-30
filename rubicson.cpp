#include "rubicson.h"
#include "esphome/core/log.h"

namespace esphome::rubicson {

static const char *TAG = "rubicson";

/* -----------------------------
 * Device key helper
 * ----------------------------- */
uint32_t RubicsonComponent::key_(uint8_t id, uint8_t channel) {
  return (uint32_t(id) << 8) | channel;
}

/* -----------------------------
 * Setup
 * ----------------------------- */
void RubicsonComponent::setup() {
  ESP_LOGI(TAG, "Rubicson receiver initialized");
}

void RubicsonComponent::loop() {
  // nothing needed (event-driven via remote_receiver)
}


/* -----------------------------
 * Pulse decoding helpers
 * ----------------------------- */

static inline bool is_short(uint32_t t) { return t > 80 && t < 200; }
static inline bool is_long(uint32_t t)  { return t > 350 && t < 650; }

static inline bool is_gap0(uint32_t t)  { return t > 800 && t < 1300; }
static inline bool is_gap1(uint32_t t)  { return t > 1600 && t < 2400; }

/* -----------------------------
 * Decode RF frame → logical frame
 * ----------------------------- */
bool RubicsonComponent::decode_(remote_base::RemoteReceiveData data, Frame &out) {
  std::vector<int> bits;

  while (data.expect_more()) {
    auto pulse = data.pop_pulse();
    auto gap = data.pop_gap();

    if (!pulse.has_value() || !gap.has_value())
      break;

    uint32_t p = pulse->duration;
    uint32_t g = gap->duration;

    // sync rejection
    if (p < 200 || p > 800)
      continue;

    if (g > 800 && g < 1300) {
      bits.push_back(0);
    } else if (g > 1600 && g < 2400) {
      bits.push_back(1);
    } else if (g > 3000) {
      // reset / frame boundary
      if (bits.size() >= 36)
        break;
      else
        bits.clear();
    }

    if (bits.size() > 64)
      break;
  }

  if (bits.size() < 36)
    return false;

  // -----------------------------
  // frame extraction
  // -----------------------------
  int offset = bits.size() - 36;

  uint8_t b[5] = {0};

  for (int i = 0; i < 36; i++) {
    b[i / 8] <<= 1;
    b[i / 8] |= bits[offset + i];
  }

  // -----------------------------
  // CRC (rtl_433 compatible)
  // -----------------------------
  uint8_t tmp[5];
  tmp[0] = b[0];
  tmp[1] = b[1];
  tmp[2] = b[2];
  tmp[3] = b[3] & 0xF0;
  tmp[4] = ((b[3] & 0x0F) << 4) | (b[4] >> 4);

  auto crc8 = [](uint8_t *data, int len) -> uint8_t {
    uint8_t crc = 0x6C;
    for (int i = 0; i < len; i++) {
      crc ^= data[i];
      for (int j = 0; j < 8; j++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
      }
    }
    return crc;
  };

  if (crc8(tmp, 5) != 0)
    return false;

  // -----------------------------
  // decode fields
  // -----------------------------
  out.id = b[0];
  out.battery_ok = (b[1] & 0x80) != 0;
  out.channel = ((b[1] & 0x30) >> 4) + 1;

  int16_t temp_raw = (int16_t)((b[1] << 12) | (b[2] << 4));
  out.temperature = (temp_raw >> 4) * 0.1f;

  return true;
}

/* -----------------------------
 * RF frame entry point
 * ----------------------------- */
bool RubicsonComponent::on_receive(remote_base::RemoteReceiveData data) {
  Frame frame;

  if (!decode_(data, frame))
    return false;

  process_frame_(frame);
  return true;
}

/* -----------------------------
 * Frame routing
 * ----------------------------- */
void RubicsonComponent::process_frame_(const Frame &frame) {
  uint32_t key = key_(frame.id, frame.channel);
  auto &dev = devices_[key];

  update_device_(dev, frame);
}

/* -----------------------------
 * Device update (burst buffer)
 * ----------------------------- */
void RubicsonComponent::update_device_(DeviceState &dev, const Frame &frame) {
  uint32_t now = millis();

  // reset if stale
  if (now - dev.last_update > 5000) {
    dev.frames.clear();
    dev.vote_score = 0;
  }

  dev.last_update = now;
  dev.frames.push_back(frame);

  if (dev.frames.size() > 12)
    dev.frames.erase(dev.frames.begin());

  flush_device_(dev, 0);
}

/* -----------------------------
 * Burst stabilization + publish
 * ----------------------------- */
void RubicsonComponent::flush_device_(DeviceState &dev, uint32_t key) {
  if (dev.frames.size() < 6)
    return;

  Frame best;
  int best_score = 0;

  for (auto &f : dev.frames) {
    int score = 0;

    for (auto &g : dev.frames) {
      if (f.id == g.id &&
          f.channel == g.channel &&
          fabs(f.temperature - g.temperature) < 0.3f) {
        score++;
      }
    }

    if (score > best_score) {
      best_score = score;
      best = f;
    }
  }

  if (best_score < 3)
    return;

  if (!isnan(dev.last_temperature)) {
    if (fabs(best.temperature - dev.last_temperature) < 0.2f)
      return;
  }

  dev.last_temperature = best.temperature;
  dev.vote_score = best_score;

  ESP_LOGI(TAG,
           "DEV id=%u ch=%u temp=%.1f bat=%d votes=%d",
           best.id,
           best.channel,
           best.temperature,
           best.battery_ok,
           best_score);

  // -----------------------------
  // publish temperature
  // -----------------------------
  if (dev.temperature_sensor != nullptr) {
    dev.temperature_sensor->publish_state(best.temperature);
  } else if (default_temperature_sensor_ != nullptr) {
    default_temperature_sensor_->publish_state(best.temperature);
  }

  // -----------------------------
  // publish battery (binary sensor)
  // -----------------------------
  bool battery_low = !best.battery_ok;

  if (dev.battery_sensor != nullptr) {
    dev.battery_sensor->publish_state(battery_low);
  } else if (default_battery_sensor_ != nullptr) {
    default_battery_sensor_->publish_state(battery_low);
  }
}

}  // namespace esphome::rubicson
