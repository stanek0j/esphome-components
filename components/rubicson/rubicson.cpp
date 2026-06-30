#include "rubicson.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::rubicson {

static const char *TAG = "rubicson";

void RubicsonComponent::setup() {
  ESP_LOGI(TAG, "Rubicson receiver initialized");
}

void RubicsonComponent::loop() {
  if (!this->recv_) return;

  auto raw = this->recv_->read_raw();
  if (raw.empty()) return;

  Frame frame;
  if (!this->decode_raw_(raw, frame)) return;
  if (!frame.valid) return;

  // --- FIELD EXTRACTION (rtl_433 logic port) ---

  uint8_t id = (frame.data >> 24) & 0xFF;
  uint8_t flags = (frame.data >> 16) & 0xFF;
  bool battery_ok = flags & 0x80;
  uint8_t channel = ((flags >> 4) & 0x03) + 1;

  int16_t temp_raw = frame.data & 0x0FFF;
  if (temp_raw & 0x0800) temp_raw |= 0xF000;

  float temp_c = temp_raw * 0.1f;

  if (this->temperature_ != nullptr) {
    this->temperature_->publish_state(temp_c);
  }

  if (this->battery_ != nullptr) {
    this->battery_->publish_state(battery_ok);
  }

  ESP_LOGD(TAG, "id=%d ch=%d temp=%.1f battery=%d",
           id, channel, temp_c, battery_ok);
}

void RubicsonComponent::dump_config() {
  ESP_LOGCONFIG("rubicson", "Rubicson loaded");
}

// ------------------------------------------------------------
// RAW → BIT DECODE (PPM threshold model)
// ------------------------------------------------------------

bool RubicsonComponent::decode_raw_(const std::vector<int32_t> &raw, Frame &out) {
  std::vector<int> bits;
  bits.reserve(64);

  for (size_t i = 0; i + 1 < raw.size(); i += 2) {
    int pulse = raw[i];
    int gap = raw[i + 1];

    // PPM thresholding (based on your rtl_433 analysis)
    if (gap > 1500) {
      bits.push_back(1);
    } else {
      bits.push_back(0);
    }
  }

  return this->decode_bits_(bits, out);
}

// ------------------------------------------------------------
// BITSTREAM → FRAME
// ------------------------------------------------------------

bool RubicsonComponent::decode_bits_(const std::vector<int> &bits, Frame &out) {
  if (bits.size() < 36) return false;

  uint64_t data = 0;

  for (int i = 0; i < 36; i++) {
    data <<= 1;
    data |= bits[i] & 1;
  }

  if (!this->crc_ok_(data)) return false;

  out.data = data;
  out.bits = 36;
  out.valid = true;
  return true;
}

// ------------------------------------------------------------
// CRC8 (0x31, init 0x6c)
// ------------------------------------------------------------

bool RubicsonComponent::crc_ok_(uint64_t data) {
  uint8_t crc = 0x6c;

  for (int i = 0; i < 5; i++) {
    uint8_t b = (data >> (i * 8)) & 0xFF;
    crc ^= b;

    for (int j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }

  return crc == 0;
}

bool RubicsonComponent::align_and_vote_(
    const std::vector<std::vector<int>> &packets,
    std::vector<int> &out_bits) {

  if (packets.size() < 3) return false;  // need repeats

  size_t max_len = 0;
  for (auto &p : packets)
    max_len = std::max(max_len, p.size());

  out_bits.assign(max_len, 0);

  // ---------------------------
  // 1. MAJORITY VOTE PER BIT
  // ---------------------------
  for (size_t i = 0; i < max_len; i++) {
    int ones = 0;
    int zeros = 0;

    for (auto &p : packets) {
      if (i < p.size()) {
        if (p[i]) ones++;
        else zeros++;
      }
    }

    out_bits[i] = (ones > zeros) ? 1 : 0;
  }

  return true;
}

}  // namespace esphome::rubicson
