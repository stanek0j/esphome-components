#include "rubicson.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::rubicson {

static const char *TAG = "rubicson";

void RubicsonComponent::setup() {
  ESP_LOGI(TAG, "Rubicson receiver initialized");
}

void RubicsonComponent::set_remote_receiver(remote_receiver::RemoteReceiverComponent *recv) {
  this->recv_ = recv;
}

void RubicsonComponent::loop() {
  if (!this->recv_) return;

  auto raw = this->recv_->read_raw();
  if (raw.empty()) return;

  FrameBits fb;
  if (!this->decode_raw_(raw, fb)) return;

  buffer_.push_back(fb);

  if (buffer_.size() < REPEATS) return;

  std::vector<int> bits;
  if (!this->build_voted_frame_(bits)) {
    buffer_.clear();
    return;
  }

  buffer_.clear();

  if (bits.size() < 36) return;

  uint64_t data = 0;
  for (size_t i = 0; i < 36; i++) {
    data <<= 1;
    data |= bits[i];
  }

  if (!this->crc_ok_(data)) return;

  uint8_t id = (data >> 24) & 0xFF;
  uint8_t flags = (data >> 16) & 0xFF;

  bool battery_ok = flags & 0x80;
  uint8_t channel = ((flags >> 4) & 0x03) + 1;

  int16_t temp_raw = data & 0x0FFF;
  if (temp_raw & 0x0800) temp_raw |= 0xF000;

  float temp_c = temp_raw * 0.1f;

  if (this->temperature_)
    this->temperature_->publish_state(temp_c);

  if (this->battery_)
    this->battery_->publish_state(battery_ok);

  ESP_LOGD(TAG, "id=%d ch=%d temp=%.1f battery=%d",
           id, channel, temp_c, battery_ok);
}

bool RubicsonComponent::decode_raw_(const std::vector<int32_t> &raw,
                                    FrameBits &out) {
  std::vector<int> bits;

  for (size_t i = 0; i + 1 < raw.size(); i += 2) {
    int gap = raw[i + 1];

    // PPM threshold from rtl_433 analysis
    bits.push_back(gap > 1500 ? 1 : 0);
  }

  out.bits = bits;
  return bits.size() >= 36;
}

// ------------------------------------------------------------
// Majority voting + sync alignment
// ------------------------------------------------------------

bool RubicsonComponent::build_voted_frame_(std::vector<int> &out_bits) {
  if (buffer_.empty()) return false;

  size_t max_len = 0;
  for (auto &b : buffer_)
    max_len = std::max(max_len, b.bits.size());

  std::vector<int> ref = buffer_[0].bits;

  // find best alignment for all frames vs reference
  std::vector<std::vector<int>> aligned;

  for (auto &b : buffer_) {
    int shift = align_shift_(ref, b.bits);

    std::vector<int> shifted(max_len, 0);

    for (size_t i = 0; i < b.bits.size(); i++) {
      int j = (int)i + shift;
      if (j >= 0 && j < (int)max_len)
        shifted[j] = b.bits[i];
    }

    aligned.push_back(shifted);
  }

  out_bits.assign(max_len, 0);

  for (size_t i = 0; i < max_len; i++) {
    int ones = 0, zeros = 0;

    for (auto &a : aligned) {
      if (a[i]) ones++;
      else zeros++;
    }

    out_bits[i] = (ones > zeros);
  }

  return true;
}

int RubicsonComponent::align_shift_(const std::vector<int> &ref,
                                     const std::vector<int> &cand) {
  int best_shift = 0;
  int best_score = -1;

  for (int shift = -4; shift <= 4; shift++) {
    int score = 0;

    for (size_t i = 0; i < ref.size(); i++) {
      int j = (int)i + shift;
      if (j < 0 || j >= (int)cand.size()) continue;

      if (ref[i] == cand[j]) score++;
    }

    if (score > best_score) {
      best_score = score;
      best_shift = shift;
    }
  }

  return best_shift;
}

// ------------------------------------------------------------
// CRC-8 (poly 0x31, init 0x6c)
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

}  // namespace esphome::rubicson
