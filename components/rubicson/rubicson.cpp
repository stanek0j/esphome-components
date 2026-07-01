/**
 * @file rubicson.cpp
 * @brief Rubicson 433 MHz thermometer decoder — ESPHome external component.
 *
 * Protocol decoding is identical to rtl_433 "Rubicson-Temperature".
 *
 * Compatible hardware
 * ───────────────────
 *   Rubicson (all variants), Esic WT 450H, Holman WS0201,
 *   TFA 30.3197, UPM WT450 / WT260
 *
 * Physical layer
 * ──────────────
 *   Carrier   : 433.92 MHz OOK/ASK
 *   Encoding  : PWM — mark duration carries the bit value
 *                 short mark  ≈  500 µs  → logical 0
 *                 long  mark  ≈ 1000 µs  → logical 1
 *               inter-bit space ≈ 500 µs
 *   Repetition: sensor broadcasts 3–12 identical packets per update
 *   Gap       : > 8 ms between repetitions (rtl_433 reset_limit)
 *
 * 36-bit packet layout (bit 0 = first received = MSB of byte 0)
 * ──────────────────────────────────────────────────────────────
 *   bits  0– 7  byte[0]  Sensor ID  (random; re-randomised on battery swap)
 *   bit       8  byte[1]  battery_ok   1 = OK, 0 = replace
 *   bit       9  byte[1]  don't-care
 *   bits 10–11  byte[1]  Channel  0–3
 *   bits 12–15  byte[1]  Temperature[11:8]   high nibble
 *   bits 16–23  byte[2]  Temperature[7:0]    low byte
 *   bits 24–31  byte[3]  Unknown / checksum source byte
 *   bits 32–35  byte[4]  Unused tail nibble
 *
 * Checksum (rtl_433 convention)
 * ──────────────────────────────
 *   (byte[0] ^ byte[1] ^ byte[2] ^ byte[3]) & 0x0F == 0x0F
 *
 * Temperature
 * ───────────
 *   12-bit signed two's-complement integer, divide by 10 → °C
 *   Decoded with the exact bit manipulation used in rtl_433:
 *     temp = (int16_t)(((byte[1] & 0x0F) << 12) | (byte[2] << 4)) >> 4
 */

#include "rubicson.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rubicson {

static const char *const TAG = "rubicson";

// ── Pulse timing windows (µs) ─────────────────────────────────────────────────
//
//  rtl_433 reference values: short_width = 500 µs, long_width = 1000 µs
//
//  The ±50 % windows absorb:
//    • ESPHome remote_receiver ISR jitter  (can be tens of µs on ESP32/8266)
//    • Rubicson crystal tolerance          (typically ±0.5 %)
//    • CC1101 demodulator pulse distortion
//
//  The midpoint 750 µs cleanly separates the two bit classes.
//
static constexpr int32_t kMarkShortMin = 200;   // minimum valid short mark  → bit 0
static constexpr int32_t kMarkBoundary = 750;   // everything < 750 → 0, ≥ 750 → 1
static constexpr int32_t kMarkLongMax  = 1500;  // maximum valid long  mark  → bit 1

//  Inter-bit space is nominally the same duration as a short mark (≈ 500 µs).
//  Upper bound must stay well below the 8 ms inter-packet gap so we never
//  mistake a packet boundary for an inter-bit gap.
static constexpr int32_t kGapMin = 100;
static constexpr int32_t kGapMax = 1600;

static constexpr size_t kBits       = 36;
// Minimum raw timings needed: 36 marks + 35 spaces (last trailing space
// may be absent when the buffer ends exactly at the packet boundary).
static constexpr size_t kMinRawSize = kBits * 2 - 1;

// ─────────────────────────────────────────────────────────────────────────────
//  try_decode_()  —  attempt to parse one Rubicson packet at raw[start]
// ─────────────────────────────────────────────────────────────────────────────

bool RubicsonComponent::try_decode_(const remote_base::RawTimings &raw,
                                    size_t start) {
    if (raw.size() < start + kMinRawSize)
        return false;

    // Accumulate decoded bits here.
    // 36 bits fit in 5 bytes; byte[4] only uses its upper nibble.
    uint8_t bytes[5] = {};

    for (size_t bit = 0; bit < kBits; ++bit) {
        const size_t mark_pos  = start + bit * 2u;
        const size_t space_pos = mark_pos + 1u;

        // ── Mark ─────────────────────────────────────────────────────────────
        // In ESPHome raw timings, marks are positive integers (µs).
        const int32_t mark = raw[mark_pos];

        if (mark <= 0)
            return false;   // Got a space where a mark was expected

        uint8_t bval;
        if (mark >= kMarkShortMin && mark < kMarkBoundary) {
            bval = 0;
        } else if (mark >= kMarkBoundary && mark <= kMarkLongMax) {
            bval = 1;
        } else {
            // Duration outside both Rubicson windows — not our protocol
            return false;
        }

        // Pack bit MSB-first:
        //   bit 0  → bit 7 of bytes[0]
        //   bit 7  → bit 0 of bytes[0]
        //   bit 8  → bit 7 of bytes[1]  … etc.
        bytes[bit >> 3u] |= static_cast<uint8_t>(bval << (7u - (bit & 7u)));

        // ── Space ────────────────────────────────────────────────────────────
        // Spaces are negative integers in ESPHome raw timings.
        // Skip validation after the very last bit (space may be missing).
        if (bit < kBits - 1u) {
            if (space_pos >= raw.size())
                return false;

            const int32_t sp = raw[space_pos];
            if (sp >= 0)
                return false;               // Expected space, got mark

            const int32_t gap = -sp;        // Make positive for comparison
            if (gap < kGapMin || gap > kGapMax)
                return false;               // Implausible inter-bit gap
        }
    }

    // ── Checksum ──────────────────────────────────────────────────────────────
    // XOR the low nibbles of the first four bytes.  Must equal 0x0F.
    // (Identical to rtl_433 rubicson_callback checksum.)
    const uint8_t crc_nibble =
        (bytes[0] ^ bytes[1] ^ bytes[2] ^ bytes[3]) & 0x0Fu;

    if (crc_nibble != 0x0Fu) {
        ESP_LOGV(TAG,
                 "Checksum FAIL  bytes=%02X %02X %02X %02X  crc_nibble=0x%X (want 0xF)",
                 bytes[0], bytes[1], bytes[2], bytes[3], crc_nibble);
        return false;
    }

    // ── Field extraction ──────────────────────────────────────────────────────
    const uint8_t sensor_id  = bytes[0];
    const bool    battery_ok = (bytes[1] & 0x80u) != 0u;
    const uint8_t channel    = (bytes[1] & 0x30u) >> 4u;

    // 12-bit signed temperature × 10 = °C
    //
    // Exact reproduction of rtl_433's sign-extension trick:
    //
    //   temp_raw = (int16_t)(((b[1] & 0x0F) << 12) | (b[2] << 4)) >> 4
    //
    // How it works:
    //   1. Shift the 12 temperature bits into the top 12 positions of a
    //      uint16_t  (bits [15:4]).
    //   2. Reinterpret as int16_t so that bit 15 becomes the sign bit.
    //   3. Arithmetic-right-shift by 4 to place the value in bits [11:0]
    //      with correct sign extension into bits [15:12].
    //
    // Note: right-shifting a negative int16_t is implementation-defined by
    // the C++ standard, but GCC (which builds all ESP firmware) guarantees
    // arithmetic shift for signed integers on ARM/Xtensa — matching rtl_433.
    //
    const uint16_t packed =
        (static_cast<uint16_t>(bytes[1] & 0x0Fu) << 12u) |
        (static_cast<uint16_t>(bytes[2])          <<  4u);

    const int16_t temp_raw = static_cast<int16_t>(packed) >> 4;
    const float   temp_c   = static_cast<float>(temp_raw) / 10.0f;

    // ── Optional packet filters ───────────────────────────────────────────────
    // sensor_id_ == -1 or channel_ == -1 means "accept any".
    if (sensor_id_ != -1 &&
        sensor_id != static_cast<uint8_t>(sensor_id_)) {
        ESP_LOGV(TAG, "Filtered: sensor ID 0x%02X  (want 0x%02X)",
                 sensor_id, static_cast<uint8_t>(sensor_id_));
        return false;
    }
    if (channel_ != -1 &&
        channel != static_cast<uint8_t>(channel_)) {
        ESP_LOGV(TAG, "Filtered: channel %u  (want %u)",
                 channel, static_cast<uint8_t>(channel_));
        return false;
    }

    // ── Publish ───────────────────────────────────────────────────────────────
    ESP_LOGD(TAG,
             "Rubicson decoded — ID=0x%02X  ch=%u  battery=%s  temp=%.1f °C",
             sensor_id, channel,
             battery_ok ? "OK" : "LOW",
             temp_c);

    if (temperature_sensor_ != nullptr)
        temperature_sensor_->publish_state(temp_c);

    // The ESPHome binary_sensor for battery LOW is the inverse of battery_ok
    // so it reads true when the battery actually needs replacing.
    if (battery_low_sensor_ != nullptr)
        battery_low_sensor_->publish_state(!battery_ok);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  on_receive()  —  RemoteReceiverListener entry point
// ─────────────────────────────────────────────────────────────────────────────

bool RubicsonComponent::on_receive(remote_base::RemoteReceiveData data) {
    const remote_base::RawTimings &raw = data.get_raw_data();

    if (raw.size() < kMinRawSize)
        return false;

    bool found = false;

    // We scan the *entire* raw buffer rather than assuming the packet starts
    // at position 0.  Three reasons:
    //
    //  1. The buffer might open with a trailing space from a previous burst
    //     that the ISR clipped.
    //
    //  2. If remote_receiver's `idle` timeout is longer than the ≥8 ms
    //     inter-packet gap, multiple repetitions land in one buffer.
    //     Scanning lets us decode all of them and return as soon as we find
    //     the first valid one (or confirm agreement across repeats).
    //
    //  3. A partially-received first repetition won't poison the scan;
    //     try_decode_() will reject it at the first bad timing or checksum
    //     mismatch and the scan resumes at the next candidate position.
    //
    for (size_t i = 0; i + kMinRawSize <= raw.size(); ++i) {
        // Fast pre-filter: only attempt decode from positions that carry a
        // mark (positive) whose duration is within the Rubicson mark window.
        // This skips spaces and obviously wrong marks in O(1) per position,
        // keeping the scan cheap for the typical remote_receiver buffer.
        const int32_t v = raw[i];
        if (v <= 0 || v < kMarkShortMin || v > kMarkLongMax)
            continue;

        if (try_decode_(raw, i)) {
            found = true;
            // Jump past the decoded packet.
            // Subtract 1 because the loop's ++i will advance one more step.
            i += kBits * 2u - 1u;
        }
    }

    // Return true when at least one valid Rubicson packet was decoded.
    // This signals to the remote_receiver that the burst was consumed and
    // prevents lower-priority listeners from wasting time on the same data.
    return found;
}

} // namespace rubicson
} // namespace esphome
