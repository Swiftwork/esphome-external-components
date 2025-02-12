#include "housegard_origo_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.housegard_origo";

/**
 * The Housegard Origo Smoke Detector sends out a signals 8 times in succession.
 * Each signal consists of 53 pulses where:
 * - A narrow pulse (450μs) represents a logical 0
 * - A wide pulse (1250μs) represents a logical 1
 * - The signal is given in reverse order, so the least significant bit is sent first
 *
 * Signal structure:
 * - First 8 bits: Device ID
 *   When device is unpaired, ID is typically 0xAA (10101010)
 * - Remaining bits: Pairing key
 * - The signal is padded with zeroes at the end
 *
 * Protocol research and analysis by https://github.com/fredilarsen/OrigoSmokeDetector
 *
 * An example signal could be:
 *
 * 01100101 011010100110100101010110010101010101101000000
 *    ID                     Pairing key
 *
 * In reverse order, the signal is:
 * 10100110 = 0x6A = Device ID
 * 000000101101010101010011010101001011001010110 = 0x5AAA6A9656 = Pairing Key
 */

static const uint8_t SEQUENCE_LEN = 53;   // Minimum bits in sequence
static const uint16_t BIT_ZERO_US = 450;  // A narrow pulse signaling logical 0
static const uint16_t BIT_ONE_US = 1250;  // A wide pulse signaling logical 1

void HousegardOrigoProtocol::encode_bit(RemoteTransmitData *dst, bool value, bool mark) const {
  if (value) {
    // Wide pulse = 1
    if (mark) {
      dst->mark(BIT_ONE_US);
    } else {
      dst->space(BIT_ONE_US);
    }
  } else {
    // Narrow pulse = 0
    if (mark) {
      dst->mark(BIT_ZERO_US);
    } else {
      dst->space(BIT_ZERO_US);
    }
  }
}

void HousegardOrigoProtocol::encode(RemoteTransmitData *dst, const HousegardOrigoData &data) {
  ESP_LOGD(TAG, "Encoding Housegard Origo signal...");

  // Encode Device ID bits (0-7)
  for (uint8_t i = 0; i < 8; i++) {
    this->encode_bit(dst, data.device & (1 << i), i % 2 == 0);
  }

  // Encode Pairing Key bits (8-end)
  for (uint8_t i = 0; i < SEQUENCE_LEN - 8; i++) {
    this->encode_bit(dst, data.pairing_key & (1 << i), i % 2 == 0);
  }
}

optional<bool> HousegardOrigoProtocol::decode_bit(RemoteReceiveData &src, bool is_mark, uint8_t bit_position) const {
  if (is_mark) {
    if (src.expect_mark(BIT_ONE_US)) {
      return true;
    }
    if (src.expect_mark(BIT_ZERO_US)) {
      return false;
    }
    ESP_LOGV(TAG, "Failed to decode mark at bit %d", bit_position);
    return {};
  } else {
    if (src.expect_space(BIT_ONE_US)) {
      return true;
    }
    if (src.expect_space(BIT_ZERO_US)) {
      return false;
    }
    ESP_LOGV(TAG, "Failed to decode space at bit %d", bit_position);
    return {};
  }
}

optional<HousegardOrigoData> HousegardOrigoProtocol::decode(RemoteReceiveData src) {
  ESP_LOGV(TAG, "Attempting to decode Housegard Origo signal... (size=%d)", src.size());

  HousegardOrigoData data{};

  // Need at least SEQUENCE_LEN
  if (src.size() < SEQUENCE_LEN) {
    ESP_LOGV(TAG, "Signal too short for Housegard Origo protocol (%d < %d)", src.size(), SEQUENCE_LEN);
    return {};
  }

  data.device = 0;
  data.pairing_key = 0;

  // Decode all bits in the sequence
  for (uint8_t i = 0; i < SEQUENCE_LEN; i++) {
    bool is_mark = (i % 2 == 0);
    auto bit_result = decode_bit(src, is_mark, i);
    if (!bit_result.has_value()) {
      return {};
    }
    if (!*bit_result) {
      continue;
    }

    // Store bit in appropriate field based on position
    if (i < 8) {
      // Device ID bits (0-7)
      data.device |= (1 << i);
    } else if (i < SEQUENCE_LEN) {
      // Pairing Key bits (8-end)
      data.pairing_key |= (1 << (i - 8));
    }
  }

  ESP_LOGV(TAG, "Successfully decoded Housegard Origo signal!");
  return data;
}

void HousegardOrigoProtocol::dump(const HousegardOrigoData &data) {
  ESP_LOGD(TAG, "Received Housegard Origo: device=0x%02X, pairing_key=0x%llX", data.device, data.pairing_key);
}

}  // namespace remote_base
}  // namespace esphome
