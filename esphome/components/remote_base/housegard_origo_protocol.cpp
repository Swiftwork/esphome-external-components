#include "housegard_origo_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.housegard_origo";

/**
 * The Housegard Origo Smoke Detector sends out a signals 8 times in succession.
 * Each signal consists of 53 pulses where:
 * - A narrow pulse (450μs) represents a logical 1
 * - A wide pulse (1250μs) represents a logical 0
 *
 * Signal structure:
 * - First 8 bits (in reverse order): Device ID
 *   When device is unpaired, ID is typically 0xAA (10101010)
 * - Remaining bits (in reverse order): Pairing key
 *   Split into low (24 bits) and high (19 bits) to avoid 64-bit integers
 * - The signal is padded with zeroes at the end, hence only 51 bits are required
 *
 * Protocol research and analysis by https://github.com/fredilarsen/OrigoSmokeDetector
 *
 * An example signal could be:
 *
 * 01100101 011010100110100101010110 010101010101101000000
 *    ID            Low bits               High bits
 *
 * In reverse order, the signal is:
 * 10100110 = 0xA6 = Device ID
 * 011010101001011001010110 = 0x6A6956 = Low bits
 * 010101010101101000000 = 0x5AAA = High bits
 */

static const uint8_t SEQUENCE_LEN = 51;    // Minimum bits in sequence
static const uint32_t BIT_ONE_US = 450;    // A narrow pulse signaling logical 1
static const uint32_t BIT_ZERO_US = 1250;  // A wide pulse signaling logical 0
static const uint32_t BIT_ONE_TOLERANCE_US = 80;
static const uint32_t BIT_ZERO_TOLERANCE_US = 100;

void HousegardOrigoProtocol::encode_bit(RemoteTransmitData *dst, bool value, bool mark) const {
  if (value) {
    // Narrow pulse = 1
    if (mark) {
      dst->mark(BIT_ONE_US);
    } else {
      dst->space(BIT_ONE_US);
    }
  } else {
    // Wide pulse = 0
    if (mark) {
      dst->mark(BIT_ZERO_US);
    } else {
      dst->space(BIT_ZERO_US);
    }
  }
}

void HousegardOrigoProtocol::encode(RemoteTransmitData *dst, const HousegardOrigoData &data) {
  ESP_LOGD(TAG, "Encoding Housegard Origo signal...");

  // Encode device ID (8 bits)
  for (uint8_t i = 0; i < 8; i++) {
    this->encode_bit(dst, data.device & (1 << i), i % 2 == 0);
  }

  // Encode low bits (24 bits)
  for (uint8_t i = 0; i < 24; i++) {
    this->encode_bit(dst, data.sequence_lowbits & (1 << i), i % 2 == 0);
  }

  // Encode high bits (19 bits)
  for (uint8_t i = 0; i < 19; i++) {
    this->encode_bit(dst, data.sequence_highbits & (1 << i), i % 2 == 0);
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
  data.sequence_highbits = 0;
  data.sequence_lowbits = 0;

  // Decode device ID (8 bits)
  for (uint8_t i = 0; i < 8; i++) {
    bool is_mark = (i % 2 == 0);
    bool is_one;
    if (is_mark) {
      is_one = src.expect_mark(BIT_ONE_US);
      if (!is_one && !src.expect_mark(BIT_ZERO_US)) {
        ESP_LOGV(TAG, "Failed to decode device ID mark at bit %d", i);
        return {};
      }
    } else {
      is_one = src.expect_space(BIT_ONE_US);
      if (!is_one && !src.expect_space(BIT_ZERO_US)) {
        ESP_LOGV(TAG, "Failed to decode device ID space at bit %d", i);
        return {};
      }
    }
    if (is_one) {
      data.device |= (1 << i);
    }
  }

  // Decode low bits (24 bits)
  for (uint8_t i = 0; i < 24; i++) {
    bool is_mark = (i % 2 == 0);
    bool is_one;
    if (is_mark) {
      is_one = src.expect_mark(BIT_ONE_US);
      if (!is_one && !src.expect_mark(BIT_ZERO_US)) {
        ESP_LOGV(TAG, "Failed to decode low bits mark at bit %d", i);
        return {};
      }
    } else {
      is_one = src.expect_space(BIT_ONE_US);
      if (!is_one && !src.expect_space(BIT_ZERO_US)) {
        ESP_LOGV(TAG, "Failed to decode low bits space at bit %d", i);
        return {};
      }
    }
    if (is_one) {
      data.sequence_lowbits |= (1 << i);
    }
  }

  // Decode high bits (19 bits)
  for (uint8_t i = 0; i < 19; i++) {
    bool is_mark = (i % 2 == 0);
    bool is_one;
    if (is_mark) {
      is_one = src.expect_mark(BIT_ONE_US);
      if (!is_one && !src.expect_mark(BIT_ZERO_US)) {
        ESP_LOGV(TAG, "Failed to decode high bits mark at bit %d", i);
        return {};
      }
    } else {
      is_one = src.expect_space(BIT_ONE_US);
      if (!is_one && !src.expect_space(BIT_ZERO_US)) {
        ESP_LOGV(TAG, "Failed to decode high bits space at bit %d", i);
        return {};
      }
    }
    if (is_one) {
      data.sequence_highbits |= (1 << i);
    }
  }

  ESP_LOGV(TAG, "Successfully decoded Housegard Origo signal!");
  return data;
}

void HousegardOrigoProtocol::dump(const HousegardOrigoData &data) {
  ESP_LOGD(TAG, "Received Housegard Origo: device=0x%02X, sequence_high=0x%06X, sequence_low=0x%05X", data.device,
           data.sequence_lowbits, data.sequence_highbits);
}

}  // namespace remote_base
}  // namespace esphome
