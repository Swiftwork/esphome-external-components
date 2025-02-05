#include "housegard_origo_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.housegard_origo";

static const uint8_t SEQUENCE_LEN = 51;    // Total bits in sequence
static const uint32_t NARROW_PULSE = 450;  // A narrow pulse signaling logical 1
static const uint32_t WIDE_PULSE = 1250;   // A wide pulse signaling logical 0
static const uint32_t NARROW_TOLERANCE = 80;
static const uint32_t WIDE_TOLERANCE = 100;

void HousegardOrigoProtocol::encode_bit_(RemoteTransmitData *dst, bool value) const {
  if (value) {
    // Narrow pulse = 1
    dst->item(NARROW_PULSE, NARROW_PULSE);
  } else {
    // Wide pulse = 0
    dst->item(WIDE_PULSE, WIDE_PULSE);
  }
}

void HousegardOrigoProtocol::encode(RemoteTransmitData *dst, const HousegardOrigoData &data) {
  dst->set_carrier_frequency(0);

  // First encode the 32-bit sequence_lowbits (including 8-bit device ID)
  uint32_t lowbits = data.sequence_lowbits | (data.device & 0xFF);
  for (uint8_t i = 0; i < 32; i++) {
    this->encode_bit_(dst, lowbits & (1 << i));
  }

  // Then encode the 19-bit sequence_highbits
  for (uint8_t i = 0; i < 19; i++) {
    this->encode_bit_(dst, data.sequence_highbits & (1 << i));
  }
}

optional<HousegardOrigoData> HousegardOrigoProtocol::decode(RemoteReceiveData src) {
  HousegardOrigoData data{
      .device = 0,
      .sequence_highbits = 0,
      .sequence_lowbits = 0,
  };

  // Validate length
  if (src.size() < SEQUENCE_LEN * 2)
    return {};

  uint32_t buffer[2] = {0, 0};  // Like in original code
  uint8_t seq_pos = 0;

  // Process all pulses
  for (uint8_t i = 0; i < SEQUENCE_LEN; i++) {
    // Each bit is represented by a high and low pulse
    uint32_t high = src[i * 2];
    uint32_t low = -src[i * 2 + 1];  // Convert from negative to positive

    // Validate pulse widths
    bool is_narrow_high = high >= (NARROW_PULSE - NARROW_TOLERANCE) && high <= (NARROW_PULSE + NARROW_TOLERANCE);
    bool is_narrow_low = low >= (NARROW_PULSE - NARROW_TOLERANCE) && low <= (NARROW_PULSE + NARROW_TOLERANCE);
    bool is_wide_high = high >= (WIDE_PULSE - WIDE_TOLERANCE) && high <= (WIDE_PULSE + WIDE_TOLERANCE);
    bool is_wide_low = low >= (WIDE_PULSE - WIDE_TOLERANCE) && low <= (WIDE_PULSE + WIDE_TOLERANCE);

    // Both pulses should be either narrow (1) or wide (0)
    bool is_one = is_narrow_high && is_narrow_low;
    bool is_zero = is_wide_high && is_wide_low;

    if (!is_one && !is_zero)
      return {};

    // Add bit to appropriate buffer
    uint8_t byte_index = seq_pos > 31 ? 1 : 0;
    uint8_t bit_index = seq_pos - byte_index * 32;
    if (is_one) {
      buffer[byte_index] |= ((uint32_t) 1) << bit_index;
    }
    seq_pos++;
  }

  // Extract data
  data.device = buffer[0] & 0xFF;             // 8 lowest bits
  data.sequence_lowbits = buffer[0] & ~0xFF;  // Rest of lower 32 bits
  data.sequence_highbits = buffer[1];         // Upper 19 bits

  return data;
}

void HousegardOrigoProtocol::dump(const HousegardOrigoData &data) {
  ESP_LOGD(TAG, "Received Housegard Origo: device=0x%02X sequence_high=0x%05X sequence_low=0x%08X", data.device,
           data.sequence_highbits, data.sequence_lowbits);
}

}  // namespace remote_base
}  // namespace esphome
