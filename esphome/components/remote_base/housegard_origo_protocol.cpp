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
  // Encode device ID (8 bits)
  for (uint8_t i = 0; i < 8; i++) {
    this->encode_bit_(dst, data.device & (1 << i));
  }

  // Encode high bits (19 bits)
  for (uint8_t i = 0; i < 19; i++) {
    this->encode_bit_(dst, data.sequence_highbits & (1 << i));
  }

  // Encode low bits (24 bits)
  for (uint8_t i = 0; i < 24; i++) {
    this->encode_bit_(dst, data.sequence_lowbits & (1 << i));
  }
}

optional<HousegardOrigoData> HousegardOrigoProtocol::decode(RemoteReceiveData src) {
  HousegardOrigoData data{};

  // Need at least SEQUENCE_LEN * 2 items for mark/space pairs
  if (src.size() < SEQUENCE_LEN * 2)
    return {};

  // Validate initial timing
  if (!src.expect_item(NARROW_PULSE, NARROW_PULSE))
    return {};

  data.device = 0;
  data.sequence_highbits = 0;
  data.sequence_lowbits = 0;

  // Decode device ID (8 bits)
  for (uint8_t i = 0; i < 8; i++) {
    if (src.expect_item(NARROW_PULSE, NARROW_PULSE)) {
      data.device |= (1 << i);
    } else if (!src.expect_item(WIDE_PULSE, WIDE_PULSE)) {
      return {};
    }
  }

  // Decode high bits (19 bits)
  for (uint8_t i = 0; i < 19; i++) {
    if (src.expect_item(NARROW_PULSE, NARROW_PULSE)) {
      data.sequence_highbits |= (1 << i);
    } else if (!src.expect_item(WIDE_PULSE, WIDE_PULSE)) {
      return {};
    }
  }

  // Decode low bits (24 bits)
  for (uint8_t i = 0; i < 24; i++) {
    if (src.expect_item(NARROW_PULSE, NARROW_PULSE)) {
      data.sequence_lowbits |= (1 << i);
    } else if (!src.expect_item(WIDE_PULSE, WIDE_PULSE)) {
      return {};
    }
  }

  return data;
}

void HousegardOrigoProtocol::dump(const HousegardOrigoData &data) {
  ESP_LOGD(TAG, "Received Housegard Origo: device=0x%02X, sequence_high=0x%05X, sequence_low=0x%06X", data.device,
           data.sequence_highbits, data.sequence_lowbits);
}

}  // namespace remote_base
}  // namespace esphome
