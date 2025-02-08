#pragma once

#include "remote_base.h"

namespace esphome {
namespace remote_base {

struct HousegardOrigoData {
  uint32_t device;             // 8-bit device ID
  uint32_t sequence_highbits;  // Upper 19 bits of sequence
  uint32_t sequence_lowbits;   // Lower 32 bits of sequence
  bool operator==(const HousegardOrigoData &rhs) const {
    return device == rhs.device && sequence_highbits == rhs.sequence_highbits &&
           sequence_lowbits == rhs.sequence_lowbits;
  }
};

class HousegardOrigoProtocol : public RemoteProtocol<HousegardOrigoData> {
 public:
  void encode(RemoteTransmitData *dst, const HousegardOrigoData &data) override;
  optional<HousegardOrigoData> decode(RemoteReceiveData src) override;
  void dump(const HousegardOrigoData &data) override;

 protected:
  // Constants from reference implementation
  static const uint16_t NARROW_PULSE = 450;
  static const uint16_t WIDE_PULSE = 1250;
  static const uint8_t SEQUENCE_LEN = 51;  // 32 + 19 bits

  void encode_bit_(RemoteTransmitData *dst, bool value) const {
    if (value) {
      dst->mark(NARROW_PULSE);
      dst->space(NARROW_PULSE);
    } else {
      dst->mark(WIDE_PULSE);
      dst->space(WIDE_PULSE);
    }
  }
};

DECLARE_REMOTE_PROTOCOL(HousegardOrigo)

template<typename... Ts> class HousegardOrigoAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint32_t, device)
  TEMPLATABLE_VALUE(uint32_t, sequence_highbits)
  TEMPLATABLE_VALUE(uint32_t, sequence_lowbits)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    HousegardOrigoData data{};
    data.device = this->device_.value(x...);
    data.sequence_highbits = this->sequence_highbits_.value(x...);
    data.sequence_lowbits = this->sequence_lowbits_.value(x...);
    HousegardOrigoProtocol().encode(dst, data);
  }
};

}  // namespace remote_base
}  // namespace esphome
