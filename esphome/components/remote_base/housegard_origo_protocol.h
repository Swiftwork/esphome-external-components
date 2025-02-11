#pragma once

#include "remote_base.h"

namespace esphome {
namespace remote_base {

struct HousegardOrigoData {
  uint8_t device;     // 8-bit device ID
  uint32_t lowbits;   // Lower 32 bits of sequence
  uint32_t highbits;  // Upper 19 bits of sequence
  bool operator==(const HousegardOrigoData &rhs) const {
    return device == rhs.device && lowbits == rhs.lowbits && highbits == rhs.highbits;
  }
};

class HousegardOrigoProtocol : public RemoteProtocol<HousegardOrigoData> {
 public:
  void encode(RemoteTransmitData *dst, const HousegardOrigoData &data) override;
  optional<HousegardOrigoData> decode(RemoteReceiveData src) override;
  void dump(const HousegardOrigoData &data) override;

 private:
  void encode_bit(RemoteTransmitData *dst, bool value, bool mark) const;
  optional<bool> decode_bit(RemoteReceiveData &src, bool is_mark, uint8_t bit_position) const;
};

DECLARE_REMOTE_PROTOCOL(HousegardOrigo)

template<typename... Ts> class HousegardOrigoAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint8_t, device)
  TEMPLATABLE_VALUE(uint32_t, highbits)
  TEMPLATABLE_VALUE(uint32_t, lowbits)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    HousegardOrigoData data{};
    data.device = this->device_.value(x...);
    data.lowbits = this->lowbits_.value(x...);
    data.highbits = this->highbits_.value(x...);
    HousegardOrigoProtocol().encode(dst, data);
  }
};

}  // namespace remote_base
}  // namespace esphome
