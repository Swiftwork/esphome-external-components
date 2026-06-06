#pragma once

#include "remote_base.h"

namespace esphome::remote_base {

struct HousegardOrigoData {
  uint8_t device;        // 8-bit device ID
  uint64_t pairing_key;  // 45-bit pairing key
  bool operator==(const HousegardOrigoData &rhs) const {
    return device == rhs.device && pairing_key == rhs.pairing_key;
  }
};

class HousegardOrigoProtocol : public RemoteProtocol<HousegardOrigoData> {
 public:
  void encode(RemoteTransmitData *dst, const HousegardOrigoData &data) override;
  void encode_pairing(RemoteTransmitData *dst, const HousegardOrigoData &data);
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
  TEMPLATABLE_VALUE(uint64_t, pairing_key)
  TEMPLATABLE_VALUE(bool, is_pairing)

  void encode(RemoteTransmitData *dst, const Ts &...x) override {
    HousegardOrigoData data{};
    data.device = this->device_.value(x...);
    data.pairing_key = this->pairing_key_.value(x...);

    if (this->is_pairing_.value(x...)) {
      HousegardOrigoProtocol().encode_pairing(dst, data);
    } else {
      HousegardOrigoProtocol().encode(dst, data);
    }
  }
};

}  // namespace esphome::remote_base
