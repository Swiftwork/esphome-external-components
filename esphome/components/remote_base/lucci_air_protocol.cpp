#include "lucci_air_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace remote_base {

static const char *const TAG = "remote.lucci_air";

/**
 * The Lucci Air ceiling fan remote sends out signals in pairs.
 * Each button press sends 5 repetitions of command start, then 5 repetitions of command end.
 * Each signal consists of 81 bits where:
 * - A narrow pulse (290μs) represents a logical 0
 * - A wide pulse (875μs) represents a logical 1
 * - The signal is given in reverse order, so the least significant bit is sent first
 * - Between each pulse is 5000μs
 * - Between command pairs is 10000μs
 *
 * Signal structure:
 * - First 50 bits: Device ID
 * - Remaining 31 bits: Command
 *
 * Examples from a remote:
 * Direction: 011010101010101001101010101010 (0x1AAA9AAA) | Device: 10010101100110101010011010010110100101011001101010 (0x2566A9A5A566A)
 * Speed 1:   010110101010101001011010101010 (0x16AA96AA) | Device: 10010101100110101010011010010110100101011001101010 (0x2566A9A5A566A)
 * Power:     011001101010101001100110101010 (0x19AA99AA) | Device: 10010101100110101010011010010110100101011001101010 (0x2566A9A5A566A)
 * Light:     011001011010101001100101101010 (0x196A996A) | Device: 10010101100110101010011010010110100101011001101010 (0x2566A9A5A566A)
 * 
 * Encoded in reverse order (LSB first):
 * Light: 010101100110101001011010010110010101011001101010010101011010011001010101101001100
 */

static const uint8_t SEQUENCE_LEN = 81;    // 50 device ID + 31 command bits
static const uint16_t BIT_ZERO_US = 290;   // A narrow pulse signaling logical 0
static const uint16_t BIT_ONE_US = 875;    // A wide pulse signaling logical 1
static const uint16_t GAP_US = 5000;       // Gap between signals
static const uint16_t COMMAND_GAP_US = 10000; // Gap between command pairs

const std::map<std::string, uint32_t> LucciAirProtocol::COMMANDS = {
    {"direction", 0x1AAA9AAA},
    {"speed_1", 0x16AA96AA},
    {"speed_2", 0x29AAA9AA},
    {"speed_3", 0x2A6AAA6A},
    {"speed_4", 0x15AA95AA},
    {"speed_5", 0x25AAA5AA},
    {"speed_6", 0x26AAA6AA},
    {"power", 0x19AA99AA},
    {"timer_1h", 0x166A966A},
    {"timer_4h", 0x266AA66A},
    {"timer_8h", 0x1A6A9A6A},
    {"light", 0x196A996A},
    {"speed_cycle", 0x1A9A9A9A},
    {"away", 0x2A9AAA9A},
};

void LucciAirProtocol::encode_bit(RemoteTransmitData *dst, bool value, bool mark) const {
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

uint32_t LucciAirProtocol::get_command_start(const std::string &command) {
  auto it = COMMANDS.find(command);
  if (it != COMMANDS.end()) {
    return it->second;
  }
  return 0; // Unknown command
}

uint32_t LucciAirProtocol::get_command_end(const std::string &command) {
  uint32_t start = get_command_start(command);
  return start ^ COMMAND_END_MASK;
}

std::string LucciAirProtocol::get_command_name(uint32_t command_value) {
  for (const auto &pair : COMMANDS) {
    if (pair.second == command_value || (pair.second ^ COMMAND_END_MASK) == command_value) {
      return pair.first;
    }
  }
  return "unknown";
}

void LucciAirProtocol::encode_signal_with_command(RemoteTransmitData *dst, uint32_t command, uint64_t device_id) {

  // Encode Device ID bits (0-49)
  for (uint8_t i = 0; i < 50; i++) {
    this->encode_bit(dst, device_id & (1ULL << i), i % 2 == 0);
  }
  
  // Encode Command bits (50-80)
  for (uint8_t i = 0; i < 31; i++) {
    this->encode_bit(dst, command & (1UL << i), (50 + i) % 2 == 0);
  }
}

void LucciAirProtocol::encode(RemoteTransmitData *dst, const LucciAirData &data) {
  ESP_LOGD(TAG, "Encoding Lucci Air signal for command: %s", data.command.c_str());

  uint32_t command_start = get_command_start(data.command);
  uint32_t command_end = get_command_end(data.command);
  
  if (command_start == 0) {
    ESP_LOGE(TAG, "Unknown command: %s", data.command.c_str());
    return;
  }

  // Send 5 repetitions of command start
  for (uint8_t rep = 0; rep < 5; rep++) {
    if (rep > 0) {
      dst->space(GAP_US);
    }
    this->encode_signal_with_command(dst, command_start, data.device_id);
  }

  // Gap between command pairs
  dst->space(COMMAND_GAP_US);

  // Send 5 repetitions of command end
  for (uint8_t rep = 0; rep < 5; rep++) {
    if (rep > 0) {
      dst->space(GAP_US);
    }
    this->encode_signal_with_command(dst, command_end, data.device_id);
  }
}

optional<bool> LucciAirProtocol::decode_bit(RemoteReceiveData &src, bool is_mark, uint8_t bit_position) const {
  if (is_mark) {
    if (src.peek_mark(BIT_ONE_US, bit_position)) {
      return true;
    }
    if (src.peek_mark(BIT_ZERO_US, bit_position)) {
      return false;
    }
    ESP_LOGV(TAG, "Failed to decode mark at bit %d", bit_position);
    return {};
  } else {
    if (src.peek_space(BIT_ONE_US, bit_position)) {
      return true;
    }
    if (src.peek_space(BIT_ZERO_US, bit_position)) {
      return false;
    }
    ESP_LOGV(TAG, "Failed to decode space at bit %d", bit_position);
    return {};
  }
}

optional<std::pair<uint32_t, uint64_t>> LucciAirProtocol::decode_single_signal(RemoteReceiveData &src, size_t start_index) const {
  // Need at least SEQUENCE_LEN bits from start_index
  if (src.size() < start_index + SEQUENCE_LEN) {
    return {};
  }

  uint32_t raw_command = 0;
  uint64_t device_id = 0;

  // Decode all bits in the sequence
  for (uint8_t i = 0; i < SEQUENCE_LEN; i++) {
    size_t bit_index = start_index + i;
    bool is_mark = (i % 2 == 0);
    
    // Use peek methods with absolute index
    optional<bool> bit_result;
    if (is_mark) {
      if (src.peek_mark(BIT_ONE_US, bit_index)) {
        bit_result = true;
      } else if (src.peek_mark(BIT_ZERO_US, bit_index)) {
        bit_result = false;
      } else {
        // Failed to decode mark
        return {};
      }
    } else {
      if (src.peek_space(BIT_ONE_US, bit_index)) {
        bit_result = true;
      } else if (src.peek_space(BIT_ZERO_US, bit_index)) {
        bit_result = false;
      } else {
        // Failed to decode space
        return {};
      }
    }
    
    if (*bit_result) {
      // Store bit in appropriate field based on position
      if (i < 50) {
        // Device ID bits (0-49)
        device_id |= (1ULL << i);
      } else if (i < SEQUENCE_LEN) {
        // Command bits (50-80)
        raw_command |= (1UL << (i - 50));
      }
    }
  }

  return std::make_pair(raw_command, device_id);
}

optional<LucciAirData> LucciAirProtocol::decode(RemoteReceiveData src) {
  LucciAirData data{};

  // Decode first signal (start command)
  auto start_result = decode_single_signal(src, 0);
  if (!start_result.has_value()) {
    ESP_LOGV(TAG, "Failed to decode start command");
    return {};
  }

  uint32_t start_command = start_result->first;
  data.device_id = start_result->second;

  // Calculate expected end command
  uint32_t expected_end_command = start_command ^ COMMAND_END_MASK;

  // Look for the end command sequence
  // Search more comprehensively through the latter half of the signal
  // to account for timing variations and gaps
  
  size_t search_start = SEQUENCE_LEN * 2; // Start searching after first few signals
  size_t max_search_end = src.size() - SEQUENCE_LEN; // Don't go past the end
  
  bool found_end_command = false;
  ESP_LOGV(TAG, "Searching for end command 0x%X, device_id=0x%llX", expected_end_command, data.device_id);
  
  // Search with smaller increments to be more thorough
  for (size_t offset = search_start; offset <= max_search_end; offset += 10) {
    auto end_result = decode_single_signal(src, offset);
    if (end_result.has_value()) {
      uint32_t end_command = end_result->first;
      uint64_t end_device_id = end_result->second;
      
      ESP_LOGV(TAG, "Found signal at offset %zu: command=0x%X, device_id=0x%llX", 
               offset, end_command, end_device_id);
      
      // Check if this matches our expected end command and device ID
      if (end_command == expected_end_command && end_device_id == data.device_id) {
        found_end_command = true;
        ESP_LOGV(TAG, "Found matching end command at offset %zu", offset);
        break;
      }
    }
  }

  if (!found_end_command) {
    ESP_LOGV(TAG, "Failed to find matching end command for start=0x%X, expected_end=0x%X", 
             start_command, expected_end_command);
    return {};
  }

  // Convert start command to command name
  data.command = get_command_name(start_command);
  
  if (data.command == "unknown") {
    ESP_LOGV(TAG, "Unknown command: 0x%X", start_command);
    return {};
  }

  ESP_LOGD(TAG, "Successfully decoded Lucci Air command pair: %s", data.command.c_str());
  return data;
}

void LucciAirProtocol::dump(const LucciAirData &data) {
  ESP_LOGD(TAG, "Received Lucci Air: command=%s, device_id=0x%llX", data.command.c_str(), data.device_id);
}

}  // namespace remote_base
}  // namespace esphome 
