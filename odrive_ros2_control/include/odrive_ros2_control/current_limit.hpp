#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>

namespace odrive_ros2_control {

constexpr auto kCurrentLimitInterfaceName = "current_limit_ratio";

constexpr uint8_t kSupportedFwMajor = 0;
constexpr uint8_t kSupportedFwMinor = 6;
constexpr uint8_t kSupportedFwRevision = 12;

constexpr uint16_t kVelLimitEndpointId = 379;
constexpr uint16_t kCurrentSoftMaxEndpointId = 308;

constexpr uint8_t kSdoReadOpcode = 0x00;
constexpr uint8_t kRxSdoCmdId = 0x04;
constexpr uint8_t kTxSdoCmdId = 0x05;
constexpr uint8_t kGetVersionCmdId = 0x00;

constexpr size_t kSdoRequestLength = 4;
constexpr size_t kSdoReplyLength = 8;
constexpr size_t kSdoEndpointIdOffset = 1;
constexpr size_t kSdoValueOffset = 4;

constexpr double kMaxCurrentLimitRatio = 1.0;

inline std::optional<double> sanitize_current_limit_ratio(double ratio) {
    if (!std::isfinite(ratio) || ratio <= 0.0) {
        return std::nullopt;
    }
    return std::min(ratio, kMaxCurrentLimitRatio);
}

inline bool is_supported_firmware(uint8_t major, uint8_t minor, uint8_t revision) {
    return major == kSupportedFwMajor && minor == kSupportedFwMinor && revision == kSupportedFwRevision;
}

inline float compute_current_limit(double ratio, float current_soft_max) {
    return static_cast<float>(ratio * current_soft_max);
}

inline std::array<uint8_t, kSdoRequestLength> encode_sdo_read_request(uint16_t endpoint_id) {
    std::array<uint8_t, kSdoRequestLength> data{};
    data[0] = kSdoReadOpcode;
    data[kSdoEndpointIdOffset] = static_cast<uint8_t>(endpoint_id & 0xFF);
    data[kSdoEndpointIdOffset + 1] = static_cast<uint8_t>(endpoint_id >> 8);
    return data;
}

inline std::optional<float> decode_sdo_float_reply(const uint8_t* data, size_t length, uint16_t expected_endpoint_id) {
    if (length < kSdoReplyLength) {
        return std::nullopt;
    }
    const uint16_t endpoint_id = static_cast<uint16_t>(data[kSdoEndpointIdOffset])
                               | static_cast<uint16_t>(data[kSdoEndpointIdOffset + 1] << 8);
    if (endpoint_id != expected_endpoint_id) {
        return std::nullopt;
    }
    float value;
    std::memcpy(&value, data + kSdoValueOffset, sizeof(value));
    return value;
}

} // namespace odrive_ros2_control
