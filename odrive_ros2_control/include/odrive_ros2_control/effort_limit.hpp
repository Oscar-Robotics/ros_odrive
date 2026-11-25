#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>

namespace odrive_ros2_control {

constexpr auto kEffortLimitInterfaceName = "effort_limit_ratio";

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

constexpr double kMaxEffortLimitRatio = 1.0;

inline std::optional<double> sanitize_effort_limit_ratio(double ratio) {
    if (!std::isfinite(ratio) || ratio <= 0.0) {
        return std::nullopt;
    }
    return std::min(ratio, kMaxEffortLimitRatio);
}

inline bool is_supported_firmware(uint8_t major, uint8_t minor, uint8_t revision) {
    return major == kSupportedFwMajor && minor == kSupportedFwMinor && revision == kSupportedFwRevision;
}

inline float compute_effort_limit_current(double ratio, float current_soft_max) {
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

struct EffortLimitCommand {
    float velocity_limit;
    float current_limit;
    double ratio;
    bool changed;
};

class EffortLimiter {
public:
    void activate(float vel_limit, float configured_current_soft_max) {
        vel_limit_ = vel_limit;
        configured_current_soft_max_ = configured_current_soft_max;
        enabled_ = true;
        resend_ = false;
        sent_ratio_ = std::numeric_limits<double>::quiet_NaN();
    }

    void deactivate() {
        enabled_ = false;
        resend_ = false;
        sent_ratio_ = std::numeric_limits<double>::quiet_NaN();
    }

    bool enabled() const { return enabled_; }

    void request_resend() { resend_ = true; }

    std::optional<EffortLimitCommand> next(double commanded_ratio) {
        if (!enabled_) {
            return std::nullopt;
        }
        const auto ratio = sanitize_effort_limit_ratio(commanded_ratio);
        if (!ratio) {
            return std::nullopt;
        }
        const bool changed = *ratio != sent_ratio_;
        if (!changed && !resend_) {
            return std::nullopt;
        }
        return build(*ratio, changed);
    }

    std::optional<EffortLimitCommand> release() {
        if (!enabled_) {
            return std::nullopt;
        }
        return build(kMaxEffortLimitRatio, true);
    }

private:
    EffortLimitCommand build(double ratio, bool changed) {
        sent_ratio_ = ratio;
        resend_ = false;
        return EffortLimitCommand{
            vel_limit_, compute_effort_limit_current(ratio, configured_current_soft_max_), ratio, changed};
    }

    float vel_limit_ = std::numeric_limits<float>::quiet_NaN();
    float configured_current_soft_max_ = std::numeric_limits<float>::quiet_NaN();
    bool enabled_ = false;
    bool resend_ = false;
    double sent_ratio_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace odrive_ros2_control
