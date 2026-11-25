#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "odrive_ros2_control/current_limit.hpp"

using namespace odrive_ros2_control;

namespace {
constexpr double kTolerance = 1e-6;
constexpr double kHalfRatio = 0.5;
constexpr double kOverRatio = 1.7;
constexpr float kSoftMax = 1.5f;
constexpr float kReplyValue = 12.5f;
}

TEST(CurrentLimitTest, RatioIsClampedToUnitInterval) {
    EXPECT_NEAR(*sanitize_current_limit_ratio(kHalfRatio), kHalfRatio, kTolerance);
    EXPECT_NEAR(*sanitize_current_limit_ratio(kOverRatio), kMaxCurrentLimitRatio, kTolerance);
    EXPECT_NEAR(*sanitize_current_limit_ratio(kMaxCurrentLimitRatio), kMaxCurrentLimitRatio, kTolerance);
}

TEST(CurrentLimitTest, RatioRejectsNonPositiveAndNonFinite) {
    EXPECT_FALSE(sanitize_current_limit_ratio(0.0).has_value());
    EXPECT_FALSE(sanitize_current_limit_ratio(-kHalfRatio).has_value());
    EXPECT_FALSE(sanitize_current_limit_ratio(NAN).has_value());
    EXPECT_FALSE(sanitize_current_limit_ratio(INFINITY).has_value());
}

TEST(CurrentLimitTest, OnlyFirmware0612IsSupported) {
    EXPECT_TRUE(is_supported_firmware(0, 6, 12));
    EXPECT_FALSE(is_supported_firmware(0, 6, 10));
    EXPECT_FALSE(is_supported_firmware(0, 6, 11));
    EXPECT_FALSE(is_supported_firmware(0, 5, 12));
    EXPECT_FALSE(is_supported_firmware(1, 6, 12));
}

TEST(CurrentLimitTest, CurrentLimitScalesSoftMax) {
    EXPECT_NEAR(compute_current_limit(kHalfRatio, kSoftMax), kHalfRatio * kSoftMax, kTolerance);
    EXPECT_NEAR(compute_current_limit(kMaxCurrentLimitRatio, kSoftMax), kSoftMax, kTolerance);
}

TEST(CurrentLimitTest, SdoRequestEncodesLittleEndianEndpointId) {
    const auto data = encode_sdo_read_request(kVelLimitEndpointId);
    EXPECT_EQ(data[0], kSdoReadOpcode);
    EXPECT_EQ(data[1], kVelLimitEndpointId & 0xFF);
    EXPECT_EQ(data[2], kVelLimitEndpointId >> 8);
    EXPECT_EQ(data[3], 0);
}

TEST(CurrentLimitTest, SdoReplyDecodesMatchingEndpointOnly) {
    uint8_t frame[kSdoReplyLength] = {};
    const auto request = encode_sdo_read_request(kCurrentSoftMaxEndpointId);
    std::memcpy(frame, request.data(), request.size());
    std::memcpy(frame + kSdoValueOffset, &kReplyValue, sizeof(kReplyValue));

    const auto value = decode_sdo_float_reply(frame, sizeof(frame), kCurrentSoftMaxEndpointId);
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, kReplyValue, kTolerance);
    EXPECT_FALSE(decode_sdo_float_reply(frame, sizeof(frame), kVelLimitEndpointId).has_value());
    EXPECT_FALSE(decode_sdo_float_reply(frame, kSdoReplyLength - 1, kCurrentSoftMaxEndpointId).has_value());
}
