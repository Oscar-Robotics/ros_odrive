#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "odrive_ros2_control/effort_limit.hpp"

using namespace odrive_ros2_control;

namespace {
constexpr double kTolerance = 1e-6;
constexpr double kHalfRatio = 0.5;
constexpr double kOverRatio = 1.7;
constexpr float kSoftMax = 1.5f;
constexpr float kReplyValue = 12.5f;
constexpr float kVelLimit = 10.0f;
constexpr double kDockRatio = 0.3;
constexpr int kActivationCycles = 5;
}

TEST(EffortLimitTest, RatioIsClampedToUnitInterval) {
    EXPECT_NEAR(*sanitize_effort_limit_ratio(kHalfRatio), kHalfRatio, kTolerance);
    EXPECT_NEAR(*sanitize_effort_limit_ratio(kOverRatio), kMaxEffortLimitRatio, kTolerance);
    EXPECT_NEAR(*sanitize_effort_limit_ratio(kMaxEffortLimitRatio), kMaxEffortLimitRatio, kTolerance);
}

TEST(EffortLimitTest, RatioRejectsNonPositiveAndNonFinite) {
    EXPECT_FALSE(sanitize_effort_limit_ratio(0.0).has_value());
    EXPECT_FALSE(sanitize_effort_limit_ratio(-kHalfRatio).has_value());
    EXPECT_FALSE(sanitize_effort_limit_ratio(NAN).has_value());
    EXPECT_FALSE(sanitize_effort_limit_ratio(INFINITY).has_value());
}

TEST(EffortLimitTest, OnlyFirmware0612IsSupported) {
    EXPECT_TRUE(is_supported_firmware(0, 6, 12));
    EXPECT_FALSE(is_supported_firmware(0, 6, 10));
    EXPECT_FALSE(is_supported_firmware(0, 6, 11));
    EXPECT_FALSE(is_supported_firmware(0, 5, 12));
    EXPECT_FALSE(is_supported_firmware(1, 6, 12));
}

TEST(EffortLimitTest, EffortLimitScalesConfiguredSoftMax) {
    EXPECT_NEAR(compute_effort_limit_current(kHalfRatio, kSoftMax), kHalfRatio * kSoftMax, kTolerance);
    EXPECT_NEAR(compute_effort_limit_current(kMaxEffortLimitRatio, kSoftMax), kSoftMax, kTolerance);
}

TEST(EffortLimitTest, SdoRequestEncodesLittleEndianEndpointId) {
    const auto data = encode_sdo_read_request(kVelLimitEndpointId);
    EXPECT_EQ(data[0], kSdoReadOpcode);
    EXPECT_EQ(data[1], kVelLimitEndpointId & 0xFF);
    EXPECT_EQ(data[2], kVelLimitEndpointId >> 8);
    EXPECT_EQ(data[3], 0);
}

TEST(EffortLimitTest, SdoReplyDecodesMatchingEndpointOnly) {
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

TEST(EffortLimitTest, ActivationCyclesNeverCompound) {
    EffortLimiter limiter;
    for (int cycle = 0; cycle < kActivationCycles; ++cycle) {
        limiter.activate(kVelLimit, kSoftMax);
        const auto command = limiter.next(kDockRatio);
        ASSERT_TRUE(command.has_value());
        EXPECT_NEAR(command->current_limit, kDockRatio * kSoftMax, kTolerance);
        EXPECT_NEAR(command->velocity_limit, kVelLimit, kTolerance);
        EXPECT_FALSE(limiter.next(kDockRatio).has_value());

        const auto released = limiter.release();
        ASSERT_TRUE(released.has_value());
        EXPECT_NEAR(released->current_limit, kSoftMax, kTolerance);
        limiter.deactivate();
        EXPECT_FALSE(limiter.release().has_value());
        EXPECT_FALSE(limiter.next(kDockRatio).has_value());
    }
}

TEST(EffortLimitTest, ReactivationAtFullRatioSendsConfiguredValue) {
    EffortLimiter limiter;
    limiter.activate(kVelLimit, kSoftMax);
    ASSERT_TRUE(limiter.next(kDockRatio).has_value());
    limiter.deactivate();
    limiter.activate(kVelLimit, kSoftMax);
    const auto command = limiter.next(kMaxEffortLimitRatio);
    ASSERT_TRUE(command.has_value());
    EXPECT_NEAR(command->current_limit, kSoftMax, kTolerance);
}

TEST(EffortLimitTest, ResendAfterHeartbeatLossKeepsSameLimit) {
    EffortLimiter limiter;
    limiter.activate(kVelLimit, kSoftMax);
    ASSERT_TRUE(limiter.next(kDockRatio).has_value());
    limiter.request_resend();
    const auto command = limiter.next(kDockRatio);
    ASSERT_TRUE(command.has_value());
    EXPECT_FALSE(command->changed);
    EXPECT_NEAR(command->current_limit, kDockRatio * kSoftMax, kTolerance);
    EXPECT_FALSE(limiter.next(kDockRatio).has_value());
}

TEST(EffortLimitTest, InvalidCommandedRatioSendsNothing) {
    EffortLimiter limiter;
    limiter.activate(kVelLimit, kSoftMax);
    EXPECT_FALSE(limiter.next(NAN).has_value());
    EXPECT_FALSE(limiter.next(0.0).has_value());
}
