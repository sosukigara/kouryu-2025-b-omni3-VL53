#pragma once

#include <main.h>

#include <code.hpp>
#include <shared.hpp>
#include <tr/prelude.hpp>

constexpr Qty<Second> TIMER_PERIOD = 1_ms;

constexpr EnumMap<mods::software_timer::Channel, Option<Qty<Second>>> SW_TIMER_PERIODS = {
    {mods::software_timer::Channel::CHANNEL_1, 1_ms},
};

// TapeLED
constexpr uint8_t TAPELED_LENGTH = 100;
constexpr EnumMap<shared::State, Color> TAPELED_COLORS = {
    {shared::State::ESP_TIMEOUT, {255, 255, 255}},
    {shared::State::TERUNET_TIMEOUT, {255, 255, 255}}
};

enum class ServoState : uint8_t { OPEN, CLOSE };

// ペット回収
enum class PetServo : uint8_t { RIGHT, LEFT };

constexpr EnumMap<PetServo, uint32_t> PET_SERVO_CHANNEL = {
    {PetServo::RIGHT, TIM_CHANNEL_2}, {PetServo::LEFT, TIM_CHANNEL_3}
};

constexpr EnumMap<PetServo, EnumMap<ServoState, Qty<Radian>>> PET_SERVO_ANGLE = {
    {PetServo::RIGHT, {{ServoState::OPEN, 30_deg}, {ServoState::CLOSE, 90_deg}}},
    {PetServo::LEFT, {{ServoState::OPEN, 90_deg}, {ServoState::CLOSE, 30_deg}}}
};

// アーム
constexpr EnumMap<ServoState, Qty<Radian>> ARM_SERVO_ANGLE = {
    {ServoState::OPEN, 0_deg}, {ServoState::CLOSE, 0_deg}
};

// 射出
constexpr Qty<Meter> BELT_LENGTH = 560.0_mm;
constexpr Qty<Meter> BELT_RADIUS = 13.53_mm;
constexpr Qty<MeterPerSecondSquared> BELT_MAX_ACC = 53_mps2;

constexpr EnumMap<mods::dji::Id, Option<mods::dji::Detail>> DJI_DETAILS = {
    {mods::dji::Id::ONE, mods::dji::Detail::M3508()}
};

ctls::control_position::Param BELT_CP_PARAM = {
    .position_pid_param = {.kp = 140, .ki = 0, .kd = 0.01},
    .velocity_pid_param = {.kp = 0.3, .ki = 0, .kd = 0}
};

constexpr ctls::control_position::Config<Qty<Radian>, Qty<Ampere>> BELT_CP_CONFIG = {
    .output_range = {-30_A, 30_A},
    .position_range = {none, none},
    .velocity_range = {none, none},
    .acceleration_range = {-BELT_MAX_ACC / BELT_RADIUS, BELT_MAX_ACC / BELT_RADIUS}
};