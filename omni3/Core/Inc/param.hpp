#pragma once

#include <main.h>

#include <code.hpp>
#include <tr/prelude.hpp>

#include "tr/controllers/control_velocity/control_velocity.hpp"
#include "tr/messages/enum_map/enum_map.hpp"
#include "tr/messages/option/option.hpp"
#include "tr/messages/units/units.hpp"

// #include "tr/messages/vector_3d/vector_3d.hpp"
#include "tr/modules/dji/dji.hpp"

constexpr Qty<Second> DJI_SAMPLING_PERIOD = 1 / 1_kHz;
constexpr Qty<Second> TIMER_PERIOD_TWO = 1 / 1_kHz;
Transform2d target_transform_max = {1.5_mps, 1.5_mps, 4.5_radps};

constexpr EnumMap<mechs::omni3::Id, mods::dji::Id> OMNI3_TO_DJI = {
    {mechs::omni3::Id::ONE, mods::dji::Id::ONE},
    {mechs::omni3::Id::TWO, mods::dji::Id::TWO},
    {mechs::omni3::Id::THREE, mods::dji::Id::THREE},
};
constexpr EnumMap<mods::dji::Id, Option<mods::dji::Detail>> DJI_DETAILS = {
    {mods::dji::Id::ONE, mods::dji::Detail::M2006()},
    {mods::dji::Id::TWO, mods::dji::Detail::M2006()},
    {mods::dji::Id::THREE, mods::dji::Detail::M2006()},
};

constexpr Qty<Second> TIMER_PERIOD = 1 / 1_kHz;

// いらない
//  constexpr EnumMap<modules::dji::Id, Option<modules::dji::Detail>> dji_kinds_fd3 = {
//      {modules::dji::Id::ONE, modules::dji::Detail::M2006()}

// };

// constexpr tr::controllers::control_velocity::Config<Qty<Radian>,Qty<Ampere>> cv_config_fd3 = {
//     .output_range = {-10_A, 10_A},
//     .velocity_range = {-50_radps, 50_radps},   //目標角度
//     .acceleration_range = {-20_radps2, 20_radps2} //最大加速度
// };

constexpr mechs::omni3::Config OMNI3_CONFIG = {
    .base_radius = 20_cm,
};

constexpr ctls::control_velocity::Config<Qty<Radian>, Qty<Ampere>> CV_CONFIG = {
    .output_range = {-10_A, 10_A},
    .velocity_range = {none, none},
    .acceleration_range = {-35_radps2, 35_radps2},
    //.acceleration_range = {none, none},
};

constexpr tr::controllers::control_velocity::Param CV_PARAM = {
    .pid_param = {
        .kp = 1, .ki = 0, .kd = 0

    }
};

constexpr Qty<Meter> WHEEL_RADIUS = 0.05_m;
