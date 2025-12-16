#pragma once

#include <main.h>

#include <code.hpp>
#include <tr/prelude.hpp>

constexpr Qty<Second> TIMER_PERIOD = 1 / 1_kHz;
constexpr Qty<Second> DJI_SAMPLING_PERIOD = 1 / 1_kHz;
