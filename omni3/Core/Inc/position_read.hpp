#pragma once

#include <param.hpp>
#include <tr/prelude.hpp>

// Lightweight POD for head (wheel) state used by state estimation
struct HeadState {
    Qty<Radian> angle = 0_rad;                 // radians
    Qty<RadianPerSecond> angvel = 0_radps;     // rad/s
};

// Read head states for all omni3 motors (ONE, TWO, THREE)
// Inline to avoid non-inline symbol definitions across TUs
inline EnumMap<mechs::omni3::Id, HeadState> read_head_states(mods::Dji<Fdcan>* dji) {
    EnumMap<mechs::omni3::Id, HeadState> res;
    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        const mods::dji::Id djid = OMNI3_TO_DJI[id];
        res[id].angle = dji->get_now_head_angle(djid).unwrap_or(0_rad);
        res[id].angvel = dji->get_now_head_angvel(djid).unwrap_or(0_radps);
    }
    return res;
}

// Global storage for latest read state (defined in code.cpp)
extern EnumMap<mechs::omni3::Id, HeadState> now_states_global;
