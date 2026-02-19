#pragma once


#include <tr/prelude.hpp>
#include "../vl53l0x/vl53l0x.hpp"

namespace shared {
enum class TerunetId : uint8_t {
    PET,
    BELT,
    ARM,
    STATE    
};
}
