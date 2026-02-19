#pragma once


#include <tr/prelude.hpp>

namespace shared {
enum class TerunetId : uint8_t {
    PET,
    BELT,
    ARM,
    STATE    
};
enum class State : uint8_t {
    ESP_TIMEOUT,
    TERUNET_TIMEOUT
};
}
