// Core includes
#include <main.h>
#include <code.hpp>

// Project headers
#include <cstdint>
#include <param.hpp>
#include <shared.hpp>
#include <tr/prelude.hpp>
#include <optional>

// Local helper: keep HeadState and read function inside this translation unit
// to avoid cross-TU symbol duplication and make code self-contained.
struct HeadState {
    Qty<Radian> angle = 0_rad;
    Qty<RadianPerSecond> angvel = 0_radps;
};

static EnumMap<mechs::omni3::Id, HeadState> read_head_states_local(mods::Dji<Fdcan>* dji) {
    EnumMap<mechs::omni3::Id, HeadState> res;
    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        const mods::dji::Id djid = OMNI3_TO_DJI[id];
        res[id].angle = dji->get_now_head_angle(djid).unwrap_or(0_rad);
        res[id].angvel = dji->get_now_head_angvel(djid).unwrap_or(0_radps);
    }
    return res;
}



extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern I2C_HandleTypeDef hi2c3; 

bool initialized = false;

mods::Terunet3 *tn3; 

Qty<Radian> now_angle = 0_rad;
Qty<RadianPerSecond> now_angvel = 0_radps;
Qty<Radian> target_angle = 0_rad;


//imu

mods::Bno055* imu;
Qty<Radian> yaw;


mods::Dji<Fdcan> *dji; 


EnumMap<mechs::omni3::Id, HeadState> now_states_global = {};

static inline float head_angle_deg(const HeadState &s) {
    return s.angle.get_value() * 180.0F / 3.141592653589793F;
}

static inline Qty<Radian> normalize_angle(const Qty<Radian> a) {
    
    return a;
}

//omni3
mechs::omni3::Ik ik = mechs::omni3::Ik(OMNI3_CONFIG);
Transform2d target_transform = {0_mps, 0_mps, 0_radps};

Transform2d target_transform_max = {3_mps, 3_mps, 0_radps}; 

controllers::ControlPosition<Qty<Radian>, Qty<Ampere>>* cp;

EnumMap<mechs::omni3::Id, std::optional<ctls::ControlVelocity<Qty<Radian>, Qty<Ampere>>>> cvs; 

void get_terunet() {}
void set_terunet() {}


void setup() {
    disable_irq();

//imu
    imu = new mods::Bno055(new hardwares::I2cMaster(&hi2c3));

    tn3 = new mods::Terunet3(new Fdcan(&hfdcan1));

    timer::start_interrupt(&htim6);
    timer::start_interrupt(&htim7);

    dji = new mods::Dji<Fdcan>(new Fdcan(&hfdcan3), DJI_SAMPLING_PERIOD, DJI_DETAILS);

  
    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        cvs[id].emplace(TIMER_PERIOD, CV_PARAM, CV_CONFIG, true);
    }

    initialized = true;

    enable_irq();
} 

void loop() {
    get_terunet();

    //IMU
    imu ->rx();
    imu ->update();
    yaw = imu -> get_yaw();

    //omni3
    ik.set_heading(yaw); //imu
    ik.set_transform(target_transform);
    ik.update();


    now_states_global = read_head_states_local(dji);






    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        cvs[id]->set_target_velocity(ik.get_velocity(id) / WHEEL_RADIUS);
        cvs[id]->set_now_velocity(dji->get_now_head_angvel(OMNI3_TO_DJI[id]).unwrap());
        dji->set_target_current(OMNI3_TO_DJI[id], cvs[id]->get_output());
    }
    
    set_terunet();
} 

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (!initialized) {
        return;
    }

    // 演算用
    if (htim->Instance == TIM7) {
        for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
            cvs[id]->update();
            dji->set_target_current(OMNI3_TO_DJI[id], cvs[id]->get_output());
        }
        return;
    }

    // 通信用
    if (htim->Instance == TIM6) {
        dji->tx();
        tn3->tx();
        return;
    }
}
#endif 

#ifdef HAL_FDCAN_MODULE_ENABLED
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its) {
    if (!initialized) {
        return;
    }

    if (rx_fifo0_its == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        if (hfdcan->Instance == FDCAN1) {
            tn3->rx();
            return;
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo1_its) {
    if (!initialized) {
        return;
    }

    if (rx_fifo1_its == FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        if (hfdcan->Instance == FDCAN3) {
            dji->rx();
            return;
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (!initialized) {
        return;
    }

    if (htim->Instance == TIM1) {
        return;
    }
}
#endif
