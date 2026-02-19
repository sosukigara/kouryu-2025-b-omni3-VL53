
#include <main.h>

#include <cmath>
#include <code.hpp>
#include <cstdint>
#include <param.hpp>
#include <shared.hpp>
#include <tr/controllers/sync_control_velocity/sync_control_velocity.hpp>
#include <tr/prelude.hpp>

#include "../../../vl53l0x/vl53l0x.hpp"
#include "tr/utilities/angles/angles.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

extern UART_HandleTypeDef huart1;

mods::Terunet3<shared::TerunetId>* tn3;

bool initialized = false;

volatile uint8_t gpio_ctl_pa2 = 0;
volatile uint8_t gpio_ctl_pa3 = 0;
volatile uint8_t gpio_ctl_pa4 = 0;
// espdbt
mods::Espdbt* espdbt;
mods::espdbt::State joy;

mods::Bno055* imu;
Qty<Radian> yaw;

mods::Dji* dji;

volatile uint32_t heartbeat = 0;
volatile int setup_step = 0;
volatile uint8_t found_addr = 0;
volatile uint32_t input_ignore_until = 0;
volatile uint32_t motor_output_inhibit_until = 0;

mods::Vl53l0x* vl53_1;
mods::Vl53l0x* vl53_2;
volatile bool vl53_1_ok = false;
volatile bool vl53_2_ok = false;
volatile float vl53_1_distance_mm = -1.0f;
volatile float vl53_2_distance_mm = -1.0f;
volatile float vl53_1_raw_m = 0.0f;
volatile float vl53_2_raw_m = 0.0f;
volatile int vl53_1_last_hal_status = 0;
volatile int vl53_2_last_hal_status = 0;
volatile uint32_t vl53_1_i2c_error = 0;
volatile uint32_t vl53_2_i2c_error = 0;
volatile uint32_t vl53_1_i2c_isr = 0;
volatile uint32_t vl53_2_i2c_isr = 0;
volatile bool vl53_1_found_at_default = false;
volatile bool vl53_2_found_at_default = false;
volatile float vl53_1_emergency_mm = 0.0f;
volatile float vl53_2_emergency_mm = 0.0f;
volatile bool vl53_1_emergency_init_done = false;
volatile bool vl53_2_emergency_init_done = false;
volatile bool vl53_error_reset = false;
volatile uint32_t vl53_1_last_update = 0;
volatile uint32_t vl53_2_last_update = 0;
volatile uint32_t vl53_retry_count = 0;

volatile bool imu_updated = false;

volatile float vl53_target_dist_mm = 300.0f;
volatile float vl53_tolerance_mm = 30.0f;
volatile float vl53_p_gain = 0.003f;
volatile float vl53_max_speed_mps = 0.6f;
volatile float vl53_dir_x = -1.0f;
volatile float vl53_dir_y = 0.0f;
volatile float heading_offset_rad = 0.523599f;

// omni3
mechs::omni3::Ik* ik;
Transform2d target_transform = {0_mps, 0_mps, 0_radps};

tr::controllers::SyncControlVelocity<mechs::omni3::Id, Qty<Radian>, Qty<Ampere>>* cvs;

tr::hardwares::Timer* timer6;
tr::hardwares::Timer* timer7;

void get_terunet() {}
void set_terunet() {}
void try_vl53_init();

void setup() {
    disable_irq();

    tn3 = new mods::Terunet3<shared::TerunetId>(new Fdcan(&hfdcan1));

    ik = new mechs::omni3::Ik(OMNI3_CONFIG);

    vl53_1 = new mods::Vl53l0x(&hi2c2);
    vl53_2 = new mods::Vl53l0x(&hi2c2);

    espdbt = new mods::Espdbt(new hardwares::Uart(&huart1));

    timer6 = new tr::hardwares::Timer(&htim6, true);
    timer7 = new tr::hardwares::Timer(&htim7, true);

    dji = new mods::Dji(new Fdcan(&hfdcan3), DJI_SAMPLING_PERIOD, DJI_DETAILS);

    cvs = new tr::controllers::SyncControlVelocity<mechs::omni3::Id, Qty<Radian>, Qty<Ampere>>(
        TIMER_PERIOD, CV_PARAM, CV_CONFIG, true
    );

    initialized = true;

    enable_irq();

    HAL_Delay(100);
    imu = new mods::Bno055(new hardwares::I2cMaster(&hi2c3));

    setup_step = 1;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    try_vl53_init();
}

void try_vl53_init() {
    vl53_retry_count = vl53_retry_count + 1;
    setup_step = 2;

    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_Init(&hi2c2);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(10);

    vl53_1->set_local_address(0x29);
    if (vl53_1->init()) {
        vl53_1->set_i2c_address(0x30);
        vl53_1->start_continuous();
        vl53_1_ok = true;
    } else {
        vl53_1_ok = false;
        // Capture init failure details
        uint32_t err = vl53_1->get_i2c_error();
        if (err != 0) vl53_1_i2c_error = err;
        uint32_t isr = vl53_1->get_i2c_isr();
        if (isr != 0 && isr != 1) vl53_1_i2c_isr = isr;
        vl53_1_last_hal_status = vl53_1->get_last_hal_status();
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(10);

    vl53_2->set_local_address(0x29);
    if (vl53_2->init()) {
        vl53_2->set_i2c_address(0x31);
        vl53_2->start_continuous();
        vl53_2_ok = true;
    } else {
        vl53_2_ok = false;
        // Capture init failure details
        uint32_t err = vl53_2->get_i2c_error();
        if (err != 0) vl53_2_i2c_error = err;
        uint32_t isr = vl53_2->get_i2c_isr();
        if (isr != 0 && isr != 1) vl53_2_i2c_isr = isr;
        vl53_2_last_hal_status = vl53_2->get_last_hal_status();
    }

    setup_step = 100;
}

void loop() {
    heartbeat = HAL_GetTick();
    tn3->get({});
    espdbt->update();
    joy = espdbt->get();

    if (HAL_GetTick() < input_ignore_until) {
        joy = mods::espdbt::State{};
    }

    if (imu != nullptr) {
        imu->rx();
        imu->update();
        yaw = imu->get_yaw();
        imu_updated = true;
    }

    static uint32_t last_vl53_read_tick = 0;
    if (HAL_GetTick() - last_vl53_read_tick > 10) {
        last_vl53_read_tick = HAL_GetTick();

        if (vl53_error_reset) {
            vl53_1_i2c_error = 0;
            vl53_2_i2c_error = 0;
            vl53_1_i2c_isr = 0;
            vl53_2_i2c_isr = 0;
            vl53_1_found_at_default = false;
            vl53_2_found_at_default = false;
            vl53_1_emergency_mm = 0.0f;
            vl53_2_emergency_mm = 0.0f;
            vl53_1_emergency_init_done = false;
            vl53_2_emergency_init_done = false;
            vl53_error_reset = false;
        }

        if (vl53_1_ok) {
            auto vl53_1_dist = vl53_1->read_distance_continuous();
            vl53_1_raw_m = vl53_1_dist.get_value();
            vl53_1_last_hal_status = vl53_1->get_last_hal_status();

            float raw_mm_1 = vl53_1_raw_m * 1000.0f;
            uint32_t isr1 = vl53_1->get_i2c_isr();
            if (isr1 != 0 && isr1 != 1) vl53_1_i2c_isr = isr1;

            uint32_t err1 = vl53_1->get_i2c_error();
            if (err1 != 0) vl53_1_i2c_error = err1;
            if (err1 == 4) {
                if (vl53_1->is_at_address(0x29)) vl53_1_found_at_default = true;
            }

            if (raw_mm_1 < -1.5f) {
                vl53_1_ok = false;
                vl53_1_distance_mm = 3000.0f;
            } else if (raw_mm_1 < 20.0f || raw_mm_1 > 3000.0f) {
                vl53_1_distance_mm = 3000.0f;

                vl53_1_last_update = HAL_GetTick();
            } else {
                vl53_1_distance_mm = raw_mm_1;
                vl53_1_last_update = HAL_GetTick();
            }
        }

        if (vl53_2_ok) {
            auto vl53_2_dist = vl53_2->read_distance_continuous();
            vl53_2_raw_m = vl53_2_dist.get_value();
            vl53_2_last_hal_status = vl53_2->get_last_hal_status();

            float raw_mm_2 = vl53_2_raw_m * 1000.0f;
            uint32_t isr2 = vl53_2->get_i2c_isr();
            if (isr2 != 0 && isr2 != 1) vl53_2_i2c_isr = isr2;

            uint32_t err2 = vl53_2->get_i2c_error();
            if (err2 != 0) vl53_2_i2c_error = err2;
            if (err2 == 4) {
                if (vl53_2->is_at_address(0x29)) vl53_2_found_at_default = true;
            }

            if (raw_mm_2 < -1.5f) {
                vl53_2_ok = false;
                vl53_2_distance_mm = 3000.0f;
            } else if (raw_mm_2 < 20.0f || raw_mm_2 > 3000.0f) {
                vl53_2_distance_mm = 3000.0f;

                vl53_2_last_update = HAL_GetTick();
            } else {
                vl53_2_distance_mm = raw_mm_2;
                vl53_2_last_update = HAL_GetTick();
            }
        }
    }

    static uint32_t last_vl53_retry = 0;

    if (!vl53_1_ok || !vl53_2_ok) {
        if (HAL_GetTick() - last_vl53_retry > 5000) {
            last_vl53_retry = HAL_GetTick();

            HAL_TIM_Base_Stop_IT(&htim7);
            tn3->get({0, 0, 0});
            tn3->send();
            try_vl53_init();

            joy = mods::espdbt::State{};
            input_ignore_until = HAL_GetTick() + 500;
            motor_output_inhibit_until = HAL_GetTick() + 500;

            target_transform = {0_mps, 0_mps, 0_radps};

            HAL_TIM_Base_Start_IT(&htim7);

            return;
        }
    }
    */

        if (vl53_1_ok && (HAL_GetTick() - vl53_1_last_update > 3000)) {
        vl53_1_ok = false;
    }
    if (vl53_2_ok && (HAL_GetTick() - vl53_2_last_update > 3000)) {
        vl53_2_ok = false;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (gpio_ctl_pa4 != 0 ? GPIO_PIN_SET : GPIO_PIN_RESET));

    target_transform = {
        joy.sticks[mods::espdbt::Stick::L].x * target_transform_max.velocity.x,
        joy.sticks[mods::espdbt::Stick::L].y * target_transform_max.velocity.y * -1.0,
        joy.sticks[mods::espdbt::Stick::R].x * target_transform_max.angvel * -1.0,
    };
    if (joy.buttons[mods::espdbt::Button::UP]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::LEFT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(-0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::RIGHT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::DOWN]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(-0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::OPTIONS] ||
               joy.buttons[mods::espdbt::Button::SHARE]) {
        constexpr float target_rad = 0.0f;
        constexpr float gain = 2.0f;
        constexpr float tolerance_rad = 0.05236f;

        float error =
            tr::utilities::angles::shortest_angular_distance(yaw, Qty<Radian>(target_rad))
                .get_value();

        if (std::abs(error) < tolerance_rad) {
            target_transform.angvel = 0.0_radps;
        } else {
            target_transform.angvel = Qty<RadianPerSecond>(error * gain * 1.0_radps);
        }
    } else if (joy.buttons[mods::espdbt::Button::TRIANGLE]) {
        if (!vl53_1_ok) {
            target_transform.velocity.x = 0.0_mps;
            target_transform.velocity.y = 0.0_mps;
        } else {
            float current_dist = vl53_1_distance_mm;
            float error = current_dist - vl53_target_dist_mm;

            if (std::abs(error) < vl53_tolerance_mm) {
                target_transform.velocity.x = 0.0_mps;
                target_transform.velocity.y = 0.0_mps;
            } else {
                float vel_mag = -1.0f * error * vl53_p_gain;

                if (vel_mag > vl53_max_speed_mps) vel_mag = vl53_max_speed_mps;
                if (vel_mag < -vl53_max_speed_mps)
                    vel_mag = -vl53_max_speed_mps;

                target_transform.velocity.x = (vel_mag * vl53_dir_x) * 1.0_mps;
                target_transform.velocity.y = (vel_mag * vl53_dir_y) * 1.0_mps;
            }
        }
    } else if (joy.buttons[mods::espdbt::Button::CROSS]) {
        if (!vl53_2_ok) {
            target_transform.velocity.x = 0.0_mps;
            target_transform.velocity.y = 0.0_mps;
        } else {
            float current_dist = vl53_2_distance_mm;
            float error = current_dist - vl53_target_dist_mm;

            if (std::abs(error) < vl53_tolerance_mm) {
                target_transform.velocity.x = 0.0_mps;
                target_transform.velocity.y = 0.0_mps;
            } else {
                float vel_mag = -1.0f * error * vl53_p_gain;

                if (vel_mag > vl53_max_speed_mps) vel_mag = vl53_max_speed_mps;
                if (vel_mag < -vl53_max_speed_mps)
                    vel_mag = -vl53_max_speed_mps;

                target_transform.velocity.x = (vel_mag * -vl53_dir_x) * 1.0_mps;
                target_transform.velocity.y = (vel_mag * -vl53_dir_y) * 1.0_mps;
            }
        }
    }

    ik->set_heading((yaw - Qty<Radian>(heading_offset_rad)) + 3.14159265_rad);
    ik->set_transform(target_transform);
    ik->update();

    static uint32_t reset_combo_start = 0;
    if (joy.buttons[mods::espdbt::Button::OPTIONS] &&
        joy.buttons[mods::espdbt::Button::SHARE]) {
        if (reset_combo_start == 0) {
            reset_combo_start = HAL_GetTick();
        } else if (HAL_GetTick() - reset_combo_start > 100) {
            HAL_NVIC_SystemReset();
        }
    } else {
        reset_combo_start = 0;
    }

    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        cvs->set_target_velocity(id, ik->get_velocity(id) / WHEEL_RADIUS);
        cvs->set_now_velocity(id, dji->get_now_head_angvel(OMNI3_TO_DJI[id]).unwrap());
        dji->set_target_current(OMNI3_TO_DJI[id], cvs->get_output(id));
    }

    tn3->set({
        {shared::TerunetId::PET, joy.buttons[mods::espdbt::Button::L1]},
        {shared::TerunetId::BELT, joy.buttons[mods::espdbt::Button::R1]},
        {shared::TerunetId::ARM, joy.buttons[mods::espdbt::Button::L2]},
    });
}

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (!initialized) return;

    if (htim->Instance == TIM7) {
        cvs->update();
        if (HAL_GetTick() < motor_output_inhibit_until) {
            for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
                dji->set_target_current(OMNI3_TO_DJI[id], 0_A);
            }
        } else {
            for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
                dji->set_target_current(OMNI3_TO_DJI[id], cvs->get_output(id));
            }
        }

        return;
    }

    if (htim->Instance == TIM6) {
        dji->tx();
        tn3->tx();
        return;
    }
}
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its) {
    if (!initialized) {
        return;
    }
    if (rx_fifo0_its == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo1_its) {
    if (!initialized) {
        return;
    }

    if (rx_fifo1_its == FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        if (hfdcan->Instance == FDCAN3) {
            dji->rx();
            return;
        }

        if (hfdcan->Instance == FDCAN1) {
            tn3->rx();
            return;
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    if (!initialized) {
        return;
    }

    if (htim->Instance == TIM1) {
        return;
    }
}

#endif
// espdbt割り込み
#ifdef HAL_UART_MODULE_ENABLED
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (!initialized) return;
    if (huart->Instance == USART1) {
        espdbt->rx();
    }
}
#endif
