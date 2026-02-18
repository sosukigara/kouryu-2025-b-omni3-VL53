
#include <main.h>

#include <cmath>
#include <code.hpp>
#include <cstdint>
#include <param.hpp>
#include <shared.hpp>
#include <tr/controllers/sync_control_velocity/sync_control_velocity.hpp>
#include <tr/prelude.hpp>

#include "tr/utilities/angles/angles.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

extern UART_HandleTypeDef huart1;

bool initialized = false;

volatile uint8_t gpio_ctl_pa2 = 0;
volatile uint8_t gpio_ctl_pa3 = 0;
volatile uint8_t gpio_ctl_pa4 = 0;

// espdbt
mods::Espdbt *espdbt;
mods::espdbt::State joy;

mods::Terunet3<mechs::omni3::Id> *tn3;

mods::Bno055 *imu;
Qty<Radian> yaw;

mods::Dji *dji;

volatile uint32_t heartbeat = 0;
volatile int setup_step = 0;
volatile uint8_t found_addr = 0;

mods::Vl53l0x *vl53_1;
mods::Vl53l0x *vl53_2;
volatile bool vl53_1_ok = false;
volatile bool vl53_2_ok = false;
volatile float vl53_1_distance_mm = -1.0f;
volatile float vl53_2_distance_mm = -1.0f;
volatile uint32_t vl53_1_last_update = 0;
volatile uint32_t vl53_2_last_update = 0;

volatile bool imu_updated = false;

volatile float live_kp = 0.0f;
volatile float live_ki = 0.0f;
volatile float live_kd = 0.0f;
volatile float last_live_kp = -1.0f;
volatile float last_live_ki = -1.0f;
volatile float last_live_kd = -1.0f;

volatile float vl53_target_dist_mm = 300.0f;
volatile float vl53_tolerance_mm = 30.0f;
volatile float vl53_p_gain = 0.003f;
volatile float vl53_max_speed_mps = 0.6f;
volatile float vl53_dir_x = -1.0f;
volatile float vl53_dir_y = 0.0f;
volatile float heading_offset_rad = 0.5236f;

// omni3
mechs::omni3::Ik *ik;
Transform2d target_transform = {0_mps, 0_mps, 0_radps};

tr::controllers::SyncControlVelocity<mechs::omni3::Id, Qty<Radian>, Qty<Ampere>> *cvs;

tr::hardwares::Timer *timer6;
tr::hardwares::Timer *timer7;

struct Position {
    Qty<Meter> x = 0_m;
    Qty<Meter> y = 0_m;
};
Position current_pos;

uint32_t last_odom_tick = 0;

volatile float live_pos_x_m = 0.0f;
volatile float live_pos_y_m = 0.0f;
volatile float live_pos_yaw_deg = 0.0f;

mechs::omni3::Fk *fk;

void get_terunet() {}
void set_terunet() {}
void try_vl53_init();

void setup() {
    disable_irq();

    ik = new mechs::omni3::Ik(OMNI3_CONFIG);
    fk = new mechs::omni3::Fk(OMNI3_CONFIG);

    vl53_1 = new mods::Vl53l0x(&hi2c2);
    vl53_2 = new mods::Vl53l0x(&hi2c2);

    tn3 = new mods::Terunet3<mechs::omni3::Id>(new Fdcan(&hfdcan1));

    espdbt = new mods::Espdbt(new hardwares::Uart(&huart1));

    timer6 = new tr::hardwares::Timer(&htim6, true);
    timer7 = new tr::hardwares::Timer(&htim7, true);

    dji = new mods::Dji(new Fdcan(&hfdcan3), DJI_SAMPLING_PERIOD, DJI_DETAILS);

    cvs = new tr::controllers::SyncControlVelocity<mechs::omni3::Id, Qty<Radian>, Qty<Ampere>>(
        TIMER_PERIOD, CV_PARAM, CV_CONFIG, true
    );

    live_kp = CV_PARAM.pid_param.kp;
    live_ki = CV_PARAM.pid_param.ki;
    live_kd = CV_PARAM.pid_param.kd;
    last_live_kp = live_kp;
    last_live_ki = live_ki;
    last_live_kd = live_kd;

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
    setup_step = 2;

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
    }

    setup_step = 100;
}

void loop() {
    heartbeat = HAL_GetTick();

    if (live_kp != last_live_kp || live_ki != last_live_ki || live_kd != last_live_kd) {
        last_live_kp = live_kp;
        last_live_ki = live_ki;
        last_live_kd = live_kd;

        tr::controllers::control_velocity::Param new_param = {
            .pid_param = {.kp = live_kp, .ki = live_ki, .kd = live_kd}
        };

        for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
            cvs->set_param(id, new_param);
        }
    }

    get_terunet();

    espdbt->update();
    joy = espdbt->get();

    static uint32_t last_vl53_read_tick = 0;
    if (HAL_GetTick() - last_vl53_read_tick > 10) {
        last_vl53_read_tick = HAL_GetTick();

        {
            auto vl53_1_dist = vl53_1->read_distance_continuous();
            float raw_mm_1 = vl53_1_dist.get_value() * 1000.0f;
            if (raw_mm_1 < 20.0f || raw_mm_1 > 3000.0f) {
                vl53_1_distance_mm = 3000.0f;
            } else {
                vl53_1_distance_mm = raw_mm_1;
            }
            vl53_1_last_update = HAL_GetTick();
        }

        {
            auto vl53_2_dist = vl53_2->read_distance_continuous();
            float raw_mm_2 = vl53_2_dist.get_value() * 1000.0f;
            if (raw_mm_2 < 20.0f || raw_mm_2 > 3000.0f) {
                vl53_2_distance_mm = 3000.0f;
            } else {
                vl53_2_distance_mm = raw_mm_2;
            }
            vl53_2_last_update = HAL_GetTick();
        }
    }

    static uint32_t last_vl53_retry = 0;

    if (!vl53_1_ok || !vl53_2_ok) {
        if (HAL_GetTick() - last_vl53_retry > 500) {
            last_vl53_retry = HAL_GetTick();
            try_vl53_init();
        }
    }

    if (vl53_1_ok && (HAL_GetTick() - vl53_1_last_update > 3000)) {
        vl53_1_ok = false;
    }
    if (vl53_2_ok && (HAL_GetTick() - vl53_2_last_update > 3000)) {
        vl53_2_ok = false;
    }

    static uint32_t last_imu_tick = 0;
    if (imu != nullptr && (HAL_GetTick() - last_imu_tick > 10)) {
        last_imu_tick = HAL_GetTick();
        imu->rx();
        imu->update();
        yaw = imu->get_yaw();

        imu_updated = true;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (gpio_ctl_pa4 != 0 ? GPIO_PIN_SET : GPIO_PIN_RESET));

    target_transform = {
        joy.sticks[mods::espdbt::Stick::L].x * target_transform_max.velocity.x * -1.0,
        joy.sticks[mods::espdbt::Stick::L].y * target_transform_max.velocity.y,
        joy.sticks[mods::espdbt::Stick::R].x * target_transform_max.angvel,
    };
    if (joy.buttons[mods::espdbt::Button::UP]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(-0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::LEFT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::RIGHT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(-0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::DOWN]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::OPTIONS]) {
        target_transform.angvel =
            tr::utilities::angles::shortest_angular_distance(0_rad, yaw) *
            Qty<RadianPerSecond>(4.6_radps);
    } else if (joy.buttons[mods::espdbt::Button::SHARE]) {
        target_transform.angvel =
            tr::utilities::angles::shortest_angular_distance(0_rad, yaw) *
            Qty<RadianPerSecond>(4.6_radps);
    } else if (joy.buttons[mods::espdbt::Button::TRIANGLE]) {
        float current_dist = vl53_1_distance_mm;
        float error = current_dist - vl53_target_dist_mm;

        if (std::abs(error) < vl53_tolerance_mm) {
            target_transform.velocity.x = 0.0_mps;
            target_transform.velocity.y = 0.0_mps;
        } else {
            float vel_mag = -1.0f * error * vl53_p_gain;

            // Clamp Speed Magnitude
            if (vel_mag > vl53_max_speed_mps) vel_mag = vl53_max_speed_mps;
            if (vel_mag < -vl53_max_speed_mps) vel_mag = -vl53_max_speed_mps;

            target_transform.velocity.x = (vel_mag * vl53_dir_x) * 1.0_mps;
            target_transform.velocity.y = (vel_mag * vl53_dir_y) * 1.0_mps;
        }
    } else if (joy.buttons[mods::espdbt::Button::L3]) {
        float error_x = 0.0f - live_pos_x_m;
        float error_y = 0.0f - live_pos_y_m;
        float dist_sq = error_x * error_x + error_y * error_y;

        if (dist_sq > 0.0025f) {
            float kp = 1.0f;
            float max_speed = 1.0f;

            float vx = error_x * kp;
            float vy = error_y * kp;

            // Clamp Speed
            float speed = std::sqrt(vx * vx + vy * vy);
            if (speed > max_speed) {
                float scale = max_speed / speed;
                vx *= scale;
                vy *= scale;
            }

            target_transform.velocity.x = Qty<MeterPerSecond>(vx * 1.0f);
            target_transform.velocity.y = Qty<MeterPerSecond>(vy * 1.0f);
        } else {
            target_transform.velocity.x = 0_mps;
            target_transform.velocity.y = 0_mps;
        }
    } else if (joy.buttons[mods::espdbt::Button::CROSS]) {
        float current_dist = vl53_2_distance_mm;
        float error = current_dist - vl53_target_dist_mm;

        if (std::abs(error) < vl53_tolerance_mm) {
            target_transform.velocity.x = 0.0_mps;
            target_transform.velocity.y = 0.0_mps;
        } else {
            float vel_mag = -1.0f * error * vl53_p_gain;

            if (vel_mag > vl53_max_speed_mps) vel_mag = vl53_max_speed_mps;
            if (vel_mag < -vl53_max_speed_mps) vel_mag = -vl53_max_speed_mps;

            target_transform.velocity.x = (vel_mag * -vl53_dir_x) * 1.0_mps;
            target_transform.velocity.y = (vel_mag * -vl53_dir_y) * 1.0_mps;
        }
    }

    ik->set_heading((yaw - Qty<Radian>(heading_offset_rad)) + 3.14159265_rad);
    ik->set_transform(target_transform);
    ik->update();

    uint32_t now_tick = HAL_GetTick();
    if (last_odom_tick != 0) {
        Qty<Second> dt = Qty<Second>((float)(now_tick - last_odom_tick) / 1000.0f);

        fk->set_heading(-(yaw - Qty<Radian>(heading_offset_rad)) + 3.14159265_rad);
        for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
            fk->set_velocity(id, dji->get_now_head_angvel(OMNI3_TO_DJI[id]).unwrap() * WHEEL_RADIUS);
        }
        fk->update();

        auto result_transform = fk->get_transform();
        live_pos_x_m += result_transform.velocity.x.get_value() * dt.get_value();
        live_pos_y_m += result_transform.velocity.y.get_value() * dt.get_value();
        live_pos_yaw_deg = (yaw).get_value() * 180.0f / 3.14159265f;
    }
    last_odom_tick = now_tick;

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

    set_terunet();
}

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (!initialized) {
        return;
    }

    // 演算用
    if (htim->Instance == TIM7) {
        cvs->update();
        for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
            dji->set_target_current(OMNI3_TO_DJI[id], cvs->get_output(id));
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
// espdbt割り込み
#ifdef HAL_UART_MODULE_ENABLED
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (!initialized) return;
    if (huart->Instance == USART1) {
        espdbt->rx();
    }
}
#endif
