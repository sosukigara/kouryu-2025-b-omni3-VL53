// Core includes
#include <main.h>

#include <code.hpp>

// Project headers
#include <cmath>
#include <cstdint>
#include <param.hpp>
#include <shared.hpp>
#include <tr/prelude.hpp>

struct HeadState {
    Qty<Radian> angle = 0_rad;
    Qty<RadianPerSecond> angvel = 0_radps;
};

static EnumMap<mechs::omni3::Id, HeadState> read_head_states_local(mods::Dji<Fdcan> *dji) {
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
extern UART_HandleTypeDef huart1;

bool initialized = false;

// espdbt
mods::Espdbt *espdbt;
mods::espdbt::State joy;

mods::Terunet3 *tn3;

Qty<Radian> now_angle = 0_rad;
Qty<RadianPerSecond> now_angvel = 0_radps;
Qty<Radian> target_angle = 0_rad;

// imu

mods::Bno055 *imu;
Qty<Radian> yaw;

mods::Dji<Fdcan> *dji;

EnumMap<mechs::omni3::Id, HeadState> now_states_global = {};

// omni3
mechs::omni3::Ik *ik;
Transform2d target_transform = {0_mps, 0_mps, 0_radps};

controllers::ControlPosition<Qty<Radian>, Qty<Ampere>> *cp;

EnumMap<mechs::omni3::Id, ctls::ControlVelocity<Qty<Radian>, Qty<Ampere>> *> cvs;

// 自己位置推定
//  順運動学インスタンス
// mechs::omni3::Fk fk(OMNI3_CONFIG);

// 現在の自己位置 (x, y)
struct Position {
    Qty<Meter> x = 0_m;
    Qty<Meter> y = 0_m;
};
Position current_pos;

// 積分用の時刻管理
uint32_t last_odom_tick = 0;

void get_terunet() {}
void set_terunet() {}

void setup() {
    disable_irq();

    ik = new mechs::omni3::Ik(OMNI3_CONFIG);

    // imu
    imu = new mods::Bno055(new hardwares::I2cMaster(&hi2c3));

    tn3 = new mods::Terunet3(new Fdcan(&hfdcan1));

    // espdbt
    espdbt = new mods::Espdbt(new hardwares::Uart(&huart1));

    timer::start_interrupt(&htim6);
    timer::start_interrupt(&htim7);

    dji = new mods::Dji<Fdcan>(new Fdcan(&hfdcan3), DJI_SAMPLING_PERIOD, DJI_DETAILS);

    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        cvs[id] = new ctls::ControlVelocity<Qty<Radian>, Qty<Ampere>>(
            TIMER_PERIOD, CV_PARAM, CV_CONFIG, true
        );
    }

    initialized = true;

    enable_irq();
}

void loop() {
    get_terunet();

    // espdbt
    espdbt->update();
    joy = espdbt->get();

    // IMU
    imu->rx();
    imu->update();
    yaw = imu->get_yaw();

    /*
    // omni3
    ik.set_heading(yaw);  // imu
    ik.set_transform(target_transform);
    ik.update();

    now_states_global = read_head_states_local(dji);

    uint32_t now_tick = HAL_GetTick();  // ミリ秒単位
    if (last_odom_tick == 0) {
        last_odom_tick = now_tick;  // 初回はスキップ
    }

    double dt_sec = (double)(now_tick - last_odom_tick) / 1000.0;
    Qty<Second> dt = Qty<Second>(dt_sec);

    last_odom_tick = now_tick;

    fk.set_heading(yaw);  // ロボットの現在の向き

    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        Qty<MeterPerSecond> wheel_vel = now_states_global[id].angvel *
    WHEEL_RADIUS; fk.set_velocity(id, wheel_vel);
    }

    fk.update();
    auto result_transform = fk.get_transform();

    current_pos.x += result_transform.velocity.x * dt;
    current_pos.y += result_transform.velocity.y * dt;
    */

    target_transform = {
        joy.sticks[mods::espdbt::Stick::L].x * target_transform_max.velocity.x,
        joy.sticks[mods::espdbt::Stick::L].y * target_transform_max.velocity.y,
        joy.sticks[mods::espdbt::Stick::R].x * target_transform_max.angvel,
    };
    if (joy.buttons[mods::espdbt::Button::UP]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::LEFT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(-0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::RIGHT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::DOWN]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(-0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::OPTIONS]) {
        target_transform.angvel = angle::shortest_angular_distance(0_rad, yaw) *
            Qty<RadianPerSecond>(1.0_radps);

    } else if (joy.buttons[mods::espdbt::Button::SHARE]) {
        target_transform.angvel = angle::shortest_angular_distance(3.14_rad, yaw) *
            Qty<RadianPerSecond>(1.0_radps);
    }

    ik->set_heading(yaw);
    ik->set_transform(target_transform);
    ik->update();

    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        cvs[id]->set_target_velocity(ik->get_velocity(id) / WHEEL_RADIUS);
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
// espdbt割り込み
#ifdef HAL_UART_MODULE_ENABLED
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (!initialized) return;
    if (huart->Instance == USART1) {
        espdbt->rx();
    }
}
#endif
