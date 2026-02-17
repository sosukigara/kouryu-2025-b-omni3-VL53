// Update timestamp: 2026-01-14 23:25 (JST)
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

/*
static EnumMap<mechs::omni3::Id, HeadState>
read_head_states_local(mods::Dji<Fdcan> *dji) { EnumMap<mechs::omni3::Id,
HeadState> res; for (const mechs::omni3::Id id :
AllVariants<mechs::omni3::Id>()) { const mods::dji::Id djid = OMNI3_TO_DJI[id];
        res[id].angle = dji->get_now_head_angle(djid).unwrap_or(0_rad);
        res[id].angvel = dji->get_now_head_angvel(djid).unwrap_or(0_radps);
    }
    return res;
}
*/

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
// extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;

bool initialized = false;

// GPIO Debug Control
volatile uint8_t gpio_ctl_pa2 = 0;
volatile uint8_t gpio_ctl_pa3 = 0;
volatile uint8_t gpio_ctl_pa4 = 0;  // Found in main.c instead of PA2
// volatile uint8_t gpio_ctl_pc4 = 0; // Configured as USART1
// volatile uint8_t gpio_ctl_pc5 = 0; // Configured as USART1
volatile uint8_t monitor_pa2 = 0;
volatile uint8_t monitor_pa3 = 0;

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
Qty<Radian> yaw_offset = 0_rad;

mods::Dji<Fdcan> *dji;

// --- 監視すべき変数はこれだけです ---
volatile uint32_t heartbeat = 0;  // 猛スピードで増えればプログラム正常
volatile int setup_step = 0;      // 100 まで行けば完了
volatile uint8_t found_addr = 0;  // 0x29 など
/*
volatile uint16_t vl53_model_id_16 = 0;  // 0xEACC ならL1X
volatile int vl53_init_step = -99;       // 200 ならL1X初期化成功
volatile float vl53_distance_mm = -1.0F;
volatile int setup_hal_status = -99;
volatile bool vl53_init_success = false;
volatile uint8_t vl53_range_status = 255;
volatile uint8_t vl53_stream_count = 0;
*/

// Dual VL53 Globals
mods::Vl53l0x *vl53_1;  // PA3 XSHUT
mods::Vl53l0x *vl53_2;  // PA2 XSHUT
volatile bool vl53_1_ok = false;
volatile bool vl53_2_ok = false;
volatile float vl53_1_distance_mm = -1.0f;
volatile float vl53_2_distance_mm = -1.0f;
volatile uint32_t vl53_1_last_update = 0;  // Timeout detection
volatile uint32_t vl53_2_last_update = 0;  // Timeout detection

// IMU Debug
volatile float debug_yaw_deg = 0.0f;
volatile float debug_yaw_relative_deg = 0.0f;  // Relative yaw (after offset)
volatile uint8_t debug_bno_status = 255;
volatile uint8_t debug_bno_error = 255;
volatile bool imu_updated = false;

// Tuning Params (Live Expressions)
volatile float live_kp = 0.0f;
volatile float live_ki = 0.0f;
volatile float live_kd = 0.0f;
volatile float last_live_kp = -1.0f;  // 変化検知用
volatile float last_live_ki = -1.0f;
volatile float last_live_kd = -1.0f;

volatile float vl53_target_dist_mm = 300.0f;
volatile float vl53_tolerance_mm = 30.0f;
volatile float vl53_p_gain = 0.003f;  // Gain: (e.g. 0.003 -> 100mm diff = 0.3m/s)
volatile float vl53_max_speed_mps = 0.6f;  // Speed Limit
volatile float vl53_dir_x = -1.0f;  // Direction vector X (Updated to -1.0)
volatile float vl53_dir_y = 0.0f;   // Direction vector Y
volatile float heading_offset_rad = 0.5236f;  // Mounting offset (30 degrees)

// mods::Vl53l0x *vl53;

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
void try_vl53_init();

void setup() {
    disable_irq();

    ik = new mechs::omni3::Ik(OMNI3_CONFIG);

    // vl53 instances (Address defaults to 0x29, will be changed in try_vl53_init)
    // vl53 instances (Address defaults to 0x29, will be changed in try_vl53_init)
    vl53_1 = new mods::Vl53l0x(&hi2c2);
    vl53_2 = new mods::Vl53l0x(&hi2c2);

    tn3 = new mods::Terunet3(new Fdcan(&hfdcan1));

    // espdbt (UART1 conflict with I2C2 PA9)
    espdbt = new mods::Espdbt(new hardwares::Uart(&huart1));

    timer::start_interrupt(&htim6);
    timer::start_interrupt(&htim7);

    dji = new mods::Dji<Fdcan>(new Fdcan(&hfdcan3), DJI_SAMPLING_PERIOD, DJI_DETAILS);

    for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
        cvs[id] = new ctls::ControlVelocity<Qty<Radian>, Qty<Ampere>>(
            TIMER_PERIOD, CV_PARAM, CV_CONFIG, true
        );
    }

    // Initialize Live PID Params
    live_kp = CV_PARAM.pid_param.kp;
    live_ki = CV_PARAM.pid_param.ki;
    live_kd = CV_PARAM.pid_param.kd;
    last_live_kp = live_kp;
    last_live_ki = live_ki;
    last_live_kd = live_kd;

    initialized = true;

    enable_irq();

    // imu (I2C3)
    HAL_Delay(100);  // Wait for BNO055 to boot (Reduced from 500ms)
    imu = new mods::Bno055(new hardwares::I2cMaster(&hi2c3));

    setup_step = 1;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // Try init immediately at startup
    try_vl53_init();
}

// Dual Init Sequence
void try_vl53_init() {
    setup_step = 2;

    // 1. Reset Both (XSHUT=Low)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(10);  // Power down cycle (Reduced from 50ms)

    // 2. Enable Sensor 1 (PA3) -> Address 0x30
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(10);  // Wait for boot (Reduced from 50ms, need 1.2ms min)

    // Init Sensor 1
    vl53_1->set_local_address(0x29);  // Reset internal knowledge to default
    if (vl53_1->init()) {
        vl53_1->set_i2c_address(0x30);
        // vl53_1->set_high_speed_mode(); // Disabled for stability
        vl53_1->start_continuous();  // Start Continuous
        vl53_1_ok = true;
    } else {
        vl53_1_ok = false;
    }

    // 3. Enable Sensor 2 (PA2) -> Address 0x31
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(10);  // Wait for boot (Reduced from 50ms)

    // Init Sensor 2
    vl53_2->set_local_address(0x29);  // Reset internal
    if (vl53_2->init()) {
        vl53_2->set_i2c_address(0x31);
        // vl53_2->set_high_speed_mode(); // Disabled for stability (33ms default)
        vl53_2->start_continuous();  // Start Continuous
        vl53_2_ok = true;
    } else {
        vl53_2_ok = false;
    }

    setup_step = 100;
}

void loop() {
    heartbeat = HAL_GetTick();

    // Live PID Update Check
    if (live_kp != last_live_kp || live_ki != last_live_ki || live_kd != last_live_kd) {
        last_live_kp = live_kp;
        last_live_ki = live_ki;
        last_live_kd = live_kd;

        tr::controllers::control_velocity::Param new_param = {
            .pid_param = {.kp = live_kp, .ki = live_ki, .kd = live_kd}
        };

        for (const mechs::omni3::Id id : AllVariants<mechs::omni3::Id>()) {
            cvs[id]->set_param(new_param);
        }
    }

    get_terunet();

    // espdbt
    espdbt->update();
    joy = espdbt->get();

    // Sensor 1 Read (0x30)
    if (vl53_1_ok && vl53_1->is_ready()) {
        auto dist = vl53_1->read_distance_continuous();  // Non-blocking read
        // VL53 returns 0 on out-of-range/error.
        // Map 0 -> 3000mm (Far) to prevent "Far=Close" bug.
        float raw_mm = dist.get_value() * 1000.0f;
        if (raw_mm < 20.0f || raw_mm > 3000.0f) {
            vl53_1_distance_mm = 3000.0f;
        } else {
            vl53_1_distance_mm = raw_mm;
        }
        vl53_1_last_update = HAL_GetTick();  // Update timestamp
    }

    // Sensor 2 Read (0x31)
    if (vl53_2_ok && vl53_2->is_ready()) {
        auto dist = vl53_2->read_distance_continuous();  // Non-blocking read
        float raw_mm = dist.get_value() * 1000.0f;
        if (raw_mm < 20.0f || raw_mm > 3000.0f) {
            vl53_2_distance_mm = 3000.0f;
        } else {
            vl53_2_distance_mm = raw_mm;
        }
        vl53_2_last_update = HAL_GetTick();  // Update timestamp
    }

    // VL53 Connection Management (Retry Logic)
    // Retry only if INIT failed initially. Watchdog removed per user request.
    static uint32_t last_vl53_retry = 0;

    if (!vl53_1_ok || !vl53_2_ok) {
        if (HAL_GetTick() - last_vl53_retry > 500) {
            last_vl53_retry = HAL_GetTick();
            try_vl53_init();
        }
    }

    // Timeout Detection: Reset sensor if no update for 3 seconds
    if (vl53_1_ok && (HAL_GetTick() - vl53_1_last_update > 3000)) {
        vl53_1_ok = false;  // Mark as failed, will be re-initialized in next retry cycle
    }
    if (vl53_2_ok && (HAL_GetTick() - vl53_2_last_update > 3000)) {
        vl53_2_ok = false;  // Mark as failed, will be re-initialized in next retry cycle
    }

    // IMU (10ms interval = ~100Hz)
    static uint32_t last_imu_tick = 0;
    if (imu != nullptr && (HAL_GetTick() - last_imu_tick > 10)) {
        last_imu_tick = HAL_GetTick();
        imu->rx();
        imu->update();
        yaw = imu->get_yaw();

        // Debug info
        imu_updated = true;
        debug_yaw_deg = yaw.get_value() * 180.0f / 3.14159265f;
        debug_yaw_relative_deg =
            (yaw.get_value() - yaw_offset.get_value()) * 180.0f / 3.14159265f;
        debug_bno_status = imu->get_system_status();
        debug_bno_error = imu->get_system_error();
    }

    // GPIO Control
    // Monitor (実際にピンがHighになっているか確認)
    extern volatile uint8_t monitor_pa2;
    extern volatile uint8_t monitor_pa3;
    monitor_pa2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    monitor_pa3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (gpio_ctl_pa4 != 0 ? GPIO_PIN_SET : GPIO_PIN_RESET));

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
        joy.sticks[mods::espdbt::Stick::L].x * target_transform_max.velocity.x * -1.0,  // X-axis Inverted
        joy.sticks[mods::espdbt::Stick::L].y * target_transform_max.velocity.y,
        joy.sticks[mods::espdbt::Stick::R].x * target_transform_max.angvel,
    };
    if (joy.buttons[mods::espdbt::Button::UP]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(-0.20_mps);  // Inverted (+ -> -)
    } else if (joy.buttons[mods::espdbt::Button::LEFT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::RIGHT]) {
        target_transform.velocity.x = Qty<MeterPerSecond>(-0.20_mps);
    } else if (joy.buttons[mods::espdbt::Button::DOWN]) {
        target_transform.velocity.y = Qty<MeterPerSecond>(0.20_mps);  // Inverted (- -> +)
    } else if (joy.buttons[mods::espdbt::Button::OPTIONS]) {
        // Auto-rotate to IMU absolute 0 degrees
        target_transform.angvel = angle::shortest_angular_distance(0_rad, yaw) *
            Qty<RadianPerSecond>(4.6_radps);
    } else if (joy.buttons[mods::espdbt::Button::SHARE]) {
        // Face Front (Target the Set Origin)
        target_transform.angvel = angle::shortest_angular_distance(yaw_offset, yaw) *
            Qty<RadianPerSecond>(4.6_radps);
    } else if (joy.buttons[mods::espdbt::Button::TRIANGLE]) {
        // Sensor-based P-Control (Using Sensor 1)
        float current_dist = vl53_1_distance_mm;
        float error = current_dist - vl53_target_dist_mm;

        // Deadband (Tolerance)
        if (std::abs(error) < vl53_tolerance_mm) {
            target_transform.velocity.x = 0.0_mps;
            target_transform.velocity.y = 0.0_mps;
        } else {
            // P-Control: Velocity = Gain * Error
            // Flip sign back to positive per user report "Reversed"
            float vel_mag = 1.0f * error * vl53_p_gain;

            // Clamp Speed Magnitude
            if (vel_mag > vl53_max_speed_mps) vel_mag = vl53_max_speed_mps;
            if (vel_mag < -vl53_max_speed_mps) vel_mag = -vl53_max_speed_mps;

            // Apply direction vector
            target_transform.velocity.x = (vel_mag * vl53_dir_x) * 1.0_mps;
            target_transform.velocity.y = (vel_mag * vl53_dir_y) * 1.0_mps;
        }
    } else if (joy.buttons[mods::espdbt::Button::CROSS]) {
        // Sensor-based P-Control (Using Sensor 2 - Opposite Action)
        float current_dist = vl53_2_distance_mm;
        float error = current_dist - vl53_target_dist_mm;

        // Deadband
        if (std::abs(error) < vl53_tolerance_mm) {
            target_transform.velocity.x = 0.0_mps;
            target_transform.velocity.y = 0.0_mps;
        } else {
            // Same P-Control
            float vel_mag = 1.0f * error * vl53_p_gain;

            if (vel_mag > vl53_max_speed_mps) vel_mag = vl53_max_speed_mps;
            if (vel_mag < -vl53_max_speed_mps) vel_mag = -vl53_max_speed_mps;

            // Apply INVERTED direction vector for Opposite Action
            target_transform.velocity.x = (vel_mag * -vl53_dir_x) * 1.0_mps;
            target_transform.velocity.y = (vel_mag * -vl53_dir_y) * 1.0_mps;
        }
    }

    // Fix Coordinate System with Mounting Offset (adjustable via heading_offset_rad)
    // Default: 45 deg = PI/4 = 0.7854 rad
    // Adjust heading_offset_rad in Live Expressions to fix drift
    ik->set_heading(-(yaw - yaw_offset - Qty<Radian>(heading_offset_rad)) + 3.14159265_rad);
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
