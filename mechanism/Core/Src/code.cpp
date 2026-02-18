#include <main.h>

#include <code.hpp>
#include <cstdint>
#include <param.hpp>
#include <shared.hpp>
#include <tr/prelude.hpp>

bool initialized = false;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

Timer* tim_5;
Timer* tim_6;

mods::SoftwareTimer* sw_timer;
mods::WatchdogTimer* loop_wdt;
Qty<Hertz> loop_frequency;

mods::Terunet3<shared::TerunetId>* tn3;

// ペットボトル回収
EnumMap<PetServo, mods::Ds3235*> pet_servo;
bool pet_servo_state;

// アーム
mods::Ds3235* arm_servo;
bool arm_servo_state;

// 射出
mods::Dji *dji;
ctls::ControlPosition<Qty<Radian>, Qty<Ampere>> belt_cp(TIMER_PERIOD, BELT_CP_PARAM, BELT_CP_CONFIG, true);
Qty<Radian> belt_target_angle = 0_rad;
Qty<RadianPerSecond> belt_now_angvel;
Qty<Radian> belt_now_angle;
bool belt_shot;

void setup() {
    disable_irq();

    tim_5 = new Timer(&htim5);
    tim_6 = new Timer(&htim6, true);

    loop_wdt = new mods::WatchdogTimer(tim_5);
    sw_timer = new mods::SoftwareTimer(tim_6, SW_TIMER_PERIODS);

    tn3 = new mods::Terunet3<shared::TerunetId>(new Fdcan(&hfdcan1));

    for (const PetServo servo : AllVariants<PetServo>()) {
        pet_servo[servo] = new mods::Ds3235(new Pwm(&htim2, PET_SERVO_CHANNEL[servo]));
    }

    arm_servo = new mods::Ds3235(new Pwm(&htim3, TIM_CHANNEL_3));

    dji = new mods::Dji(new Fdcan(&hfdcan3), TIMER_PERIOD, DJI_DETAILS);

    initialized = true;

    enable_irq();
}

void loop() {
    loop_wdt->reset();
    loop_wdt->capture();

    tn3->get({});

    // ペット回収
    for (const PetServo servo : AllVariants<PetServo>()) {
        pet_servo[servo]->set_angle(
            pet_servo_state ? PET_SERVO_ANGLE[servo][ServoState::OPEN]
                            : PET_SERVO_ANGLE[servo][ServoState::CLOSE]
        );
    }

    // アーム
    arm_servo->set_angle(
        arm_servo_state ? ARM_SERVO_ANGLE[ServoState::OPEN] : ARM_SERVO_ANGLE[ServoState::CLOSE]
    );

    // 射出
    if (belt_shot) {
        belt_target_angle = -BELT_LENGTH / BELT_RADIUS;
    }
    belt_cp.set_param(BELT_CP_PARAM);
    belt_cp.set_target_position(belt_target_angle);

    // 1kHzタイマー
    if (sw_timer->poll(mods::software_timer::Channel::CHANNEL_1)) {
        belt_cp.update();
        dji->set_target_current(mods::dji::Id::ONE, belt_cp.get_output());
        dji->tx();
        tn3->tx();
    }

    tn3->set({});

    loop_frequency = 1 / loop_wdt->elapsed();
}

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (!initialized) {
        return;
    }

    if (htim->Instance == TIM6) {
        sw_timer->update();
        return;
    }
}
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo1_its) {
    if (!initialized) {
        return;
    }

    if (rx_fifo1_its == FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        if (hfdcan->Instance == FDCAN1) {
            tn3->rx();
            return;
        }
        if (hfdcan->Instance == FDCAN3) {
            dji->rx();
            belt_now_angle = dji->get_now_bottom_angle(mods::dji::Id::ONE).unwrap();
            belt_now_angvel = dji->get_now_bottom_angvel(mods::dji::Id::ONE).unwrap();
            belt_cp.set_now_position(belt_now_angle);
            belt_cp.set_now_velocity(belt_now_angvel);
            return;
        }
    }
}
#endif
