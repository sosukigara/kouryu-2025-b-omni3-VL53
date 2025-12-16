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
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern I2C_HandleTypeDef hi2c3;

mods::Terunet3 *tn3;

mods::Bno055* imu;
Qty<Radian> yaw;



void setup() {
    disable_irq();

    imu = new mods::Bno055(new hardwares::I2cMaster(&hi2c3));

    tn3 = new mods::Terunet3(new Fdcan(&hfdcan1));

    timer::start_interrupt(&htim6);
    timer::start_interrupt(&htim7);

    initialized = true;

    enable_irq();
}

void loop() {
    get_terunet();

    imu ->rx();
    imu ->update();
    yaw = imu -> get_yaw();

    set_terunet();
}

void get_terunet() {}
void set_terunet() {}

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (!initialized) {
        return;
    }

    // 演算用
    if (htim->Instance == TIM7) {
        return;
    }

    // 通信用
    if (htim->Instance == TIM6) {
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
