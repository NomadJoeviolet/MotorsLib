#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "can.h"

#include "DjiCANMotorManager/DjiCANMotorManager.hpp"

PositionPID angle_pid(10.0f, 0.0f, 0.0f, 8000.0f, 8000.f);
PositionPID speed_pid(10.0f, 0.0f, 0.0f, 8000.0f, 8000.f);

DjiMotor test2006motor(
    MotorType::MOTOR_2006, (uint16_t)0x201, true, MotorControlMode::SPEED_MODE, 1.0f, &angle_pid, &speed_pid
);

CanMotorManager CAN1MotorManager(&hcan1, uint16_t(0x200), uint16_t(0x200), &test2006motor, NULL, NULL, NULL);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  电机控制函数
 * @retval None
 */
void MotorControlThread() {
    for (;;) {
        // 电机控制逻辑
        test2006motor.motor_data.ref_shaft_speed = 1000.0f;
        test2006motor.CalculateAndSetOutput();
        osDelay(1); // 每10ms执行一次
    }
}

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

void CANManagerSendThread() {
    for (;;) {
        // CAN发送逻辑
        CAN1MotorManager.TransmitData();
        osDelay(1); // 每5ms执行一次
    }
}
#ifdef __cplusplus
}
#endif


//feedback
#ifdef __cplusplus
extern "C" {
#endif

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { // 收到CAN数据会触发接收中断，进入该回调函数
    uint8_t             RxData[8];
    CAN_RxHeaderTypeDef CAN_RxHeader;
    if (hcan == &hcan1) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, RxData) != HAL_OK) {
            while (1) {
            };
        } else {
            CAN1MotorManager.dataDecode(CAN_RxHeader.StdId, RxData, 8);
        }
    }
}

#ifdef __cplusplus
}
#endif
