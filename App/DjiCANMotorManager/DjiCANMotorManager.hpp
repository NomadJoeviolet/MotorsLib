#ifndef CAN_MOTOR_MANAGER_HPP
#define CAN_MOTOR_MANAGER_HPP

#include "Motor/DjiMotors.hpp"
#include "etl/vector.h"
#include "can.h"

class CanMotorManager {
public:
    CanMotorManager() = default;
    CanMotorManager(
        CAN_HandleTypeDef *hcan, uint16_t baseTxID, uint16_t baseRxID, DjiMotor *motor1, DjiMotor *motor2,
        DjiMotor *motor3, DjiMotor *motor4
    )
    : hcan(hcan)
    , baseTxID(baseTxID)
    , baseRxID(baseRxID) {
        InitTxheader();
        if (motor1 != NULL) addMotor(motor1);
        if (motor2 != NULL) addMotor(motor2);
        if (motor3 != NULL) addMotor(motor3);
        if (motor4 != NULL) addMotor(motor4);
    };
    ~CanMotorManager() = default;

    void InitTxheader() {
        CAN_TxHeader.StdId = 0x200;      // 标准标识符(12bit)
        CAN_TxHeader.ExtId = 0x00;                 // 扩展标识符(29bit)
        CAN_TxHeader.IDE = CAN_ID_STD;   // 使用标准帧
        CAN_TxHeader.RTR = CAN_RTR_DATA; // 数据帧
        CAN_TxHeader.DLC = 0x08;         // 发送长度
        CAN_TxHeader.TransmitGlobalTime = DISABLE;//关闭can的时间触发通信模式
    }

    bool TransmitData() {
        for (auto motor : motors) {
            // 发送电机控制命令
            // 这里假设每个电机都有一个sendControlCommand方法
            // motor->sendControlCommand();
            SendData[(motor->motor_id - (baseRxID + 1)) * 2]     = (uint8_t)((uint16_t)(motor->getOutputValue()) >> 8);
            SendData[(motor->motor_id - (baseRxID + 1)) * 2 + 1] = (uint8_t)(motor->getOutputValue());
        }
        if (HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, SendData, &TxMailbox) == HAL_OK)
            return true;
        else
            return false;
    }

    bool addMotor(DjiMotor *motor) {
        if (motors.size() < motors.max_size() && motor != NULL) {
            motors.push_back(motor);
            return true;
        }
        return false;
    }
    bool removeMotor(DjiMotor *motor) {
        for (size_t i = 0; i < motors.size(); ++i) {
            if (motors[i] == motor) {
                motors.erase(motors.begin() + i);
                return true;
                break;
            }
        }
        return false;
    }
    bool clearMotors() {
        motors.clear();
        return true;
    }

    void dataDecode(const uint16_t motorID, const uint8_t *rxdata, uint16_t len) {
        for (size_t i = 0; i < motors.size(); ++i) {
            if (motors[i]->motor_id == motorID) {
                motors[i]->decode(rxdata, len);
                break;
            }
        }
    }

    CAN_HandleTypeDef *GetCanHandle() {
        return hcan;
    }

private:
    CAN_HandleTypeDef         *hcan{&hcan1}; // 使用的CAN句柄，默认CAN1
    etl::vector<DjiMotor *, 4> motors;       // 管理最多4个电机
    uint16_t                   baseTxID{};
    uint16_t                   baseRxID{};

    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint32_t            TxMailbox{};

    uint8_t SendData[8];
};

#endif // CAN_MOTOR_MANAGER_HPP