#ifndef DJI_MOTORS_HPP
#define DJI_MOTORS_HPP

#include "Algorithm/pid.hpp"
#include "math.h"
//#include "etl/observer.h"

enum MotorType {
    MOTOR_2006,
    MOTOR_3508,
    MOTOR_GM6020,
    MOTOR_NULL,
};

enum MotorControlMode {
    SPEED_MODE,
    ANGLE_MODE,
    NONE_MODE,
};

/**
 * @brief 大疆电机类
 * @details 提供大疆电机的基本接口和成员变量
 * @note 实现数据解码功能
 * @note 提供电机状态查询和修改接口，ref数据修改接口，CalculateAndSetOutput接口
 * @warning 该类仅支持大疆电机
 * @author nomad
 * @date 2025-10-08
 * @version 1.0
 * @param MotorType 电机类型枚举
 * @param MotorControlMode 电机控制模式枚举
 * @param DjiMotorData 电机数据结构体
 */
class DjiMotor {
    //friend class CANMotorManager ;
public:
    /**
     * @brief 电机数据结构体
     * @note 包含电机的转速、角度、电流等信息
     * @author nomad
     * @param shaft_speed 输出轴转度，单位rpm
     * @param rotor_speed 转子转速，单位rpm
     * @param shaft_angle 输出轴非连续角度，单位度
     * @param rotor_angle 转子非连续角度，单位度
     * @param last_shaft_angle 上次输出轴角度，单位度
     * @param last_rotor_angle 上次转子角度，单位度
     * @param reduction_ratio 减速比
     * @param current 电机电流，单位A，注意2006电机是转矩，但电流和转矩成正比，所以使用电流表示转矩
     * @param temperature 电机温度，单位摄氏度
     * @param continuous_shaft_round 输出轴连续转动圈数
     * @param continuous_rotor_round 转子转动圈数
     */
    typedef struct DjiMotorData {
        float shaft_speed{};            // 转度，单位rpm
        float rotor_speed{};            // 转速，单位rpm
        float shaft_angle{};            // 角度，单位度
        float rotor_angle{};            // 角度，单位度
        float last_shaft_angle{};       // 上次角度，单位度
        float last_rotor_angle{};       // 上次角度，单位度
        float reduction_ratio{1};       // 减速比
        float current{};                // 电流，单位A
        float temperature{};            // 温度，单位摄氏度
        float continuous_shaft_round{}; // 连续转动圈数
        float continuous_rotor_round{}; // 连续转动圈数
        float continuous_shaft_angle{}; // 连续角度，单位度
        float continuous_rotor_angle{}; // 连续角度，单位度

        float ref_shaft_speed{}; // 目标转速，单位rpm
        float ref_shaft_angle{}; // 目标角度，单位度
    } DjiMotorData;

public:
    DjiMotorData motor_data;
    MotorType    motor_type{MOTOR_NULL};
    uint16_t     motor_id{};
private:
        /**
     * @brief error_status 错误状态码
     * @note 0x100: 减速比设置错误，导致除零错误
     * @note 0x10: 电机类型未设置
     * @note 0x01: 数据错误
     * @note 0x00: 没有错误
     */
    uint8_t error_status{0};
    bool             motor_enabled{false};
    MotorControlMode control_mode{NONE_MODE};
    basePID         *pid_speed;
    basePID         *pid_angle;

    float outputValue{0};
    float MaxOutput{0};

    float lastUpdateTime{0};


public :
    DjiMotor() {
        pid_speed    = NULL;
        pid_angle    = NULL;
        motor_type   = MotorType::MOTOR_NULL;
        control_mode = MotorControlMode::NONE_MODE;
        error_status = 0;
        // 初始化电机数据
        motor_data = DjiMotorData();
    }

    DjiMotor(
        MotorType type, uint16_t motor_id, bool motor_enabled, MotorControlMode mode, float reduction_ratio,
        basePID *pid_angle , basePID *pid_speed
    )
    : motor_type(type)
    , motor_id(motor_id)
    , motor_enabled(motor_enabled)
    , control_mode(mode)

    , pid_speed(pid_speed)
    , pid_angle(pid_angle) {
        error_status = 0;
        // 初始化电机数据
        motor_data                 = DjiMotorData();
        motor_data.reduction_ratio = reduction_ratio;
        switch (motor_type) {
        case MOTOR_2006:
            /* code */
            MaxOutput = 10000; // 2006电机最大电流10A，对应映射值10000
            break;
        case MOTOR_3508:
            /* code */
            MaxOutput = 16384; // 3508电机电调给出的最大电流20A，对应映射值16384
            break;
        case MOTOR_GM6020:
            /* code */
            MaxOutput = 16384; // GM6020电机最大电流3A，对应映射值16384
            break;
        default:
            break;
        }
    }
    ~DjiMotor() = default;

    // bool setAngleReference(float reference_angle) {
    //     if (control_mode != ANGLE_MODE) return false;
    //     if (pid_angle == NULL) return false;
    //     motor_data.ref_shaft_angle = reference_angle;
    //     return true;
    // }
    // bool setSpeedReference(float reference_speed) {
    //     if (control_mode != SPEED_MODE) return false;
    //     if (pid_speed == NULL) return false;
    //     motor_data.ref_shaft_speed = reference_speed;
    //     return true;
    // }

    bool setControlMode(MotorControlMode mode) {
        if (control_mode == mode) return true;
        if (mode == SPEED_MODE && control_mode == ANGLE_MODE) pid_angle->clearIntegralandOutput();
        if (mode == ANGLE_MODE && control_mode == SPEED_MODE) pid_speed->clearIntegralandOutput();
        outputValue  = 0; // 切换模式时清零输出值
        control_mode = mode;
        return true;
    }

    /**
     * @brief 电机使能，作用在calculate输出上
     * @note 使能后，calculate才会计算输出值
     */
    bool enableMotor() {
        motor_enabled = true;
        return true;
    }

    /**
     * @brief 电机失能，作用在calculate输出上
     * @note 失能后，calculate不会计算输出值，输出值为0
     */
    bool disableMotor() {
        motor_enabled = false;
        return true;
    }

    /**
     * @brief 计算电机输出值
     * @return 输出值
     * @note 根据控制模式，调用不同的PID计算方法
     * @note 如果电机未使能，输出值为0
     */
    void CalculateAndSetOutput() {
        if (!motor_enabled) {
            outputValue = 0;
            return;
        }
        switch (control_mode) {
        case SPEED_MODE:
            if (pid_speed != NULL) {
                pid_speed->updateControlValue(motor_data.ref_shaft_speed, motor_data.shaft_speed);
                outputValue = pid_speed->calculate();
            } else {
                outputValue = 0;
            }
            break;
        case ANGLE_MODE:
            if (pid_angle != NULL) {
                pid_angle->updateControlValue(motor_data.ref_shaft_angle, motor_data.shaft_angle);
                float speedvalue = pid_angle->calculate();
                pid_speed->updateControlValue(speedvalue, motor_data.shaft_speed);
                outputValue = pid_speed->calculate();
            } else {
                outputValue = 0;
            }
            break;
        case NONE_MODE:
            outputValue = 0;
            break;
        default:
            outputValue = 0;
            break;
        }

        // 输出限幅
        if (outputValue > MaxOutput) {
            outputValue = MaxOutput;
        } else if (outputValue < -MaxOutput) {
            outputValue = -MaxOutput;
        }
    }

    /**
     * @brief 获取电机输出值
     * @return 输出值
     */
    float getOutputValue() const {
        return uint16_t(outputValue);
    }

    /**
     * @brief 解析电机数据
     * @param data 接收到的数据
     * @param len 数据长度
     * @return void
     */
    void decode(const uint8_t *data, uint8_t len) {
        switch (motor_type) {
        case MOTOR_2006:
            /* code */
            if (len == 8) {
                // 解析电机数据
                get2006Data(data, len);
            } else
                // 数据长度错误，处理错误
                error_status |= 0x01; // 设置数据错误标志
            break;
        case MOTOR_3508:
            /* code */
            if (len == 8) {
                // 解析电机数据
                get3508Data(data, len);
            } else
                // 数据长度错误，处理错误
                error_status |= 0x01; // 设置数据错误标志
            break;
        case MOTOR_GM6020:
            /* code */
            if (len == 8) {
                // 解析电机数据
                get6020Data(data, len);
            } else
                // 数据长度错误，处理错误
                error_status |= 0x01; // 设置数据错误标志
            break;
        case MOTOR_NULL:
            // 未设置电机类型，处理错误
            error_status |= 0x10; // 设置数据错误标志
            /* code */
            break;
        default:
            break;
        }
    }

private:

    MotorControlMode getControlMode() const {
        return control_mode;
    }

    /**
     * @brief 角度数据处理函数
     * @param raw_angle 原始反馈角度数据
     * @return void
     * @note 角度数据范围0-8191，对应0-360度
     * @note 计算转子连续圈数和连续角度
     * @note 计算输出轴连续圈数和连续角度
     * @note 计算输出轴非连续角度
     * @note 计算转子非连续角度
     */
    void angleDataHandler(int16_t raw_angle) {
        // 角度数据处理逻辑

        // 更新上一次电机角度数据
        motor_data.last_rotor_angle = motor_data.rotor_angle;
        motor_data.last_shaft_angle = motor_data.shaft_angle;

        // 计算当前转子角度
        motor_data.rotor_angle = (float)raw_angle / 8191.f * 360.f; // 2006电机角度范围0-8191，对应0-360度
        // 转子连续圈数计算
        if (motor_data.rotor_angle - motor_data.last_rotor_angle > 300.f) {
            motor_data.continuous_rotor_round--;
        } else if (motor_data.rotor_angle - motor_data.last_rotor_angle < -300.f) {
            motor_data.continuous_rotor_round++;
        }
        if( fabs(motor_data.continuous_rotor_round) > 6000 )
            motor_data.continuous_rotor_round = 0 ;

        // 计算转子连续角度
        motor_data.continuous_rotor_angle = motor_data.continuous_rotor_round * 360.f + motor_data.rotor_angle;

        // 计算输出轴角度
        if (motor_data.reduction_ratio != 0) {
            motor_data.shaft_angle = (motor_data.continuous_rotor_angle / motor_data.reduction_ratio);
            while(motor_data.shaft_angle >= 360.f )               
                motor_data.shaft_angle -= 360.0f ;
        }
        else
            error_status |= 0x100; // 设置数据错误标志
        // 输出轴连续圈数计算
        if (motor_data.shaft_angle - motor_data.last_shaft_angle > 300.f) {
            motor_data.continuous_shaft_round--;
        } else if (motor_data.shaft_angle - motor_data.last_shaft_angle < -300.f) {
            motor_data.continuous_shaft_round++;
        }

        if(std::fabs(motor_data.continuous_shaft_round) > 6000 )
            motor_data.continuous_shaft_round = 0 ;

        // 计算输出轴连续角度
        motor_data.continuous_shaft_angle = motor_data.continuous_shaft_round * 360.f + motor_data.shaft_angle;
    }

    /**
     * @brief 转速数据处理函数
     * @param raw_speed 原始反馈转速数据
     */
    void speedDataHandler(int16_t raw_speed) {
        // 转速数据处理逻辑
        motor_data.rotor_speed = (float)raw_speed; // 转速单位rpm
        if (motor_data.reduction_ratio != 0)
            motor_data.shaft_speed = motor_data.rotor_speed / motor_data.reduction_ratio; // 输出轴转速，单位rpm
        else
            error_status |= 0x100; // 设置数据错误标志
    }

    /**
     * @brief 电流数据处理函数
     * @param raw_current 原始反馈电流数据
     * @param Keyvalue 映射最大键值
     * @param Realvalue 实际最大值
     */
    void currentDataHandler(int16_t raw_current, int16_t Keyvalue, int16_t Realvalue) {
        // 电流数据处理逻辑
        motor_data.current = (float)raw_current / 5.f; // 电流单位A
    }

    void get2006Data(const uint8_t *data, const uint8_t len) {
        if (len != 8) return; // 数据长度错误，直接返回

        // 解析数据
        int16_t raw_angle   = uint16_t(data[0] << 8) | data[1]; // 角度，单位度
        int16_t raw_speed   = uint16_t(data[2] << 8) | data[3]; // 转速，单位rpm
        int16_t raw_current = uint16_t(data[4] << 8) | data[5]; // 电流，单位A*5

        // 更新电机数据
        angleDataHandler(raw_angle);
        speedDataHandler(raw_speed);
        currentDataHandler(raw_current, 10000, 10); // 2006电机最大电流10A，对应映射值10000
        motor_data.temperature = 0;                 // 2006电机没有温度数据，
    }

    void get3508Data(const uint8_t *data, const uint8_t len) {
        if (len != 8) return; // 数据长度错误，直接返回

        // 解析数据
        int16_t raw_speed       = (data[0] << 8) | data[1]; // 转速，单位rpm
        int16_t raw_current     = (data[2] << 8) | data[3]; // 电流，单位A*5
        int16_t raw_angle       = (data[4] << 8) | data[5]; // 角度，单位度
        int16_t raw_temperature = (data[6] << 8);           // 温度，单位摄氏度

        // 更新电机数据
        angleDataHandler(raw_angle);
        speedDataHandler(raw_speed);
        currentDataHandler(raw_current, 16384, 20);      // 3508电机最大电流20A，对应映射值16384
        motor_data.temperature = (float)raw_temperature; // 温度单位摄氏度
    }

    void get6020Data(const uint8_t *data, const uint8_t len) {
        if (len != 8) return; // 数据长度错误，直接返回

        // 解析数据
        int16_t raw_speed       = (data[0] << 8) | data[1]; // 转速，单位rpm
        int16_t raw_current     = (data[2] << 8) | data[3]; // 电流，单位A*5
        int16_t raw_angle       = (data[4] << 8) | data[5]; // 角度，单位度
        int16_t raw_temperature = (data[6] << 8);           // 温度，单位摄氏度

        // 更新电机数据
        angleDataHandler(raw_angle);
        speedDataHandler(raw_speed);
        currentDataHandler(raw_current, 16384, 3);       // 6020电机最大电流3A，对应映射值16384
        motor_data.temperature = (float)raw_temperature; // 温度单位摄氏度
    }
};

#endif // DJI_MOTORS_HPP
