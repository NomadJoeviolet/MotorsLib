#ifndef PID_HPP
#define PID_HPP

//采用strategy设计模式实现不同类型的PID控制器
//包括位置式PID和增量式PID

/**
 * @brief PID控制器基类
 * @details 提供PID控制器的基本接口和成员变量
 * @note 该类为纯虚类，不能直接实例化
 * @author nomad
 * @date 2025-10-09
 * @version 1.0
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param error[3] 误差数组，error[0]为当前误差，error[1]为上次误差，error[2]为上上次误差
 * @param integral 积分值
 * @param i_max 积分限幅值
 * @param output 输出值
 * @param output_max 输出限幅值
 * @param m_reference 参考值
 * @param m_feedback 反馈值
 * @param calculate 计算PID输出值
 */
class basePID {
public:
    float kp{};
    float ki{};
    float kd{};
    float error[3]{};
    float integral{};
    float i_max{};
    float output{};
    float output_max{};

    //一定注意不能给变量取名为reference，会导致该变量不可graph，换成其他的就好
    float m_reference{};
    float m_feedback{};

public:
    basePID()          = default;
    virtual ~basePID() = default;

    /**
     * @brief 设置PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param i_max 积分限幅值
     * @param output_max 输出限幅值
     * @return void
     */
    void setParameters(float kp, float ki, float kd, float i_max, float output_max) {
        this->kp         = kp;
        this->ki         = ki;
        this->kd         = kd;
        this->i_max      = i_max;
        this->output_max = output_max;
    }

    /**
     * @brief 更新参考值和反馈值
     * @param reference 参考值
     * @param feedback 反馈值
     * @return void
     */
    void updateControlValue(float reference, float feedback) {
        this->m_reference = reference;
        this->m_feedback  = feedback;
    }

    void clearIntegralandOutput() {
        integral = 0;
        output   = 0;
    }

    /**
     * @brief 计算PID输出值
     * @return 输出值
     */
    virtual float calculate() = 0;
};

/*
    float kp{};
    float ki{};
    float kd{};
    float error[3]{};
    float integral{};
    float i_max{};
    float output{};
    float output_max{};

    float reference{};
    float feedback{};
*/


/**
 * @brief 位置式PID控制器
 * @details 继承自basePID类，实现位置式PID控制算法
 * @note 位置式PID通过当前误差、积分和微分计算输出
 * @author nomad
 * @date 2025-10-09
 * @version 1.0
    * @param kp 比例系数
    * @param ki 积分系数
    * @param kd 微分系数
    * @param error[3] 误差数组，error[0]为当前误差，error[1]为上次误差，error[2]为上上次误差
    * @param integral 积分值
    * @param i_max 积分限幅值
    * @param output 输出值
    * @param output_max 输出限幅值
    * @param m_reference 参考值
    * @param m_feedback 反馈值
    * @param calculate 计算PID输出值

 */
class PositionPID : public basePID {
public:
    PositionPID()  = default;
    ~PositionPID() = default;

    int to_debug;

    /**
     * @brief 构造函数，设置PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param i_max 积分限幅值
     * @param output_max 输出限幅值
     */
    PositionPID(float kp, float ki, float kd, float i_max, float output_max) {
        setParameters(kp, ki, kd, i_max, output_max);
        m_reference = 0;
        m_feedback  = 0;
    }

    /**
     * @brief 计算位置式PID输出值
     * @return 输出值
     * @note 位置式PID通过当前误差、积分和微分计算输出
     * @note 输出值会被限幅在[-output_max, output_max]范围内
     */
    float calculate() {
        // 更新误差
        error[2] = error[1];
        error[1] = error[0];
        error[0] = this->m_reference - m_feedback;

        // 计算积分
        integral += error[0];
        if (integral > i_max) {
            integral = i_max;
        } else if (integral < -i_max) {
            integral = -i_max;
        }

        // 计算输出
        output = kp * error[0] + ki * integral + kd * (error[0] - error[1]);
        if (output > output_max) {
            output = output_max;
        } else if (output < -output_max) {
            output = -output_max;
        }

        return output;
    }
};

class DeltaPID : public basePID {
public:
    DeltaPID()  = default;
    ~DeltaPID() = default;

    int to_debug;

    DeltaPID(float kp, float ki, float kd, float i_max, float output_max) {
        setParameters(kp, ki, kd, i_max, output_max);
    }

    /**
     * @brief 计算增量式PID输出值
     * @return 输出值
     * @note 增量式PID通过当前误差、上次误差和上上次误差计算输出增量
     * @note 输出值会被限幅在[-output_max, output_max]范围内
     */
    float calculate() {
        // 更新误差
        error[2] = error[1];
        error[1] = error[0];
        error[0] = m_reference - m_feedback;

        // 计算输出增量
        float delta_output = kp * (error[0] - error[1]) + ki * error[0] + kd * (error[0] - 2 * error[1] + error[2]);

        // 更新输出
        output += delta_output;
        if (output > output_max) {
            output = output_max;
        } else if (output < -output_max) {
            output = -output_max;
        }

        return output;
    }
};

#endif // PID_HPP