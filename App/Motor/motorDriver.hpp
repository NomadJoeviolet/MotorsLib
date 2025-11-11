#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "etl/observer.h"

struct message
{
    const uint8_t* data;
    uint8_t len;
};


typedef etl::observer<const message&> Motor_Observer;

class MotorDriver : public Motor_Observer {
protected:
    virtual void decode(const message& msg)=0;
public:
    MotorDriver() = default;
    virtual ~MotorDriver() override = default;

    /**
     * @brief 继承Observer的notification方法实现数据解码
     * @param data 接收到的数据
     * @param len 数据长度
     * @return void
     */
    void notification(const message& msg) {
        decode(msg);
    }
};

#endif // MOTOR_DRIVER_HPP