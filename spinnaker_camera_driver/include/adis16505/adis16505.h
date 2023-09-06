#ifndef ADIS16505_CLASS_H
#define ADIS16505_CLASS_H

#include <bcm2835.h>
#include "RegisterMap.h"
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <ros/ros.h>

/* SPI模式下的数据延迟(us) */
#define tSTALL 16

/* 片选信号 */
#define CS_PIN RPI_GPIO_P1_07

/* 信号拉低拉高 */
inline void CS_LOW() {
    bcm2835_gpio_write(CS_PIN, LOW);
}

inline void CS_HIGH() {
    bcm2835_gpio_write(CS_PIN, HIGH);
}

/* 类型转换 */
template<typename D>
inline char* to_char(D x) {
    return reinterpret_cast<char*>(x);
}

class ADIS16505{
public:
    ADIS16505(int dec_rate, int filter) : dec_rate_(dec_rate), filter_(filter) 
    {
        if (!bcm2835_spi_begin()) {
            ROS_WARN("bcm2835_ispi_begin failed!\n");
            init_bcm_ = false;
        }
    }

    /* 启动设备 */
    bool setUp();

    /* 单次读取数据 */
    void adisSingleRead();

private:
    /* 设备初始化 */
    bool initADIS16505();

    /* 判断设备是否连接成功 */
    bool isConnected();

    /* 读取某个地址寄存器的值 */
    uint16_t adisReadReg(uint16_t addr);
    /* 向寄存器写地址 */
    void adisWriteReg(uint16_t addr, uint16_t val);

    /* 设置DEC_RATE，控制数据的更新频率 */
    bool adisSetSampleRate();

    /* 设置滤波的等级 */
    void adisHardwareFilterSelect();

public:
    /* 角速度和加速度 */
    double gyro_raw_[3], acc_raw_[3];
private:
    double frequency_;
    int dec_rate_;
    int filter_;
    bool init_bcm_ = true;
};

#endif