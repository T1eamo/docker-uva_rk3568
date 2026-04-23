#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>

#define DEVICE_NODE "/dev/mpu6050"  // 请确认您的设备节点名称是否为此

// 定义数学常量
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- 转换系数 ---
// Accel: 16g / 32767
const float ACCEL_SCALE = 16.0f / 32767.0f;
// Gyro: 2000dps / 32767 * (PI / 180) -> 转为 rad/s
const float GYRO_SCALE  = (2000.0f / 32767.0f) * (M_PI / 180.0f);
// --- 数据结构定义 ---
// 必须与内核驱动中的结构体完全一致
struct ybimu_data {
    uint8_t accel[6];      // 3 * int16
    uint8_t gyro[6];       // 3 * int16
    uint8_t mag[6];        // 3 * int16
    uint8_t quat_raw[16];  // 4 * float
    uint8_t euler_raw[12]; // 3 * float
    uint8_t baro_raw[16];  // 4 * float
};




namespace imu_space
{
class Imu
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    int fd;
    struct ybimu_data raw_data;
    int ret;

    rclcpp::Node::SharedPtr node_;
    int16_t buffer_to_int16(uint8_t *buf);
    float buffer_to_float(uint8_t *buf);
public:
    explicit Imu(rclcpp::Node::SharedPtr node);
    ~Imu();
    void start();

};
}  // namespace imu_space
