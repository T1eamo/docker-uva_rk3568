#include "imu.hpp"

namespace imu_space
{
    Imu::Imu(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        // 1. 打开设备文件
        fd = open(DEVICE_NODE, O_RDONLY);
        if (fd < 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "无法打开设备文件");
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO(node_->get_logger(), "Imu constructor");
    }
    // --- 辅助函数：将2字节转为 int16 (小端模式 LSB First) ---
    int16_t Imu::buffer_to_int16(uint8_t *buf)
    {
        // 默认小端: buf[0] 是低8位, buf[1] 是高8位
        return (int16_t)((buf[1] << 8) | buf[0]);
    }

    // --- 辅助函数：将4字节转为 float ---
    float Imu::buffer_to_float(uint8_t *buf)
    {
        float val;
        // 使用 memcpy 避免字节对齐和类型转换带来的 strict-aliasing 问题
        memcpy(&val, buf, 4);
        return val;
    }
    Imu::~Imu()
    {
        // 关闭设备文件
        if (fd >= 0)
        {
            close(fd);
        }
        RCLCPP_INFO(node_->get_logger(), "Imu destructor");
    }
    void Imu::start()
    {
        RCLCPP_INFO(node_->get_logger(), "Imu start");
        while (rclcpp::ok())
        {
            // 2. 读取原始数据 (62字节)
            ret = read(fd, &raw_data, sizeof(struct ybimu_data));
            if (ret != sizeof(struct ybimu_data))
            {
                printf("读取数据不完整,ret=%d\n", ret);
                usleep(100000); // 100ms
                continue;
            }

            // --- 3. 数据处理 ---

            // A. 加速度计 (Accel) -> g
            float ax = buffer_to_int16(&raw_data.accel[0]) * ACCEL_SCALE;
            float ay = buffer_to_int16(&raw_data.accel[2]) * ACCEL_SCALE;
            float az = buffer_to_int16(&raw_data.accel[4]) * ACCEL_SCALE;

            // B. 陀螺仪 (Gyro) -> rad/s
            float gx = buffer_to_int16(&raw_data.gyro[0]) * GYRO_SCALE;
            float gy = buffer_to_int16(&raw_data.gyro[2]) * GYRO_SCALE;
            float gz = buffer_to_int16(&raw_data.gyro[4]) * GYRO_SCALE;

            // C. 四元数 (Quaternion)
            float q0 = buffer_to_float(&raw_data.quat_raw[0]);
            float q1 = buffer_to_float(&raw_data.quat_raw[4]);
            float q2 = buffer_to_float(&raw_data.quat_raw[8]);
            float q3 = buffer_to_float(&raw_data.quat_raw[12]);

            sensor_msgs::msg::Imu msg;

            msg.header.stamp = node_->now();
            msg.header.frame_id = "imu_link";

            msg.linear_acceleration.x = ax * 9.80665f;
            msg.linear_acceleration.y = ay * 9.80665f;
            msg.linear_acceleration.z = az * 9.80665f; // imu单位是g, ROS标准是m/s^2, 需要乘以重力加速度9.80665

            msg.angular_velocity.x = gx;
            msg.angular_velocity.y = gy;
            msg.angular_velocity.z = gz;

            msg.orientation.w = q0;
            msg.orientation.x = q1;
            msg.orientation.y = q2;
            msg.orientation.z = q3;

            publisher_->publish(msg);
        }
    }

}
// namespace imu_space
