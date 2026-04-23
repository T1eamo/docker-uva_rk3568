#include "rclcpp/rclcpp.hpp"
#include "imu.hpp"
#include <memory>

int main(int argc, char ** argv)
{
    // 初始化 ROS 2 运行时。
    rclcpp::init(argc, argv);
    // 创建一个普通 ROS 节点。
    // 这里打开 intra-process 通信，后面如果同进程内有多个组件通信，可以减少一次消息拷贝。
    auto node = std::make_shared<rclcpp::Node>(
        "imu_node",
        rclcpp::NodeOptions().use_intra_process_comms(true));

    // 创建业务对象。IMU 设备读取和话题发布都封装在 Imu 类内部。
    auto imu_task = std::make_shared<imu_space::Imu>(node);
    imu_task->start();

    rclcpp::spin(node);

    // 关闭 ROS 2。
    rclcpp::shutdown();
    return 0;
}
