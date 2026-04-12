#include "rclcpp/rclcpp.hpp"
#include "test.hpp"
#include <memory>

int main(int argc, char ** argv)
{
    // 初始化 ROS 2 运行时。
    rclcpp::init(argc, argv);
    // 创建一个普通 ROS 节点。
    // 这里打开 intra-process 通信，后面如果同进程内有多个组件通信，可以减少一次消息拷贝。
    auto node = std::make_shared<rclcpp::Node>(
        "uva_node",
        rclcpp::NodeOptions().use_intra_process_comms(true));

    // 创建业务对象。真正的视频管线和工作线程都封装在 Test 类内部，
    // main 只负责生命周期管理，不直接参与采集循环。
    auto test_task = std::make_shared<test_space::Test>(node);

    // 显式启动视频处理相关线程。
    test_task->start();

    // ROS 主循环负责处理订阅、service、timer、参数等回调。
    // 这里不再自己写 while (rclcpp::ok())，避免主线程既管 ROS 又管视频采集，职责混在一起。
    rclcpp::spin(node);

    // spin 退出后，通知业务线程停止并回收资源。
    test_task->stop();

    // 关闭 ROS 2。
    rclcpp::shutdown();
    return 0;
}
