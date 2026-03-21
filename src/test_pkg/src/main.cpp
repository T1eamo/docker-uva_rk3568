#include "rclcpp/rclcpp.hpp"
#include "test.hpp" // 假设你的头文件后缀改成了 .hpp
#include <iostream>
#include <memory>

int main(int argc, char **argv) {
    // 1. 初始化 ROS 2 客户端库
    rclcpp::init(argc, argv);

    // 2. 创建节点实例 (对应 ROS 1 的 ros::NodeHandle nd("~"))
    // 在 ROS 2 中，NodeOptions 可以用来配置私有参数等
    auto node = std::make_shared<rclcpp::Node>("test_node", rclcpp::NodeOptions().use_intra_process_comms(true));

    // 3. 实例化检测任务类
    // 注意：ROS 2 推荐使用 std::shared_ptr 而不是原始指针 (new)
    // 你需要修改 Detect 类的构造函数，让其接受 std::shared_ptr<rclcpp::Node>
    auto test_task = std::make_shared<test_space::Test>(node);

    // 4. 设置循环频率 (100Hz)
    rclcpp::WallRate loop_rate(100);

    // 5. 运行循环
    while (rclcpp::ok())
    {
        // 执行检测逻辑
        test_task->runTest();

        // 6. 处理回调 (对应 ros::spinOnce)
        rclcpp::spin_some(node);

        // 7. 休眠以维持频率
        loop_rate.sleep();
    }

    // 8. 关闭
    rclcpp::shutdown();
    return 0;
}