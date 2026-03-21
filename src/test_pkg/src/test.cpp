#include "test.hpp"
namespace test_space
{
    Test::Test(rclcpp::Node::SharedPtr node)
    {
        // 构造函数实现
        RCLCPP_INFO(node->get_logger(), "Test class initialized.");
        publisher_ = node->create_publisher<std_msgs::msg::String>("topic", 10);

    }

    Test::~Test()
    {
        // 析构函数实现
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test class destroyed.");
    }

    void Test::runTest()
    {
        // 这里实现你的检测逻辑
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Running detection logic...");
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS 2!";
        publisher_->publish(message);
    }
}


