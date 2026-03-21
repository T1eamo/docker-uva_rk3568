#include <iostream>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std;
namespace test_space
{

    class Test
    {
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    public:
        explicit Test(rclcpp::Node::SharedPtr node);
        ~Test();

        void runTest();
    };



}
