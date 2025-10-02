#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

/***
创建一个 HelloWorld 节点，初始化时输出 “hello world” 日志
***/

class HelloWorldNode : public rclcpp::Node
{
public:
    // ros2 节点父类初始化
    HelloWorldNode() : Node("node_helloworld_class")
    {
        // ros2 系统是否正常运行
        while (rclcpp::ok())
        {
            // 输出日志
            RCLCPP_INFO(this->get_logger(), "Hello World");
            sleep(1); // 休眠
        }
    }
};

int main(int argc, char *argv[])
{
    // ros2 c++ 接口初始化
    rclcpp::init(argc, argv);

    // 创建 ros2 节点对象并初始化
    rclcpp::spin(std::make_shared<HelloWorldNode>());

    // 关闭 ros2 c++ 接口
    rclcpp::shutdown();

    return 0;
}