#include"std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "hello ,i am %s.", name.c_str());
		// 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
		// 创建定时器，500ms为周期，定时发布
		timer_ = this -> create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback,this));
    }

private:
	void timer_callback()
	{
		// create message
		std_msgs::msg::String message;
		message.data = "forward";
		// print log
		RCLCPP_INFO(this -> get_logger(), "publishing: '%s'", message.data.c_str());
		// publish message
		command_publisher_->publish(message);
	}
	// declare timer pointer
	rclcpp::TimerBase::SharedPtr timer_;
    // declare topic publish pointer
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

