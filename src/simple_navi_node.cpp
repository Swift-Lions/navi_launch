#include "rclcpp/rclcpp.hpp"

class SimpleNaviNode : public rclcpp::Node
{
public:
    SimpleNaviNode() : Node("simple_navi_node")
    {
        RCLCPP_INFO(this->get_logger(), "Simple Navigation Node Started!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleNaviNode>());
    rclcpp::shutdown();
    return 0;
}
