#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//     <node pkg="navigation" exec="rndController"    name="rndController" />

class rndController:public rclcpp::Node
{
public:
    rndController();
    void publish_method();
    ~rndController();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
