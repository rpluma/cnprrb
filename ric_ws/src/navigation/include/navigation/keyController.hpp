#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//<node pkg="navigation" exec="keyController"    name="keyController" />

class keyController:public rclcpp::Node
{
public:
    keyController();
    int Publicar();
    ~keyController();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

