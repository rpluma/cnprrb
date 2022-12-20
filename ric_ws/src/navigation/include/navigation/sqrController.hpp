#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "navigation/srv/resize.hpp"
//<node pkg="navigation" exec="keyController"    name="keyController" />

class sqrController:public rclcpp::Node
{
public:
    sqrController();
    void publish_method();
    ~sqrController();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int sqr_size_;
};

