#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <navigation/topics.h>
//<node pkg="navigation" exec="keyController"    name="keyController" />

class KeybrdCtrl:public rclcpp::Node
{
public:
    KeybrdCtrl();
    int Publicar_Tecla();
    ~KeybrdCtrl();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

