#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <navigation/common.h>
#include <string>


class Actuator : public rclcpp::Node
{
public:
    Actuator();
    ~Actuator();

    // callbacks para ejecutar cuando se recibe un mensaje
    void source_cb(const std_msgs::msg::String::SharedPtr msg);
    void random_cb(const std_msgs::msg::String::SharedPtr msg);
    void keybrd_cb(const std_msgs::msg::String::SharedPtr msg);
    void square_cb(const std_msgs::msg::String::SharedPtr msg);
    void reactv_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    // enviar el mensaje a los simuladores
    void Actuate(char* strController, const char* strMsg);

private:
    int mode_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sb_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr random_sb_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keybrd_sb_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reactv_sb_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr square_sb_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pbCoppel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pbTurtle_;

    size_t count_;
    float linear_;
    float angular_;
    float delta_lin_;
    float delta_ang_;
};

