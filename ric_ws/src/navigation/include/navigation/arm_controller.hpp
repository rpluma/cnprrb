#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"

class ArmController:public rclcpp::Node
{
public:
    ArmController();
    ~ArmController();
    int ManageKeyboard();
    // callbacks por tipo de dato
    void cb_Laser(const std_msgs::msg::Float32::SharedPtr msg);
    void cb_Pose(const geometry_msgs::msg::Pose::SharedPtr msg);

private:
    // variables que contienen los publicadores
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubArm_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubGoto_;

    // variables que contienen los suscriptores
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subLaser_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subPose_;

    size_t count_;
};

