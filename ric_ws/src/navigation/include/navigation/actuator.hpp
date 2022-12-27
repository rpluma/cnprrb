#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <navigation/topics.h>
#include <string>


//

//




class Actuator : public rclcpp::Node
{
public:
    Actuator(char *strTopicPub);
    void Actuate();
    void source_cb(const std_msgs::msg::String::SharedPtr msg); //const;

    void random_cb(const std_msgs::msg::String::SharedPtr msg); //const;
    void keybrd_cb(const std_msgs::msg::String::SharedPtr msg); //const;

    void reactv_cb(const geometry_msgs::msg::Twist::SharedPtr msg); // const;
    void square_cb(const std_msgs::msg::String::SharedPtr msg); // const;

    //void MoveKey();
    ~Actuator();


private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sb_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr random_sb_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keybrd_sb_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reactv_sb_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr square_sb_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    size_t count_;
    float linear_;
    float angular_;
    float delta_lin_;
    float delta_ang_;
    int mode_;
    //float v_;
    //float w_;
};

