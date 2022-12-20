#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#define MODE_INI 0
#define MODE_RND 1
#define MODE_KEY 2
#define MODE_SQR 3
/*

ros2 topic pub /src_command std_msgs/msg/String '{"data": "ninguno"}' -1
ros2 topic pub /src_command std_msgs/msg/String '{"data": "aleatorio"}' -1

ros2 run navigation keyController
ros2 topic pub /src_command std_msgs/msg/String '{"data": "teclado"}' -1

ros2 run navigation sqrController --ros-args -p sqr_size:=5
ros2 run navigation sqrController
ros2 param set /sqrController sqr_size 3

ros2 topic pub /src_command std_msgs/msg/String '{"data": "cuadrado"}' -1

 */



//

//




class Actuator : public rclcpp::Node
{
public:
    Actuator(char *strTopicPub);
    void cbk_rnd(const std_msgs::msg::String::SharedPtr msg); //const;
    void cbk_key(const std_msgs::msg::String::SharedPtr msg); //const;
    void cbk_sqr(const std_msgs::msg::String::SharedPtr msg); // const;
    void cbk_src(const std_msgs::msg::String::SharedPtr msg); //const;
    void MoveKey();
    void Publish();
    ~Actuator();
    int mode_;


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_rnd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_key_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sqr_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_src_;
    size_t count_;
    float linear_;
    float angular_;
    float v_;
    float w_;
};

