#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <navigation/topics.h>


class RandomCtrl:public rclcpp::Node
{
public:
    RandomCtrl();
    void PublishRandom();
    ~RandomCtrl();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
