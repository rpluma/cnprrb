#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <navigation/topics.h>
#include "navigation/srv/resize.hpp"

class SquareCtrl:public rclcpp::Node
{
public:
    SquareCtrl();
    void PublishSquare();
    ~SquareCtrl();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int sqr_size_;
};

