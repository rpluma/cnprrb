#include <navigation/squarectrl.hpp>
//#include "navigation/srv/resize.hpp"

SquareCtrl::SquareCtrl():Node("SquareCtrl")
{
    publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_SQUARE, 4);
    this->declare_parameter("sqr_size", 4);
    count_=0;
    sqr_size_ = this->get_parameter("sqr_size").get_parameter_value().get<int>();
    // ros2 run navigation sqrController --ros-args -p sqr_size:=5
}

SquareCtrl::~SquareCtrl()
{
    printf("Leaving gently\n");
}

void SquareCtrl::PublishSquare()
{
    auto msg = std_msgs::msg::String();
    if (sqr_size_<=0)
    {
        msg.data = "girar";
        sqr_size_ = this->get_parameter("sqr_size").get_parameter_value().get<int>();
    }
    else
    {
        msg.data = "avanzar";
        sqr_size_ --;
    }
    RCLCPP_INFO (this->get_logger(), "[#%ld] SQR Publishing: '%s'", count_, msg.data.c_str());
    publisher_->publish(msg);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    SquareCtrl p;
    rclcpp::Rate loop_rate(0.5);
    while (rclcpp::ok())
    {
        p.PublishSquare();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

