#include <navigation/squarectrl.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

SquareCtrl::SquareCtrl():Node("SquareCtrl")
{

    publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_SQUARE, 4);
    this->declare_parameter("sqr_size", 4);
    count_=0;
    sqr_size_ = this->get_parameter("sqr_size").get_parameter_value().get<int>();

    server_resize_ = this->create_service<navigation::srv::Resize>(
        "resize_service", std::bind(&SquareCtrl::handle_resize_service, this, _1, _2, _3));

    RCLCPP_INFO(this->get_logger(), "Resize service created");
}

void SquareCtrl::handle_resize_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Resize::Request> request,
        std::shared_ptr<navigation::srv::Resize::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Resize service triggered");

    //------------------- solo para depurar
    int oldsize, newsize;
    oldsize = this->get_parameter("sqr_size").get_parameter_value().get<int>();
    newsize = request->newsize;
    RCLCPP_INFO(this->get_logger(), "Oldsize %d, Newsize%d", oldsize, newsize);

    //------------------- implementación del servicio
    response->oldsize = this->get_parameter("sqr_size").get_parameter_value().get<int>();
    rclcpp::Parameter size_param("sqr_size", request->newsize);
    this->set_parameter(size_param);

}


SquareCtrl::~SquareCtrl()
{
    printf("Leaving gently resize\n");
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
    //SquareCtrl p;
    auto node=std::make_shared<SquareCtrl>();

    rclcpp::Rate loop_rate(0.5);
    while (rclcpp::ok())
    {
        node->PublishSquare();
        rclcpp::spin_some(node);
        //rclcpp::spin_once(node); // no compila
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

