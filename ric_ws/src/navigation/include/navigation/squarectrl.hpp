#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <navigation/topics.h>
#include "navigation/srv/resize.hpp"

class SquareCtrl:public rclcpp::Node
{
public:
    SquareCtrl();
    void PublishSquare();
    rclcpp::Service<navigation::srv::Resize>::SharedPtr server_resize_;
    void handle_resize_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Resize::Request> request,
              std::shared_ptr<navigation::srv::Resize::Response> response);
    ~SquareCtrl();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int sqr_size_; // size desde línea de comandos
    int sqr_srv_size_; // size desde servicio (prevalece)
};

