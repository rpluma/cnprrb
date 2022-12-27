#include <navigation/randomctrl.hpp>

RandomCtrl::RandomCtrl():Node("RandomCtrl")
{
    publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_RANDOM, 10);
    count_=0;
}

RandomCtrl::~RandomCtrl()
{
    printf("Leaving gently\n");
}

void RandomCtrl::PublishRandom()
{
    auto msg = std_msgs::msg::String();
    int random = round(4*(float)rand()/(float)RAND_MAX);
    count_++;
    switch (random)
    {
        case 0: msg.data = "stop";      break;
        case 1: msg.data = "forward";   break;
        case 2: msg.data = "backwards"; break;
        case 3: msg.data = "left";      break;
        case 4: msg.data = "right";     break;
    }
    RCLCPP_INFO (this->get_logger(), "[#%ld] Rnd Publishing: '%s'", count_, msg.data.c_str());
    publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    RandomCtrl p;
    rclcpp::Rate loop_rate(0.5);
    while (rclcpp::ok())
    {
        p.PublishRandom();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
