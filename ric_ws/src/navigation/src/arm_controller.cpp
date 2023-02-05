#include <navigation/arm_controller.hpp>

using std::placeholders::_1;

ArmController::ArmController():Node("ArmController")
{
    RCLCPP_INFO(this->get_logger(), "Use Q/W move along X axis\n");
    RCLCPP_INFO(this->get_logger(), "Use A/S move along Y axis\n");
    RCLCPP_INFO(this->get_logger(), "Use Z/X move along Z axis\n");

    // suscriptores
    subLaser_ = this->create_subscription<std_msgs::msg::Float32>(
        "/laser_data",  10, std::bind(&ArmController::cb_Laser, this, _1));
    subPose_  = this->create_subscription<geometry_msgs::msg::Pose>(
        "/currentPose", 10, std::bind(&ArmController::cb_Pose,  this, _1));

    // publicadores
    pubArm_  = this->create_publisher<std_msgs::msg::String> ("/cmd_arm", 10);
    pubGoto_ = this->create_publisher<geometry_msgs::msg::Pose> ("/goto", 10);

}

ArmController::~ArmController()
{
    printf("ArmController leaving gently\n");
}


int ArmController::ManageKeyboard()
{
    char input = getchar();
    bool bPublish = true;
    auto msg = std_msgs::msg::String();

    switch (input)
    {
        case 'Q':
            msg.data = "X+"; break;
        case 'W':
            msg.data = "X-"; break;
        case 'A':
            msg.data = "Y+"; break;
        case 'S':
            msg.data = "Y-"; break;
        case 'Z':
            msg.data = "Z+"; break;
        case 'X':
            msg.data = "Z-"; break;
        case 'H':
            msg.data = "home"; break;
        case 'P':
            msg.data = "target"; break;
        case '0':
            return 0; // salir
        default:
            bPublish=false;
    } // switch
    if (bPublish)
    {
        RCLCPP_INFO(this->get_logger(), "\n[ArmController] Publishing '%s'\n", msg.data.c_str());
        count_++;
        pubArm_->publish(msg);
    }
    return 1; // no salir de main
}


void ArmController::cb_Laser(const std_msgs::msg::Float32::SharedPtr msg)
{

}

void ArmController::cb_Pose(const geometry_msgs::msg::Pose::SharedPtr msg)
{

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    system("stty raw"); // Enter in console raw mode to avoid <Enter>
    ArmController p;

    while (rclcpp::ok() && p.ManageKeyboard())
        ;

    system("stty cooked"); //restore the console
    rclcpp::shutdown();
    return 0;
}
