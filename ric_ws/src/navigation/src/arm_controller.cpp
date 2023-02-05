#include <navigation/arm_controller.hpp>
#include <ctype.h>

using std::placeholders::_1;

ArmController::ArmController():Node("ArmController")
{
    RCLCPP_INFO(this->get_logger(), "Use Q/W move along X axis\r");
    RCLCPP_INFO(this->get_logger(), "Use A/S move along Y axis\r");
    RCLCPP_INFO(this->get_logger(), "Use Z/X move along Z axis\r");
    RCLCPP_INFO(this->get_logger(), "Use P/G to Save/Go to position\r");
    RCLCPP_INFO(this->get_logger(), "Use H to go home (vertical)\r");
    RCLCPP_INFO(this->get_logger(), "Use 0 to exit\r");

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
    char strPrefix[100];
    char input = getchar();
    auto msg = std_msgs::msg::String();
    sprintf(strPrefix, "Key=%c, Laser=%0.3f => ", input, float32Laser_);
    if (toupper(input) == 'P') {
        poseHome_ = poseCurrent_;
        RCLCPP_INFO(this->get_logger(),
                    "%s Saving={position={%0.2f, %0.2f, %0.2f}, orientation={%0.2f, %0.2f, %0.2f, %0.2f}} \r\n", strPrefix, poseHome_.position.x,poseHome_.position.y, poseHome_.position.z,
                    poseHome_.orientation.x,poseHome_.orientation.y,poseHome_.orientation.z,poseHome_.orientation.w);
    } else if (toupper(input) == 'G') {
        RCLCPP_INFO(this->get_logger(),
                    "%s Going to={position={%0.2f, %0.2f, %0.2f}, orientation={%0.2f, %0.2f, %0.2f, %0.2f} }\r\n", strPrefix, poseHome_.position.x,poseHome_.position.y, poseHome_.position.z,
                    poseHome_.orientation.x,poseHome_.orientation.y,poseHome_.orientation.z,poseHome_.orientation.w);
        pubGoto_->publish(poseHome_);
    }
    else if (input == '0') { // exit
        RCLCPP_INFO(this->get_logger(),
                    "%s Exiting\r\n", strPrefix);
        return 0;
    }
    else {
        switch (toupper(input))
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
            default:
                return 1; // no salir de main
        } //
        RCLCPP_INFO(this->get_logger(), "%s Publishing '%s'\r\n", strPrefix, msg.data.c_str());
        pubArm_->publish(msg);
    }
    return 1; // no salir de main
}


void ArmController::cb_Laser(const std_msgs::msg::Float32::SharedPtr msg)
{
    float32Laser_ = msg->data;
}

void ArmController::cb_Pose(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    poseCurrent_ = *msg;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    system("stty raw");
    auto node=std::make_shared<ArmController>();

    while (rclcpp::ok() && node->ManageKeyboard())
        rclcpp::spin_some(node);

    system("stty cooked"); //restore the console
    rclcpp::shutdown();
    return 0;
}


