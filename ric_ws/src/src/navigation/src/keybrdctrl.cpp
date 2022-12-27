#include <navigation/keybrdctrl.hpp>

KeybrdCtrl::KeybrdCtrl():Node("KeybrdCtrl")
{
    publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_KEYBRD, 10);
    count_=0;
    RCLCPP_INFO(this->get_logger(), "Use arrow keys to move the robot.");
    RCLCPP_INFO(this->get_logger(), "Press the space bar to stop the robot.");
    RCLCPP_INFO(this->get_logger(), "Press q to stop the program.");
}

KeybrdCtrl::~KeybrdCtrl()
{
    printf("Leaving gently\n");
}

int KeybrdCtrl::Publicar_Tecla()
{
    char input = getchar();
    bool bPublish = true;
    auto msg = std_msgs::msg::String();

    switch (input)
    {
        case 0x41: // up
            //RCLCPP_INFO(this->get_logger(), "[KeyController] Detected Up key\n");
            msg.data = "v++"; break;
        case 0x42: // down
            //RCLCPP_INFO(this->get_logger(), "[KeyController] Detected Down key\n");
            msg.data = "v--"; break;
        case 0x43: // right
            //RCLCPP_INFO(this->get_logger(), "[KeyController] Detected Right key\n");
            msg.data = "w--"; break;
        case 0x44: // left
            //RCLCPP_INFO(this->get_logger(), "[KeyController] Detected Left key\n");
            msg.data = "w++"; break;
        case 0x20: // stop
            //RCLCPP_INFO(this->get_logger(), "[KeyController] Detected Space key\n");
            msg.data = "stop"; break;
        case 'q': // salir
            //RCLCPP_INFO(this->get_logger(), "[KeyController] Detected q = Exit\n");
            return 0; // salir
        default:
            bPublish=false;
    } // switch
    if (bPublish)
    {
        RCLCPP_INFO(this->get_logger(), "\n[KeyController] Publishing '%s'", msg.data.c_str());
        count_++;
        publisher_->publish(msg);
    }
    return 1; // no salir de main
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    system("stty raw"); // Enter in console raw mode to avoid <Enter>
    KeybrdCtrl p;

    while (rclcpp::ok() && p.Publicar_Tecla())
        ;

    system("stty cooked"); //restore the console
    rclcpp::shutdown();
    return 0;
}
