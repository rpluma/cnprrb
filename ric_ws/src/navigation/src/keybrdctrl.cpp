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
            msg.data = STR_CMD_KEYB_INC_V; break;
        case 0x42: // down
            msg.data = STR_CMD_KEYB_DEC_V; break;
        case 0x43: // right
            msg.data = STR_CMD_KEYB_INC_W; break;
        case 0x44: // left
            msg.data = STR_CMD_KEYB_DEC_W; break;
        case 0x20: // stop
            msg.data = STR_CMD_KEYB_STOP; break;
        case 'q': // salir
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
