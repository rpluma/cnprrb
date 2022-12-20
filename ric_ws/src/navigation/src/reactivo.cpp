#include <iostream>
#include <iterator>
#include <list>

#include <navigation/reactivo.hpp>

using std::placeholders::_1;


Reactivo::Reactivo():Node("reactivo")
{
    //sub_key_  =this->create_subscription<std_msgs::msg::String>("/key_command", 10, std::bind(&Actuator::cbk_key, this, _1));
    sub_copelia_=this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/PioneerP3DX/laser_scan", 10, std::bind(&Reactivo::cb_laser_scan, this, _1));
    // publisher_ = this->create_publisher<std_msgs::msg::String> ("key_command", 10);
}

Reactivo::~Reactivo()
{
    printf("Leaving gently\n");
}

void Reactivo::cb_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int n_ranges = msg->ranges.size();

    //std::vector<float>::const_iterator min_it=
    //std::min_element(msg->ranges.begin(), msg->ranges.end());
    //double nearest = *min_it;
    //int pos=std::distance(msg->ranges.begin(), min_it);
    float   fMin, fMax, fCur;
    int     iMin, iMax, iRng, i;
    for (i=0, iMin=-1, iRng=0; i<n_ranges; i++)
    {
        fCur = (float)msg->ranges[i];
        if ((fCur>=msg->range_min) && (fCur<=msg->range_max))
        {
            iRng++;
            if (iMin==-1) { // primera asignación
                //RCLCPP_INFO(this->get_logger(), "1 i=%d, Cur=%0.2f", iMin, fCur);
                iMin=i;
                iMax=i;
                fMin=fCur;
                fMax=fCur;
            } else if (fCur < fMin) {
                iMin = i;
                fMin = fCur;
                //RCLCPP_INFO(this->get_logger(), "2 i=%d, Min=%0.2f", iMin, fMin);
            } else if (fCur > fMax) {
                iMax = i;
                fMax = fCur;
                //RCLCPP_INFO(this->get_logger(), "3 i=%d, Max=%0.2f", iMax, fMax);
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "[Reactivo] Measurements: total=%d, range=%d", n_ranges, iRng);
    if (iRng > 0) {
        float angleMin = (float)(msg->angle_min+iMin*msg->angle_increment)*180.0/3.14;
        float angleMax = (float)(msg->angle_min+iMax*msg->angle_increment)*180.0/3.14;
        RCLCPP_INFO(this->get_logger(), "[Reactivo] Minimum: index=%d, value=%0.2f", iMin, fMin);
        RCLCPP_INFO(this->get_logger(), "[Reactivo] Maximum: index=%d, value=%0.2f", iMax, fMax);
        RCLCPP_INFO(this->get_logger(), "[Reactivo] AngleMin=%0.2f, AngleMax=%0.2f", angleMin, angleMax);
    } else {
        RCLCPP_INFO(this->get_logger(), "[Reactivo] Min=%0.2f, Cur=%0.2f, Max=%0.2f", fMin, fCur, fMax);
    }

    //ROS_INFO_STREAM("Total measurements: "<<n_ranges);
    //ROS_INFO_STREAM("Nearest obstacle at: "<<nearest<< ", index="<<pos);
}

int main (int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(1);
    auto node=std::make_shared<Reactivo>();
    while (rclcpp::ok()) // rclcpp::spin(node);
    {
        rclcpp::spin_some(node);
        //node->MoveKey();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
