#include <iostream>
#include <iterator>
#include <list>

#include <navigation/reactvctrl.hpp>

using std::placeholders::_1;



ReactvCtrl::ReactvCtrl():Node("ReactvCtrl")
{
    sb_sensor_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        TPC_SENSOR, 10, std::bind(&ReactvCtrl::SensarLaser, this, _1));
    //publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_REACTV, 10);
    publisher_ =this->create_publisher<geometry_msgs::msg::Twist>(TPC_REACTV, 10);
    threshold_ = 10;
}


ReactvCtrl::~ReactvCtrl()
{
    printf("Leaving gently\n");
}

void ReactvCtrl::SensarLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int n_ranges = msg->ranges.size();

    //std::vector<float>::const_iterator min_it=
    //std::min_element(msg->ranges.begin(), msg->ranges.end());
    //double nearest = *min_it;
    //int pos=std::distance(msg->ranges.begin(), min_it);
    float   fMin, fMax, fCur, angleMin, angleMax;
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
    geometry_msgs::msg::Twist actuation;

    if (iRng > 0) {
        RCLCPP_INFO(this->get_logger(), "[Reactivo] Measurements: total=%d, range=%d, min=%0.1f", n_ranges, iRng, fMin);
        angleMin = (float)(msg->angle_min+iMin*msg->angle_increment);// *180.0/3.14;
        angleMax = (float)(msg->angle_min+iMax*msg->angle_increment);//*180.0/3.14;
        if  (fMin<threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "[Reactivo] Minimum: index=%d, value=%0.2f", iMin, fMin);
            RCLCPP_INFO(this->get_logger(), "[Reactivo] Maximum: index=%d, value=%0.2f", iMax, fMax);
            RCLCPP_INFO(this->get_logger(), "[Reactivo] AngleMin=%0.2f, AngleMax=%0.2f",
                        angleMin*180.0/3.14, angleMax*180.0/3.14);
            actuation.linear.x=0; // linear_;
            actuation.angular.z= angleMax; // angular_;
            this->publisher_->publish(actuation);
        }

    } else {
        RCLCPP_INFO(this->get_logger(), "[Reactivo] Min=%0.2f, Cur=%0.2f, Max=%0.2f", fMin, fCur, fMax);
        actuation.linear.x=0.2;
        actuation.angular.z= 0;
        this->publisher_->publish(actuation);
    }



    //ROS_INFO_STREAM("Total measurements: "<<n_ranges);
    //ROS_INFO_STREAM("Nearest obstacle at: "<<nearest<< ", index="<<pos);
}

int main (int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(1);
    auto node=std::make_shared<ReactvCtrl>();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
