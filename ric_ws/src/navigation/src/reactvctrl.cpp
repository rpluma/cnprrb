#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // constante M_PI
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
    std::vector<float>::const_iterator min_it=
    std::min_element(msg->ranges.begin(),msg->ranges.end());
    double nearest = *min_it;
    //int pos=std::distance(msg->ranges.begin(), min_it);
    ROS_INFO_STREAM("Total measurements: %d", n_ranges);
    ROS_INFO_STREAM("Nearest obstacle at %0.1f meters (pos=%d)", nearest, pos);

    static int iVuelta=0;

    int n_ranges = msg->ranges.size();

    //std::vector<float>::const_iterator min_it=
    //std::min_element(msg->ranges.begin(), msg->ranges.end());
    //double nearest = *min_it;
    //int pos=std::distance(msg->ranges.begin(), min_it);
    float   fMin, fMax, fCur, angleMin, angleMax;
    int     iMin, iMax, iRng, i;
    int     bDiscard;

    iVuelta ++;
    RCLCPP_INFO(this->get_logger(), "Angle Min=%0.1f, Angle Max=%0.1f Angle Incr=%0.1f",
                (float)msg->angle_min*180.0/M_PI, (float)msg->angle_max*180.0/M_PI, (float)msg->angle_increment*180.0/M_PI);

    for (i=0, iMin=-1, iRng=0; i<n_ranges; i++)
    {
        fCur = (float)msg->ranges[i];
        // descartar cualquier medida tomada detrás del robot
        bDiscard = ((((float)msg->angle_min+i*(float)msg->angle_increment) < 0) ||
                    (((float)msg->angle_min+i*(float)msg->angle_increment) > M_PI));
        if ((fCur>=(float)msg->range_min) && (fCur<=(float)msg->range_max) && !bDiscard)
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
        if (iVuelta==10 && !bDiscard){
            RCLCPP_INFO(this->get_logger(), "i=%d, range=%0.1f, degree=%0.1f",
                        (int)i, (float)msg->ranges[i], ((float)msg->angle_min+(float)msg->angle_increment*i)*180.0/M_PI);
        }
    }
    geometry_msgs::msg::Twist actuation;

    if (iRng > 0) {
        //RCLCPP_INFO(this->get_logger(), "[Reactivo] Measurements: total=%d, range=%d, min=%0.1f", n_ranges, iRng, fMin);
        //angleMin = (float)msg->angle_min+iMin*(float)msg->angle_increment);// *180.0/3.14;
        //angleMax = (float)msg->angle_min+iMax*(float)msg->angle_increment);//*180.0/3.14;
        if  (fMin<threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "Min: i=%d, rng=%0.1f m / %0.1fº in(%0.1f, %0.1f)",
                        iMin, fMin, angleMin*180.0/M_PI,    msg->range_min, msg->range_max);
            //RCLCPP_INFO(this->get_logger(), "[Reactivo] Maximum: index=%d, value=%0.2f", iMax, fMax);
            //RCLCPP_INFO(this->get_logger(), "[Reactivo] AngleMin=%0.2f, AngleMax=%0.2f",
            //            angleMin*180.0/3.14, angleMax*180.0/3.14);
            actuation.linear.x=0; // linear_;
            actuation.angular.z=0; //  angleMax; // angular_;
            this->publisher_->publish(actuation);
        }

    } else {
        //RCLCPP_INFO(this->get_logger(), "[Reactivo] Min=%0.2f, Cur=%0.2f, Max=%0.2f", fMin, fCur, fMax);
        actuation.linear.x=0.1;
        actuation.angular.z= 0;
        this->publisher_->publish(actuation);
    }



    //ROS_INFO_STREAM("Total measurements: "<<n_ranges);
    //ROS_INFO_STREAM("Nearest obstacle at: "<<nearest<< ", index="<<pos);
}
//const ::::SharedPtr
void processScanCallback(const sensor_msgs::msg::LaserScan::ConstPtr& msg)
{
    int n_ranges = msg->ranges.size();
    std::vector<float>::const_iterator min_it=
    std::min_element(msg->ranges.begin(),msg->ranges.end());
    double nearest = *min_it;
    int pos=std::distance(msg->ranges.begin(), min_it);
    //ROS_INFO_STREAM("Total measurements: %d", n_ranges);
    //ROS_INFO_STREAM("Nearest obstacle at %0.1f meters (pos=%d)", nearest, pos);
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
