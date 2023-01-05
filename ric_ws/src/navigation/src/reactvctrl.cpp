#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // constante M_PI
#include <navigation/reactvctrl.hpp>

using std::placeholders::_1;

#define SPIN_FREQ 5
#define VELOCIDAD_LIN 0.1
#define VELOCIDAD_ANG 0.1


ReactvCtrl::ReactvCtrl():Node("ReactvCtrl")
{
    // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr       sb_truth_;
    sb_truth_ = this->create_subscription<geometry_msgs::msg::Pose>(
        TPC_TRUTH, 10, std::bind(&ReactvCtrl::CB_Truth, this, _1));
    ultPositionX_ = 0;
    ultPositionY_ = 0;
    totalDistanc_ = 0;

    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sb_laser_;
    sb_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        TPC_LASER, 10, std::bind(&ReactvCtrl::CB_Laser, this, _1));
    umbralChoque_ = 0.5; // 50 cm de acuerdo al enunciado
    evitadosDcha_ = 0;
    evitadosIzda_ = 0;
    ticksFinGiro_ = 0; // cada iteración dura 0.2 segundos

    //publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_REACTV, 10);
    pb_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(TPC_REACTV, 10);
    velocidadLin_ = VELOCIDAD_LIN; // el robot empieza a moverse linealmente
    velocidadAng_ = 0;

    iTotalTicks_ = 0;
}


ReactvCtrl::~ReactvCtrl()
{
    printf("ReactiveCtrl leaving gently\n");
}

void ReactvCtrl::CB_Truth(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    if (iTotalTicks_ ==0) {                 // primera iteración => inicializar
        ultPositionX_ = msg->position.x;
        ultPositionY_ = msg->position.y;
    } else {                                // aumentar distancia
        totalDistanc_ += pow(pow(ultPositionX_-msg->position.x, 2)+pow(ultPositionY_-msg->position.y,2),0.5);
    }

}


void ReactvCtrl::CB_Laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int n_ranges = msg->ranges.size();
    std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin()+25,msg->ranges.end()-25);
    double nearest = *min_it;
    int pos = std::distance(msg->ranges.begin(), min_it);
    double angObstaculo = (float)msg->angle_min+pos*(float)msg->angle_increment;

    if (ticksFinGiro_ > 0) // estoy en mitad de un giro seguir girando
        ticksFinGiro_--;

    else if (nearest < umbralChoque_) {     // a punto de chocar
        ticksFinGiro_ = SPIN_FREQ;          // girar N veces hasta completar un segundo
        velocidadLin_ = 0;                  // parar el robot para que deje de avanzar
        if (angObstaculo > 0) {             // 0 apunta al frente, ángulos antihorarios
            evitadosIzda_++;                // obstáculo a la izquierda
            velocidadAng_= -VELOCIDAD_ANG;  // girar a la derecha
        }
        else {
            evitadosDcha_++;                // obstáculo a la derecha
            velocidadAng_ = VELOCIDAD_ANG;  // girar a la izquierda
        }
    }
    else {
        velocidadLin_= VELOCIDAD_LIN;       // moverse en línea recta
        velocidadAng_ = 0;                  // dejar de girar
    }

    geometry_msgs::msg::Twist actuation;
    actuation.linear.x = velocidadLin_;
    actuation.angular.z = velocidadAng_;
    this->pb_twist_->publish(actuation);

    if (iTotalTicks_% 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "Distance %.2f Izq=%d Der=%d", totalDistanc_, evitadosIzda_,evitadosDcha_);
        RCLCPP_INFO(this->get_logger(), "Nearest %.2f - %.2fº at position %d/%d", nearest, angObstaculo, pos, n_ranges);
    }
    iTotalTicks_++;
    return;

    // static int iVuelta=0;

    //int n_ranges = msg->ranges.size();

    //std::vector<float>::const_iterator min_it=
    //std::min_element(msg->ranges.begin(), msg->ranges.end());
    //double nearest = *min_it;
    //int pos=std::distance(msg->ranges.begin(), min_it);
    /*
    float   fMin, fMax, fCur, angleMin, angleMax, angleCur;
    int     iMin, iMax, iRng, i;
    int     bDiscard;

    iVuelta ++;
    RCLCPP_INFO(this->get_logger(), "Angle Min=%0.1f, Angle Max=%0.1f Angle Incr=%0.1f",
                (float)msg->angle_min*180.0/M_PI, (float)msg->angle_max*180.0/M_PI, (float)msg->angle_increment*180.0/M_PI);

    for (i=0, iMin=-1, iRng=0; i<n_ranges; i++)
    {
        fCur = (float)msg->ranges[i];
        angleCur = (float)msg->angle_min+i*(float)msg->angle_increment;
        // descartar cualquier medida tomada detrás del robot
        bDiscard = ((angleCur < -70*M_PI/180) || (angleCur > 70*M_PI/180));
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
            if (i == iMax)
                RCLCPP_INFO(this->get_logger(), "Max Distance");
            if (i == iMin)
                RCLCPP_INFO(this->get_logger(), "Min Distance");

        }
    }
    geometry_msgs::msg::Twist actuation;

    if (iRng > 0) {
        //RCLCPP_INFO(this->get_logger(), "[Reactivo] Measurements: total=%d, range=%d, min=%0.1f", n_ranges, iRng, fMin);
        //angleMin = (float)msg->angle_min+iMin*(float)msg->angle_increment);// *180.0/3.14;
        //angleMax = (float)msg->angle_min+iMax*(float)msg->angle_increment);// *180.0/3.14;
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
    */
}
//const ::::SharedPtr
/*
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
*/

int main (int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<ReactvCtrl>();
    rclcpp::Rate loop_rate(SPIN_FREQ);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
