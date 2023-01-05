#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // constante M_PI
#include <navigation/reactvctrl.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


#define SPIN_FREQ 5
#define VELOCIDAD_LIN 0.2
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
    umbralChoque_ = 0.2; // 50 cm de acuerdo al enunciado
    evitadosDcha_ = 0;
    evitadosIzda_ = 0;
    ticksFinGiro_ = 0; // cada iteración dura 0.2 segundos

    //publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_REACTV, 10);
    pb_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(TPC_REACTV, 10);
    velocidadLin_ = VELOCIDAD_LIN; // el robot empieza a moverse linealmente
    velocidadAng_ = 0;
    iTotalTicks_ = 0;

    // servidor de status
    server_status_ = this->create_service<navigation::srv::Status>(
        "status_service", std::bind(&ReactvCtrl::handle_status_service, this, _1, _2, _3));

}


void ReactvCtrl::handle_status_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Status::Request> request,
        std::shared_ptr<navigation::srv::Status::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Status service triggered");

    //------------------- implementación del servicio
    response->tot_distancia = (int) totalDistanc_;
    response->evitados_izda = evitadosIzda_;
    response->evitados_dcha = evitadosDcha_;
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
    // int n_ranges = msg->ranges.size();
    //std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin()+25,msg->ranges.end()-25);
    // La distancia al obstáculo más cercano que quede por detrás del robot no debe tenerse en cuenta
    // reducimos el FOV a +/- 180% (75 posiciones * 0.4º por posición = 30º => el FOV de +/-120º pasa a +/-90º
    std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin()+75,msg->ranges.end()-75);
    double nearest = *min_it;
    int pos = std::distance(msg->ranges.begin(), min_it);
    double angObstaculo = (float)msg->angle_min+pos*(float)msg->angle_increment;

    if (ticksFinGiro_ > 0) { // estoy en mitad de un giro seguir girando
        RCLCPP_INFO(this->get_logger(), "Sigo girando, quedan %d", ticksFinGiro_);
        ticksFinGiro_--;
    }

    else if (nearest < umbralChoque_)
    {
        // a punto de chocar
        RCLCPP_INFO(this->get_logger(), "EVITANDO CHOQUE: %3.2f, Angulo: %3.2f, ---- Iz: %02d,  Dr: %02d, Ds: %.2f",
            nearest, angObstaculo, evitadosIzda_,evitadosDcha_, totalDistanc_);

        ticksFinGiro_ = SPIN_FREQ;          // girar N veces hasta completar un segundo
        velocidadLin_ = 0;                  // parar el robot para que deje de avanzar
        if (angObstaculo > 0)
        {             // 0 apunta al frente, ángulos antihorarios
            RCLCPP_INFO(this->get_logger(), "Inicio giro a la derecha");
            evitadosIzda_++;                // obstáculo a la izquierda
            velocidadAng_= -VELOCIDAD_ANG;  // girar a la derecha
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Inicio giro a la izquierda");
            evitadosDcha_++;                // obstáculo a la derecha
            velocidadAng_ = VELOCIDAD_ANG;  // girar a la izquierda
        }
    }
    else
    {
        velocidadLin_= VELOCIDAD_LIN;       // moverse en línea recta
        velocidadAng_ = 0;                  // dejar de girar
    }

    geometry_msgs::msg::Twist actuation;
    actuation.linear.x = velocidadLin_;
    actuation.angular.z = velocidadAng_;
    this->pb_twist_->publish(actuation);

    if (iTotalTicks_% 50 == 0) {
        RCLCPP_INFO(this->get_logger(), "NEAREST: %3.2f, Angulo: %3.2f, ---- Iz: %02d,  Dr: %02d, Ds: %.2f",
                    nearest, angObstaculo, evitadosIzda_,evitadosDcha_, totalDistanc_);
        //RCLCPP_INFO(this->get_logger(), "Nearest %.2f  %.2fº at position %d/%d", nearest, angObstaculo, pos, n_ranges);
    }
    iTotalTicks_++;
    return;
}



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
