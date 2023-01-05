#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // constante M_PI
#include <navigation/reactvctrl.hpp>
#include <chrono>


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


#define SPIN_FREQ 50
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
    umbralChoque_ = 0.5; // 50 cm de acuerdo al enunciado
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
    RCLCPP_INFO(this->get_logger(), "Status service created");
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

#define STAT_INIC   0 // El controlador acaba de empezar
#define STAT_AVAN   1 // El robot está avanzando
#define STAT_GIRO   2 // El robot está girando


void ReactvCtrl::CB_Laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // int n_ranges = msg->ranges.size();

    static int stat = STAT_INIC;

    using namespace std::chrono;
    static std::chrono::steady_clock::time_point inicioGiro = std::chrono::steady_clock::now();


    //static milliseconds msInicioGiro = 0;

    if (stat == STAT_INIC)                  // Cambiar la velocidad para que empiece a avanzar
    {
        velocidadLin_= VELOCIDAD_LIN;       // moverse en línea recta
        velocidadAng_ = 0;                  // dejar de girar
        stat = STAT_AVAN;
        RCLCPP_INFO(this->get_logger(), "Seguir avanzando. Distancia recorrida = %.1f", totalDistanc_);
        msgTwist_.linear.x = velocidadLin_;
        msgTwist_.angular.z = velocidadAng_;
        this->pb_twist_->publish(msgTwist_);
    }
    else if (stat == STAT_AVAN)             // Comprobar is hay obstáculos y decidir hacia donde girar
    {
        //std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin()+25,msg->ranges.end()-25);
        // La distancia al obstáculo más cercano que quede por detrás del robot no debe tenerse en cuenta
        // reducimos el FOV a +/- 180% (75 posiciones * 0.4º por posición = 30º => el FOV de +/-120º pasa a +/-90º
        std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin()+75,msg->ranges.end()-75);
        double dist = *min_it;
        if (dist < umbralChoque_)        // a punto de chocar
        {
            int pos = std::distance(msg->ranges.begin(), min_it);
            double ang = ((float)msg->angle_min+pos*(float)msg->angle_increment)*180/M_PI;
            inicioGiro = std::chrono::steady_clock::now();

            if (ang > 0) // 0 apunta al frente, antihorario > 0 => obstáculo a izquierda
            {
                evitadosIzda_++;
                RCLCPP_INFO(this->get_logger(), "Inicio giro #%d a la derecha", evitadosIzda_);
                velocidadAng_= -VELOCIDAD_ANG;
                velocidadLin_ = 0;
                RCLCPP_INFO(this->get_logger(), "Obstáculo %3.2fm %.1fº => Giro #%d derecha", dist, ang, evitadosIzda_);
            }
            else // 0 apunta al frente, antihorario < 0 => obstáculo a la derecha
            {
                evitadosDcha_++;                // obstáculo a la derecha
                RCLCPP_INFO(this->get_logger(), "Inicio giro #%d a la izquierda", evitadosDcha_);
                velocidadAng_ = VELOCIDAD_ANG;
                velocidadLin_ = 0;
                RCLCPP_INFO(this->get_logger(), "Obstáculo %3.2fm %.1fº => Giro #%d izquierda", dist, ang, evitadosDcha_);
            }
            msgTwist_.linear.x = velocidadLin_;
            msgTwist_.angular.z = velocidadAng_;
            this->pb_twist_->publish(msgTwist_);

            stat = STAT_GIRO;
        } // si está a punto de chocar
    } // si el estado es avanzar
    else if (stat == STAT_GIRO) // Comprobar si ha pasado el segundo
    {
        std::chrono::steady_clock::time_point ahora = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(ahora - inicioGiro).count() > 1000)
        //if (system_clock::now().time_since_epoch()>(msInicioGiro+1000))
        {
            // detener el robot y pasar al estado inicial
            velocidadLin_ = 0;
            velocidadAng_ = 0;
            stat = STAT_INIC;
        }
    }

    //if (ticksFinGiro_ > 0) { // estoy en mitad de un giro seguir girando
    //    RCLCPP_INFO(this->get_logger(), "Sigo girando, quedan %d", ticksFinGiro_);
    //    ticksFinGiro_--;
    //}
    //
    //else
    //{
    //    ticksFinGiro_ = SPIN_FREQ;          // girar N veces hasta completar un segundo
    //}
    //else
    //{
    //    velocidadLin_= VELOCIDAD_LIN;       // moverse en línea recta
    //    velocidadAng_ = 0;                  // dejar de girar
    //}


    //if (iTotalTicks_% 50 == 0) {
    //    RCLCPP_INFO(this->get_logger(), "NEAREST: %3.2f, Angulo: %3.2f, ---- Iz: %02d,  Dr: %02d, Ds: %.2f",
    //                nearest, angObstaculo, evitadosIzda_,evitadosDcha_, totalDistanc_);
    //    //RCLCPP_INFO(this->get_logger(), "Nearest %.2f  %.2fº at position %d/%d", nearest, angObstaculo, pos, n_ranges);
    //}


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
        // Como no sabemos QoS asegurar que se envían mensajes al menos 1 vez por segundo
        node->iTotalTicks_++;
        if (node->iTotalTicks_ % SPIN_FREQ == 0)
        {
            node->msgTwist_.linear.x = node->velocidadLin_;
            node->msgTwist_.angular.z = node->velocidadAng_;
            node->pb_twist_->publish(node->msgTwist_);
        }

        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
