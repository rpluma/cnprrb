#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // constante M_PI
#include <navigation/reactvctrl.hpp>
#include <chrono>


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


// frecuencia del bucle principal
#define SPIN_FREQ       50
// bajamos la velocidad angular para que con el error no se sobrepase el umbral
#define VELOCIDAD_ANG   0.09
#define VELOCIDAD_LIN   0.15

#define STAT_INIC       0   // El robot inicia un nuevo avance
#define STAT_AVAN       1   // El robot está avanzando
#define STAT_GIRO       2   // El robot está girando


ReactvCtrl::ReactvCtrl():Node("ReactvCtrl")
{
    // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr       sb_truth_;S
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
    //ticksFinGiro_ = 0; // cada iteración dura 0.2 segundos

    //publisher_ = this->create_publisher<std_msgs::msg::String> (TPC_REACTV, 10);
    pb_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(TPC_REACTV, 10);
    velocidadLin_ = VELOCIDAD_LIN; // el robot empieza a moverse linealmente
    velocidadAng_ = 0;
    // iTotalTicks_ = 0;

    // servidor de status
    server_status_ = this->create_service<navigation::srv::Status>(
        "status_service", std::bind(&ReactvCtrl::handle_status_service, this, _1, _2, _3));
    RCLCPP_INFO(this->get_logger(), "Status service created");
    estado_ = STAT_INIC;
}


void ReactvCtrl::handle_status_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Status::Request> request,
              std::shared_ptr<navigation::srv::Status::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Status service triggered");
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
    if (totalDistanc_ == 0 && ultPositionX_==0) {
        // primera iteración => inicializar la posición desde donde calcular la distancia
        ultPositionX_ = msg->position.x;
        ultPositionY_ = msg->position.y;
    }
    else {
        // aumentar distancia
        totalDistanc_ += pow(pow(ultPositionX_-msg->position.x, 2)+pow(ultPositionY_-msg->position.y,2),0.5);
    }
}



void ReactvCtrl::PublicarTwist()
{
    // envía un mensaje al actuador que a su vez lo reenviará al robot
    msgTwist_.linear.x = velocidadLin_;
    msgTwist_.angular.z = velocidadAng_;
    this->pb_twist_->publish(msgTwist_);
}



void ReactvCtrl::CB_Laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // int n_ranges = msg->ranges.size();

    // La distancia al obstáculo más cercano que quede por detrás del robot no debe tenerse en cuenta
    // reducimos el FOV a +/- 180% (100x0.4º=40º => el FOV de +/-120º pasa a +/-80º
    std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin()+100,msg->ranges.end()-100);
    double dist = *min_it;
    int pos = std::distance(msg->ranges.begin(), min_it);
    double ang = ((float)msg->angle_min+pos*(float)msg->angle_increment)*180/M_PI;
    static int iGiros = 0;            // total de giro

    // tiempo transcurrido desde el inicio del giro
    using namespace std::chrono;
    static std::chrono::steady_clock::time_point inicioGiro = std::chrono::steady_clock::now();


    if (estado_ == STAT_INIC)
    {
        // Cambiar la velocidad para que empiece a avanzar
        RCLCPP_INFO(this->get_logger(), "Seguir avanzando. Distancia recorrida = %.1f", totalDistanc_);
        velocidadLin_= VELOCIDAD_LIN;
        velocidadAng_ = 0;
        PublicarTwist();
        iGiros = 0;
        estado_ = STAT_AVAN;
    }
    else if (estado_ == STAT_AVAN)
    {
        // Comprobar is hay obstáculos y decidir hacia donde girar
        if (dist < umbralChoque_)
        {
            // a punto de chocar, apuntar el timestamp de inicio de giro
            inicioGiro = std::chrono::steady_clock::now();

            if (ang > 0)
            {
                // 0 apunta al frente, antihorario > 0 => obstáculo a izquierda
                evitadosIzda_++;
                velocidadAng_= -VELOCIDAD_ANG;
                velocidadLin_ = 0;
                RCLCPP_INFO(this->get_logger(), "Obstáculo %3.2fm %.1fº => Giro #%d derecha", dist, ang, evitadosIzda_);
            }
            else
            {
                // 0 apunta al frente, antihorario < 0 => obstáculo a la derecha
                evitadosDcha_++;
                velocidadAng_ = VELOCIDAD_ANG;
                velocidadLin_ = 0;
                RCLCPP_INFO(this->get_logger(), "Obstáculo %3.2fm %.1fº => Giro #%d izquierda", dist, ang, evitadosDcha_);
            }
            PublicarTwist();
            iGiros = 1;
            estado_ = STAT_GIRO;
        } // si se está a punto de chocar
    } // si el estado es avanzar

    else if (estado_ == STAT_GIRO)
    {
        std::chrono::steady_clock::time_point ahora = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(ahora - inicioGiro).count() > 1000)
        {
            // Ha pasado un segundo desde que se dio la última orden de giro
            float angGirado = (float)(iGiros*VELOCIDAD_ANG)*180/M_PI;
            if (dist < umbralChoque_ && angGirado < 90) {
                // si sigue el obstáculo continuar girando en la misma dirección hasta 90º
                inicioGiro = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(), "Seguir girando por %d vez (ángulo total=%0.1fº)", iGiros++, angGirado);
                PublicarTwist();
            }
            else {
                // si no hay obstáculo o sigue habiéndolo después de 90º intentar avanzar
                velocidadLin_ = 0;
                velocidadAng_ = 0;
                estado_ = STAT_INIC;
            }
        } // ha pasado un segundo desde la última orden de giro
    } // si el estado es girar
}



int main (int argc, char* argv[])
{
    int iTicks = 0;
    rclcpp::init(argc, argv);
    auto node=std::make_shared<ReactvCtrl>();
    rclcpp::Rate loop_rate(SPIN_FREQ);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // En caso de que QoS=1 y se pierdan mensajes, reenviar al menos 1 vez por segundo
        iTicks++;
        if (iTicks % SPIN_FREQ == 0)
            node->PublicarTwist();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
