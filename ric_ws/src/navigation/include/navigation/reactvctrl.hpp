/*


        Control y Programación de Robots
        Práctica 4: ROS2 - Navegación Autónoma


En esta práctica vamos a diseñar algoritmos de navegación más realistas, comprobando su funcionamiento en
un simulador robótico. ROS2 cuenta con herramientas de visualización y simulación, pero en esta práctica vamos
a utilizar un simulador externo llamado CoppeliaSim. Este simulador permite simular un gran número de robots
móviles y manipuladores reales, implementa motores de física, y permite simular gran variedad de sensores.
Incluye además facilidades para comunicarse con ROS2, de forma que el simulador se comporta como un nodo
que puede suscribirse y publicar topics.

Concretamente, utilizaremos como robot el Pioneer P3DX, un robot diferencial que
dispone de los siguientes sensores: laser 2D hokuyo, bumpers, sensores de ultrasonidos
y una antena que le permite calcular la distancia y ángulo a “beacons” dispuestos en el
entorno del robot. Sin embargo, en esta práctica sólo utilizaremos el láser para explorar
el entorno y no chocar con los obstáculos.

El entorno de simulación CoppeliaSim en el que desarrollaremos la práctica se muestra en la siguiente figura,
en la que se observa el robot situado en una escena 3D y se muestra el barrido del láser (en rojo). Por cuestiones
prácticas, deshabilitaremos la opción de visualización del barrido láser, ya que enlentece la simulación.

Estudiaremos este simulador en detalle en temas posteriores, pero por ahora lo utilizaremos sólo para simular
el robot y sus medidas sensoriales. Sólo debemos saber que se comportará como un nodo ROS2, denominado
“/sim_ros2_interface” y entre otros topics está suscrito a:

- /PioneerP3DX/cmd_vel (geometry_msgs/msg/Twist)

Y publica por los topics:
- /PioneerP3DX/laser_scan (sensor_msgs/msg/LaserScan)
- /PioneerP3DX/odom (nav_msgs/msg/Odometry)
- /PioneerP3DX/ground_truth (geometry_msgs/msg/Pose)

En esta práctica nos centraremos en controlar la navegación del robot utilizando comandos de velocidad lineal
y angular a través del topic /PioneerP3DX/cmd_vel en base a la información sensorial aportada por el
láser en el topic /PioneerP3DX/laser_scan. Los demás topics los podremos utilizar opcionalmente si
queremos trazar la trayectoria seguida, por ejemplo.

1. Lanzando el simulador. La MV que se ofrece en la asignatura ya dispone de los pkg necesarios para lanzar
el simulador Coppelia junto a ROS2. Estos pkgs se encuentran en un workspace denominado
“coppelia_ws”. Dado que los recursos de la MV son muy limitados, estos pkg ya se dan compilados, por
lo que NO hace falta (ni se recomienda) compilarlos nuevamente

a) Para lanzar el simulador y cargar una escena con el robot podemos hacer dos cosas:
- i.  Lanzamos a mano el simulador, abrimos la escena y finalmente damos al “play”. Para ello en un terminal:
        $ cd ~/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/
        $ ./coppeliaSim.sh
      Una vez abierto el simulador, accedemos a File -> Open Scene y seleccionamos la escena
        “coppelia_ws/src/coppelia_sim/coppelia_ros2_pkg/scenes/ros2_pioneer.ttt”
        Finalmente, una vez cargada la escena, tendremos que darle al “play” para comenzar.

- ii. Emplear un launch file para lanzar automáticamente el simulador, cargar la escena y
      arrancarla en un solo paso (más cómodo y eficiente). Para ello, en tu pkg “navigation”
      genera un nuevo launch file (simulador.launch) con el siguiente contenido

b) Comprueba los topics disponibles en el sistema tras lanzar el simulador.

c) Abre una consola y publica por el topic “/PioneerP3DX/cmd_vel” para que el robot realice una
trayectoria circular (no importa si choca con las paredes). Las velocidades máximas por defecto
a emplear son v_max = 1m/s y w_max = 0.1rad/s.

d) Finalmente, inspecciona la información publicada en los topics de ground_truth y del láser.

2. Controlando al robot simulado. Lanza ahora los nodos implementados en la práctica anterior para
controlar el robot de Coppelia desde teclado, de forma aleatoria, o describiendo una trayectoria
cuadrada. Recuerda que tendrás que actualizar el topic por el que se envían los comandos de velocidad
[Twist] desde el nodo “actuator”


    -------------------------------------------------------
    Cambios realizados:
    - fichero simulador.launch: añadida opción launch-prefix
    - actuador.cpp: el topic donde publicar se puede cambiar desde la línea de comandos
        ros2 run navigation actuator
            -> Publicando comandos en '/turtle1/cmd_vel'
        ros2 run navigation actuator --topicpub /PioneerP3DX/cmd_vel
            -> Publicando comandos en '/PioneerP3DX/cmd_vel'
    - fichero simulador2.launch: lanzamiento de nodos de la práctica anterior cambiando el topic


    -------------------------------------------------------

    Pruebas realizadas

    1a) Lanzar el simulador
        ros2 launch navigation simulador.launch

    1b) Lista de topics
        ros2 topic list
            /PioneerP3DX/cmd_vel
            /PioneerP3DX/ground_truth
            /PioneerP3DX/laser_scan
            /PioneerP3DX/odom
            /PioneerP3DX/range_bearing_scan
            /PioneerP3DX/ultrasonic
            /clock
            /parameter_events
            /rosout
            /tf

    1c) Publicar en cmd_vel
        ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.2, y: 0.1, z: 0}, angular: {x: 0, y: 0, z: -0.05}}' --rate 0.1

    1d) Inspeccionar topics
        ros2 topic echo /PioneerP3DX/ground_truth

            ---
            position:
            x: -1.9019098281860352
            y: -1.4602839946746826
            z: 0.14659026265144348
            orientation:
            x: 0.04382689669728279
            y: 0.08676551282405853
            z: -0.44913795590400696
            w: 0.8881587386131287
            ---

         ros2 topic echo /PioneerP3DX/laser_scan
            --
            header:
            stamp:
                sec: 157
                nanosec: 850006103
            frame_id: laser_scan
            angle_min: -2.094395160675049
            angle_max: 2.094395160675049
            angle_increment: 0.006132928654551506
            time_increment: 0.0
            scan_time: 0.05000000074505806
            range_min: 0.0010000000474974513
            range_max: 5.0
            ranges:
            - -0.025628255680203438
            - -0.008906391449272633
            - -0.08543255925178528
            - 0.02077277936041355
            - 0.07310324907302856
            - 0.07225599139928818
            - '...'
            intensities: []
            ---
    2a) Lanzar nodos de práctica anterior
        ros2 launch navigation simulador2.launch

        */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <navigation/topics.h>
#include <math.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "navigation/srv/status.hpp"

class ReactvCtrl:public rclcpp::Node
{
public:
    ReactvCtrl();
    rclcpp::Service<navigation::srv::Status>::SharedPtr server_status_;
    void handle_status_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Status::Request> request,
              std::shared_ptr<navigation::srv::Status::Response> response);
    void CB_Laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void CB_Truth(const geometry_msgs::msg::Pose::SharedPtr msg);
    ~ReactvCtrl();
    float   velocidadLin_; // velocidad lineal en metros por segundo
    float   velocidadAng_; // velocidad angular en radianes por segundo
    int      iTotalTicks_;
    geometry_msgs::msg::Twist msgTwist_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pb_twist_;


private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sb_laser_;
    double  umbralChoque_; // distancia en la que el
    int     evitadosDcha_; // número de obstáculos evitados por la derecha
    int     evitadosIzda_; // número de obstáculos evitados por la izquierda

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr       sb_truth_;
    double   ultPositionX_; // última posición conocida según topic /PioneerP3DX/ground_truth
    double   ultPositionY_; // ídem
    double   totalDistanc_; // distancia recorrida desde que inició el controlador
    int      ticksFinGiro_; // número de invocaciones pendientes hasta terminar el giro




};


