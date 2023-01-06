
// Topics específicos de publicación y suscripción
#define TPC_SOURCE "/cmd_source" // a qué controlador tiene que escuchar el actuador
#define TPC_RANDOM "/cmd_random" // órdenes generadas por el controlador aleatorio
#define TPC_KEYBRD "/cmd_keybrd" // órdenes generadas por el controlador del teclado
#define TPC_SQUARE "/cmd_square" // órdenes generadas por el generador de cuadrados
#define TPC_REACTV "/cmd_reactv" // órdenes generadas por el controlador reactivo

//----- Topics genéricos donde suscribirse
#define TPC_LASER  "/PioneerP3DX/laser_scan"    // lecturas del láser del robot
#define TPC_TRUTH  "/PioneerP3DX/ground_truth"  // posición real del robot

//----- Topics genéricos donde se publica
#define TPC_TURTLE "/turtle1/cmd_vel"           // comandos a la tortuga
#define TPC_COPPEL "/PioneerP3DX/cmd_vel"       // comandos a Coppelia


//----- Modos en que puede estar el actuador
#define MD_WAIT   0 // modo inicio
#define MD_RANDOM 1 // modo aleatorio
#define MD_KEYBRD 2 // modo teclado
#define MD_REACTV 3 // modo reactivo
#define MD_SQUARE 4 // modo cuadrado

//----- Comandos para modificar el modo del actuador
#define STR_CMD_MODE_WAIT    "esperar"
#define STR_CMD_MODE_RANDOM  "aleatorio"
#define STR_CMD_MODE_KEYBRD  "teclado"
#define STR_CMD_MODE_REACTV  "reactivo"
#define STR_CMD_MODE_SQUARE  "cuadrado"

//----- Comandos enviados desde el teclado
#define STR_CMD_KEYB_INC_V   "v++"
#define STR_CMD_KEYB_DEC_V   "v--"
#define STR_CMD_KEYB_INC_W   "w++"
#define STR_CMD_KEYB_DEC_W   "w--"
#define STR_CMD_KEYB_STOP    "stop"

//----- Comandos generados aleatoriamente
#define STR_CMD_RAND_STOP    "stop"
#define STR_CMD_RAND_FWARD   "forward"
#define STR_CMD_RAND_BWARD   "backwards"
#define STR_CMD_RAND_LEFT    "left"
#define STR_CMD_RAND_RIGHT   "right"

//----- Comandos enviados desde el generador de cuadrados
#define STR_CMD_SQRE_AVAN    "avanzar"
#define STR_CMD_SQRE_GIRA    "girar"



/*


Lanzamiento de nodos desde la línea de comandos
-----------------------------------------------
ros2 run turtlesim  turtlesim_node
ros2 run navigation actuator
ros2 run navigation KeybrdCtrl
ros2 run navigation RandomCtrl
ros2 run navigation ReactvCtrl
ros2 run navigation SquareCtrl

Paso de parámetros argc/argv para determinar la salida del actuador
-----------------------------------------------
ros2 run navigation actuator --topicpub /turtle1/cmd_vel
ros2 run navigation actuator --topicpub /PioneerP3DX/cmd_vel

Cambio de modo utilizando la línea de comandos:
-----------------------------------------------
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "ninguno"}'   -1
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "teclado"}'   -1
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "aleatorio"}' -1
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "cuadrado"}'  -1

Uso de parámetros invocando a SquareCtrl:
-----------------------------------------------
ros2 run navigation SquareCtrl --ros-args -p sqr_size:=2

Uso de parámetros después de invocar a SquareCtrl:
-----------------------------------------------
ros2 param set /SquareCtrl sqr_size 3

Invocación servicio resize
--------------------------
ros2 service call /resize_service navigation/srv/Resize "{newsize: 2}"

Invocación al robot copelia

-------------------------- avanzar
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1

-------------------------- girar
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}' -1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1


------------------------------ atrás
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1

-rate 0.1



 */

