#define MD_START 0 // modo inicio
#define MD_RANDOM 1 // modo aleatorio
#define MD_KEYBRD 2 // modo teclado
#define MD_REACTV 3 // modo reactivo
#define MD_SQUARE 4 // modo cuadrado

#define TPC_SOURCE "/cmd_source"
#define TPC_RANDOM "/cmd_random"
#define TPC_KEYBRD "/cmd_keybrd"
#define TPC_SQUARE "/cmd_square"
#define TPC_REACTV "/cmd_reactv"

#define TPC_SENSOR "/PioneerP3DX/laser_scan"



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





 */

