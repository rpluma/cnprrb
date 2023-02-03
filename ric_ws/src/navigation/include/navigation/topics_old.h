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

Lanzamiento desde launch
------------------------
ros2 launch navigation copelia.launch
ros2 launch navigation navigation.launch
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "reactive"}' -1
ros2 service call /resize_service navigation/srv/Service  "{newsize: 2}"
ros2 service call /status_service navigation/srv/Status



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
----------------------------------------t-------
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "ninguno"}'   -1
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "aleatorio"}' -1
ros2 topic pub /cmd_source std_msgs/msg/String '{"data": "teclado"}'   -1
ros2 topic pub /cmd_sougirce std_msgs/msg/String '{"data": "reactive"}' -1
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

--------------------------- odometria
ros2 topic echo /PioneerP3DX/ground_truth
ros2 topic echo /PioneerP3DX/odom
ros2 topic echo /PioneerP3DX/range_bearing_scan

ros2 topic info /PioneerP3DX/ground_truth
ros2 interface show geometry_msgs/msg/Pose

ros2 topic info /PioneerP3DX/range_bearing_scan


ros2 topic echo /PioneerP3DX/ground_truth
position:
                x                       y                       z
Inicial    -0.14512349665164948     -2.2999820709228516     0.13866107165813446
Avanzar    -0.02084461972117424     -2.299801826477051      0.13869675993919373
Avanzar     0.08411936461925507     -2.3015880584716797     0.13869906961917877
Girar 90º   0.12219083309173584     -2.354740858078003      0.13867346942424774
Avanzar     0.06834430992603302     -1.5821171998977661     0.13866399228572845

position:
  x: 
  y: 
  z: 0.13866399228572845
orientation:
  x: 0.0007462729117833078
  y: -0.0010438234312459826
  z: 0.6096524000167847
  w: 0.7926678657531738

  x: 
  y: 
  z: 



-------------------------- avanzar
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1

ros2 run navigation ReactvCtrl
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.1}}' -1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1

-------------------------- girar
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.1}}' -1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1


------------------------------ atrás
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}' -1

-rate 0.1



 */

