# Lect 2 - Robot Control Architectures II

## Setup

- Crear la máquina
- Carpeta compartida

### Carpeta compartida MV

https://askubuntu.com/questions/29284/how-do-i-mount-shared-folders-in-ubuntu-using-vmware-tools
```
vmware-hgfsclient
sudo mkdir /mnt/hgfs/
sudo vmhgfs-fuse .host:/gierm2020 /mnt/hgfs/ -o allow_other -o uid=1000
```

## ROS2 COLCON


```
mkdir –p ~/dev_ws/src
cd ~/dev_ws/
colcon build
ls
echo "source ~/dev_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- mkdir -p no funciona




## Creating packages

ros2 pkg create --build-type ament_cmake <package_name>

```
cd ~/dev_ws/src
ros2 pkg create --build-type ament_cmake first_pkg
ls first_pkg
cd ~/dev_ws
colcon build
```
### Templates

```
template <class externalType>
class person {
	private: 
		externalType work;
	public:
		Person(externalType n) : work(n)
		{
			...
		}
};

template<class T>class shared_ptr;
```


### KDEVELOP

- run from console: kdevelop
- Open project
- Pick: src folder
- Name: src
- Open configuration
- Pick src > Custom Build System > 
	- Executable: /usr/bin/colcon
	- Arguments: build
- Pick language support
	- Includes: /opt/ros/humble/include/
	- Defines: AMENT_PREFIX /opt/ros/humble
	- Defines: PYTHONPATH /opt/ros/humble/lib/python3.8/site-packages
	- C/C++ Parser: c++17/c99/CL1.1/c++11


### Implement node
- src/first_pkg/src/hello.cpp
- src/first_pkg/CMakeList.txt
```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
	include_directories(
		include
	)
add_executable(hello src/hello.cpp)
	add_executable(controller src/controller.cpp)
	add_executable(actuator   src/actuator.cpp)

install(TARGETS
	hello
		controller
		actuator

	DESTINATIN lib/${PROJECT_NAME})

ament_target_dependencies(hello rclcpp std_msgs)
	ament_target_dependencies(controller rclcpp std_msgs)
	ament_target_dependencies(actuator rclcpp std_msgs geometry_msgs)

# install launch files
install(DIRECTORY
	launch
	ESTINATION share/${PROJECT_NAME}/
)
```
- FI8 or colcon build
- ros2 run first_pkg hello


### Controller
```
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
class Controller : public rclcpp::Node
{
public:
	Controller();
	void publish_method();
	~Controller();
private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
}
------
#include <first_pkg/controller.hpp>
Controller::Controller():Node("controller")
{
	publisher_ = this->crete_publisher<std_msgs::msg::String> ("/topicX", 10);
	count_ = 0;
}
void Controller::publish_method()
{
	auto message = std_msgs::msg::String();
	message.data = "abcd";
	RCLCPP_INFO(this->get_logger(), "fghi");
	publiser_->publish(message);

}
Controller::~Controller()
{
	printf("Leaving gently\n");
}
```

### Main
```
int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv); // ros initialization
	Controller p;
	rclcpp::Rate loop_rate(10);
	while(rclcpp::ok())
	{
		p.publish_method();
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}
```

### Run
```
cd dev_ws
ros2 node list
ros2 topic list
ros2 topic info /command
```


## Creating/communicating nodes

### Subscribe (actuator.cpp)
```
#include rclcpp std_msgs
#include "geometry_msgs/msg/twist.hpp"
#include <string>
class Auctuator:public rclcpp::Node
{
public:
	Actuator();
	void execute_command(const std_msgs::msg::String::SahredPtr msg) const;

private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
	size_t count_;
}
----------
#include <first_pkg/actuator.cpp>
using std::placeholders::_1;
Actuator::Actuator(): Node("actuator")
{
	pub_=this->create_publisher<geometry_msgs::msg::Twist> ("/turtle1/cmd_vel", 10);
	sub_=this->create_subscription<std_msgs::msg::String>("/action", 10,
		std::bind(&Actuator::execute_command, this, _1));		
}
void Actuator::execute_command(const std_msgs::msg::String::SharedPtr msg) const
{
	geometry_msgs::msg::Twist actuation;
	RCLCPP_INFO(this->get_logger(), "Received '%s'",msg->data.c_str());
	actuation.linear.x=1;
	actuation.angular.z=0;
	pub_->publish(actuation);
}
```

main
```
rclcpp::init (argc, argv);
auto node=std::make_hared<Actuator>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
```

```
ros2 run first_pkg controller
ros2 run first_pkg manager_node
ros2 run turtlesim turtlesim_nodes
```

## Launcch command

Crear carpeta y fichero first_pkg/launch/file.launch

```
<launch>
	<node pkg="first_pkg" exec="controller" name="controller" />
	<node pkg="first_pkg" exec="actuator" name="actuator" />
	<node pkg="turtlesim" exec="turtlesim" name="simulator" />
</launch>
```

ros2 launch fisrt_pkg file.launch

### Automated running. roslaunch command
- output="screen"
- launch-prefix="make-terminal --tab --tilte='tabname' -x"
- args="param1 param2..."
- <param name="name" type="type" value="value" />


## Services

Service configuration: srv/**S**tart.srv
```
int64 A
int64 B
---
int64 Sum
```
Service configuration: srv/**S**top.srv
```
string reason
---
int32[5] movements
int32 total
```


Server configuration: package.xml
```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Service config: CmakeList.txt (antes de ament_package())
```
find_package(rosidl_default_generators REQUIRED)
rosidl_genarate_interfaces(${PROJECT_NAME}
"srv/Start.srv"
"srv/Stop.srv"
)

...
ament_package()
...

... add_executable
...

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
"rosidl_typesupport_cpp")
target_link_libraries(actuator "${cpp_typesupport_target}")
```

Compilar para generar start.hpp

nodo: actuator.hpp
```
...
#include "first_pkg/srv/start.hpp"
#include "first_pkg/srv/stop.hpp"
Class Actuator : public rclcpp :: Node
...
private:
	bool active; // variable adicional de ejemplo del servicio
	int movments[5]; // variable adicional para ejmplo stop
	rclcpp::Service<first_pkg::srv::Start>::SharedPtr server_start_;
	void handle_start_service(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<first_pkg::srv::Start::Request> request,
		std::shared_ptr<first_pkg::srv::Start::Response> response);
		)
	void handle_stop_service(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<first_pkg::srv::Stop::Request> request,
		std::shared_ptr<first_pkg::srv::Stop::Response> response);
		)
```

nodo: actuator.cpp
```
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

Actuator::Actuator():Node("actuator")
{
	server_start_ = this->create_service<first_pkg::srv::Start>
	("start_service", std::bind(&Actuator::handle_start_service, this, _1, _2, _3));
	active = false;

...

void Actuator::handle_start_service(
	const std::shared_ptr<rmw_request_id_t> request_header,
	const std::shared_ptr<first_pkg::srv::Start::Request> request,
	std::shared_ptr<first_pkg::srv::Start::Response> response)	
{
	RCLCPP_INFO(this->get_logger(), "Service started");
	active=true;
}


void Actuator::handle_stop_service(
	const std::shared_ptr<rmw_request_id_t> request_header,
	const std::shared_ptr<first_pkg::srv::Start::Request> request,
	std::shared_ptr<first_pkg::srv::Start::Response> response)	
{
	RCLCPP_INFO(this->get_logger(), "Service stopped");
	RCLCPP_INFO(this->get_logger(), "Reason %s",request->reason.c_str()");	
	active=false;
	for (auto i=0;i<5;i++)
		response->movements[i]=movements[i];
}



void Actuator::execute_command(const...)
{
	if (active)
	{	
...

if(mst->data=="stop") {... movements[0]++}

```

Invocación 
```
ros2 service call /start_service first_pkg/srv/Start
ros2 service call /stop_service first_pkg/srv/Stop
```

# Copelia

## Topics

Suscripciones
- /PioneerP3DX/cmd_vel (geometry_msgs/msg/Twist)

Publicaciones
- /PioneerP3DX/laser_scan (sensor_msgs/msg/LaserScan)
- /PioneerP3DX/odom (nav_msgs/msg/Odometry)
- /PioneerP3DX/ground_truth (geometry_msgs/msg/Pose)

## Lanzar simulador
```
cd ~/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/
./coppeliaSim.sh
File -> Open Scene -> coppelia_ws/src/coppelia_sim/coppelia_ros2_pkg/scenes/ros2_pioneer.ttt"

ros2 launch navigation simulador.launch
ros2 topic list
ros2 topic info /PioneerP3DX/cmd_vel
ros2 interface show geometry_msgs/msg/Twist
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1, y: 0.1, z: 0}, angular: {x: 0, y: 0, z: -0.1}}' --rate 0.1
ros2 topic pub /PioneerP3DX/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.5, y: 0.1, z: 0}, angular: {x: 0, y: 0, z: -0.1}}' --rate 0.1

ros2 topic echo /PioneerP3DX/ground_truth
ros2 topic echo /PioneerP3DX/laser_scan


ros2 topic pub /src_command std_msgs/msg/String '{"data": "ninguno"}' -1
ros2 topic pub /src_command std_msgs/msg/String '{"data": "aleatorio"}' -1

ros2 run navigation keyController
ros2 topic pub /src_command std_msgs/msg/String '{"data": "teclado"}' -1

ros2 run navigation sqrController --ros-args -p sqr_size:=5
ros2 run navigation sqrController
ros2 param set /sqrController sqr_size 3

ros2 topic pub /src_command std_msgs/msg/String '{"data": "cuadrado"}' -1



```
