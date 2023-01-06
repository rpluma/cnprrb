#include <navigation/actuator.hpp>
#include <math.h>

using std::placeholders::_1;


Actuator::Actuator(char *strTopicPub): Node("Actuator")
{
    mode_ = MD_START;
    source_sb_=this->create_subscription<std_msgs::msg::String>(TPC_SOURCE, 10, std::bind(&Actuator::source_cb, this, _1));

    random_sb_=this->create_subscription<std_msgs::msg::String>(TPC_RANDOM, 10, std::bind(&Actuator::random_cb, this, _1));
    keybrd_sb_=this->create_subscription<std_msgs::msg::String>(TPC_KEYBRD, 10, std::bind(&Actuator::keybrd_cb, this, _1));
    reactv_sb_=this->create_subscription<geometry_msgs::msg::Twist>(TPC_REACTV, 10, std::bind(&Actuator::reactv_cb, this, _1));
    square_sb_=this->create_subscription<std_msgs::msg::String>(TPC_SQUARE, 10, std::bind(&Actuator::square_cb, this, _1));

    //pub_    =this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    //pub_    =this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);
    pub_      =this->create_publisher<geometry_msgs::msg::Twist>(strTopicPub, 10);

    linear_ = 0;
    angular_ = 0;
    delta_lin_ = 0.5;
    delta_ang_ = 0.5;
}



void Actuator::source_cb(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data==STR_CMD_MODE_RANDOM)
        mode_ = MD_RANDOM;
    else if (msg->data==STR_CMD_MODE_KEYBRD)
        mode_ = MD_KEYBRD;
    else if (msg->data==STR_CMD_MODE_REACTV)
        mode_ = MD_REACTV;
    else if (msg->data==STR_CMD_MODE_SQUARE)
        mode_ = MD_SQUARE;
    else
        mode_ = MD_START;
}

void Actuator::Actuate(char* strController, char* strMsg)
{
    // Visualizar el modo, la acción y cómo se traduce a tipo Twist
    RCLCPP_INFO(this->get_logger(),
        "Mode %d-%s. Command=%s => linear = %0.1f; angular = %0.1frad (%0.1fº)",
        this->mode_, strController, strMsg, linear_, angular_, angular_*180/3.14);

    // reenviar la acción al robot
    geometry_msgs::msg::Twist actuation;
    actuation.linear.x=linear_;
    actuation.angular.z=angular_;
    this->pub_->publish(actuation);
}



Actuator::~Actuator()
{
    printf("Leaving gently\n");
}



// Callback de mensajes recibidos desde el controlador aleatorio
void Actuator::random_cb(const std_msgs::msg::String::SharedPtr msg) {    
    if (mode_==MD_RANDOM && msg!=NULL) {
        // traducir el mensaje de texto al tipo Twist
        if (msg->data==STR_CMD_RAND_STOP)       {linear_=0; angular_=0;}
        else if (msg->data==STR_CMD_RAND_FWARD) {linear_=1; angular_=0;}
        else if (msg->data==STR_CMD_RAND_BWARD) {linear_=-1;angular_=0;}
        else if (msg->data==STR_CMD_RAND_LEFT)  {linear_=0; angular_=-1;}
        else if (msg->data==STR_CMD_RAND_RIGHT) {linear_=0; angular_=1;}
        // enviar al robot
        this->Actuate(STR_CMD_MODE_RANDOM, msg->data.c_str());        
    }
}

// Callback de mensajes recibidos desde el controlador del teclado
void Actuator::keybrd_cb(const std_msgs::msg::String::SharedPtr msg)
{
    if (mode_==MD_KEYBRD && msg != NULL) {
        // traducir el mensaje de texto al tipo Twist        
        if (msg->data==STR_CMD_KEYB_INC_V)            {linear_  += delta_lin_;}
        else if (msg->data==STR_CMD_KEYB_DEC_V)       {linear_  -= delta_lin_;}
        else if (msg->data==STR_CMD_KEYB_INC_W)       {angular_ += delta_ang_;}
        else if (msg->data==STR_CMD_KEYB_DEC_W)       {angular_ -= delta_ang_;}
        else if (msg->data==STR_CMD_KEYB_STOP)        {linear_=0; angular_=0;}
        // enviar al robot
        this->Actuate(STR_CMD_MODE_KEYBRD, msg->data.c_str());
    } 
}



void Actuator::reactv_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (mode_==MD_REACTV && msg!=NULL) {
        // trasladar el mensaje recibido directamente al robot
        linear_ =msg->linear.x;
        angular_=msg->angular.z;
        this->Actuate(STR_CMD_MODE_REACTV,"Twist");
    }
}



void Actuator::square_cb(const std_msgs::msg::String::SharedPtr msg) //const
{
    if (mode_==MD_SQUARE && msg!=NULL) {
        // traducir el mensaje de texto al tipo Twist
        if (msg->data==STR_CMD_SQRE_AVAN) {
            linear_=1;
            angular_=0;
        }
        else if (msg->data==STR_CMD_SQRE_GIRA) {
            linear_=0;
            angular_=M_PI/2;
        }
        // enviar al robot
        this->Actuate(STR_CMD_MODE_SQUARE, msg->data.c_str());
    }
}




int main (int argc, char* argv[])
{
    char defTopicPub[]="/turtle1/cmd_vel";
    char *strTopicPub =defTopicPub; // topic por defecto para escribir en la tortuga

    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(2);

    for (int i=0;i<argc;i++)
        if (strcmp(argv[i],"--topicpub")==0)
            strTopicPub = argv[i+1];


    auto node=std::make_shared<Actuator>(strTopicPub);
    RCLCPP_INFO(node->get_logger(),"Publicando comandos en '%s'", strTopicPub);


    while (rclcpp::ok()) // rclcpp::spin(node);
    {
        rclcpp::spin_some(node);
        node->keybrd_cb(NULL); // la acción se repite si está en modo teclado
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
