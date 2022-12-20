#include <navigation/actuator.hpp>
#include <math.h>

using std::placeholders::_1;


Actuator::Actuator(char *strTopicPub): Node("actuator")
{
    sub_rnd_=this->create_subscription<std_msgs::msg::String>("/rnd_command", 10, std::bind(&Actuator::cbk_rnd, this, _1));
    sub_key_=this->create_subscription<std_msgs::msg::String>("/key_command", 10, std::bind(&Actuator::cbk_key, this, _1));
    sub_sqr_=this->create_subscription<std_msgs::msg::String>("/sqr_command", 10, std::bind(&Actuator::cbk_sqr, this, _1));
    sub_src_=this->create_subscription<std_msgs::msg::String>("/src_command", 10, std::bind(&Actuator::cbk_src, this, _1));
    //pub_    =this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    //pub_    =this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);
    pub_      =this->create_publisher<geometry_msgs::msg::Twist>(strTopicPub, 10);

    v_ = 0.1;
    w_ = 0.1;
    mode_ = MODE_INI;
}


Actuator::~Actuator()
{
    printf("Leaving gently\n");
}

void Actuator::Publish()
{
    geometry_msgs::msg::Twist actuation;
    actuation.linear.x=linear_;
    actuation.angular.z=angular_;

    RCLCPP_INFO(this->get_logger(),
        "Mode %d: linear = %0.1f; angular = %0.1frad = %0.1fº",
        this->mode_, linear_, angular_, angular_*180/3.14);
    this->pub_->publish(actuation);
}


void Actuator::cbk_rnd(const std_msgs::msg::String::SharedPtr msg) //const
{
    if (mode_ != MODE_RND)
        return;

    //geometry_msgs::msg::Twist actuation;
    //float linear, angular;

    RCLCPP_INFO(this->get_logger(), "Random-Received: '%s'",msg->data.c_str());

    if (msg->data=="stop")           {linear_=0; angular_=0;}
    else if (msg->data=="forward")   {linear_=1; angular_=0;}
    else if (msg->data=="backwards") {linear_=-1;angular_=0;}
    else if (msg->data=="left")      {linear_=0; angular_=-1;}
    else if (msg->data=="right")     {linear_=0; angular_=1;}
    //actuation.linear.x=linear;
    //actuation.angular.z=angular;
    //pub_->publish(actuation);
    //this->Publish(actuation);
    this->Publish();//linear, angular);
}

void Actuator::cbk_key(const std_msgs::msg::String::SharedPtr msg) //const
{
    if (mode_ != MODE_KEY)
        return;

    RCLCPP_INFO(this->get_logger(), "Keyboard-Received: '%s'",msg->data.c_str());
    if (msg->data=="v++")            {v_++;}
    else if (msg->data=="v--")       {v_--;}
    else if (msg->data=="w++")       {w_++;}
    else if (msg->data=="w--")       {w_--;}
    else if (msg->data=="stop")      {v_=0;w_=0;}
}


void Actuator::MoveKey()
{
    if (mode_ == MODE_KEY)
    {
        //geometry_msgs::msg::Twist actuation;
        //actuation.linear.x=v_;
        //actuation.angular.z=w_;
        //pub_->publish(actuation);
        linear_ = v_;
        angular_ = w_;
        this->Publish();//actuation);
    }
}


void Actuator::cbk_sqr(const std_msgs::msg::String::SharedPtr msg) //const
{
    if (mode_ != MODE_SQR)
        return;

    //geometry_msgs::msg::Twist actuation;
    //float linear, angular;

    RCLCPP_INFO(this->get_logger(), "Square-Received: '%s'",msg->data.c_str());

    if (msg->data=="avanzar")
    {
        linear_=1;
        angular_=0;

    }
    else if (msg->data=="girar")
    {
        linear_=0;
        angular_=M_PI/2;

    }
    //actuation.linear.x=linear;
    //actuation.angular.z=angular;
    //pub_->publish(actuation);
    this->Publish(); // linear, angular);
}


void Actuator::cbk_src(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data=="aleatorio")
        mode_ = MODE_RND;
    else if (msg->data=="teclado")
        mode_ = MODE_KEY;
    else if (msg->data=="cuadrado")
        mode_ = MODE_SQR;

}

int main (int argc, char* argv[])
{
    char *defTopicPub="/turtle1/cmd_vel";
    char *strTopicPub=defTopicPub; // topic por defecto para escribir en la tortuga

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
         if (node->mode_ == MODE_KEY)
             node->Publish();
        //node->MoveKey();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
