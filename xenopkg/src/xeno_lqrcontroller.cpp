#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xenopkg_interfaces/msg/float32_header.hpp>
#include <std_srvs/srv/set_bool.hpp>
RT_TASK controller_task;
RT_TASK spin_task_lqr;
RTIME current_time_ticks;
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<xenopkg_interfaces::msg::Float32Header>::SharedPtr pub;
xenopkg_interfaces::msg::Float32Header msg;
std::vector<double> K;
bool runcontrol = false;
float encoder_angle = M_PI;
float encoder_omega = 0.0;
float robot_angle = 0.0;
float robot_omega = 0.0;
float theta_range = 0.08;
float MAX_position_robot = 1.34 * 0.5;  // rispetto al centro
float MIN_position_robot = -1.34 * 0.5; // rispetto al centro
float Center_position_robot = -0.75;
bool runswingup = 0;

void spin_task_cb(void *arg)
{
    rclcpp::spin(node);
}

void send_cmd(float &velocity)
{

    current_time_ticks = rt_timer_read();
    msg.header.stamp.sec = current_time_ticks / 1000000000;
    msg.header.stamp.nanosec = current_time_ticks % 1000000000;

    if (robot_angle > MAX_position_robot && velocity > 0)
    {
        velocity = 0.0;
    }

    if (robot_angle < MIN_position_robot && velocity < 0)
    {
        velocity = 0.0;
    }

    msg.data[0] = velocity;

    pub->publish(msg);
}

float swing_up()
{

    float J = 4.8e-4;
    float l = 0.27;
    float L0 = 0.45;
    float m = 0.036;
    float beta = 2.55e-4;
    float g = 9.81;
    float gamma_swingup = 20;

    float E = 0.5 * (J + m * l * l) * (encoder_omega * encoder_omega) + m * g * l * (cos(encoder_angle) - 1);

    float beta_correction_term_den = m * l * L0 * cos(encoder_angle);
    if (fabs(beta_correction_term_den) < 1e-5)
    {
        if (beta_correction_term_den > 0)
        {
            beta_correction_term_den = 1e-5;
        }
        else
        {
            beta_correction_term_den = -1e-5;
        }
    }
    float accel = 0;
    accel = -(cos(encoder_angle) * encoder_omega) * (gamma_swingup * E);

    accel += (beta * encoder_omega) / (beta_correction_term_den);
    accel += -(l / L0) * (robot_omega * robot_omega) * sin(encoder_angle);

    return accel;
}

void controller_task_cb(void *arg)
{

    msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    float velocity = 0.0;
    // float velocityswing=0.0;
    float last_vel = 0.0;
    float last_acc = 0.0;
    float velocity_saturation = 2.6;
    float Ts = 0.001;
    float accel = 0.0;

    rt_task_set_periodic(NULL, TM_NOW, 1000000);
    while (rclcpp::ok())
    {
        rt_task_wait_period(NULL);

        if (!runcontrol)
        {
            velocity = 0.0;
            continue;
        }

        if (fabs(encoder_angle) > theta_range)
        {

            if (!runswingup)
            {
                velocity = 0.0;
                send_cmd(velocity);
            }
            else
            {
                // velocity=0.0;
                accel = swing_up();
                velocity = velocity + Ts * accel;
                if (velocity > velocity_saturation)
                {
                    velocity = velocity_saturation;
                }
                else if (velocity < -velocity_saturation)
                {
                    velocity = -velocity_saturation;
                }

                send_cmd(velocity);
            }

            continue;
        }

        // velocityswing=0.0;
        // calcolo acc con i guadagni dell'lqr
        accel = 0.0;
        accel -= K[0] * encoder_angle;
        accel -= K[1] * encoder_omega;
        accel -= K[2] * robot_angle;
        accel -= K[3] * robot_omega;
        accel = -accel; //*********SEGNO MENO INSERITO */
        // integro per ottenere la velocita desiderata
        velocity = velocity + Ts * accel;
        // last_acc=accel;

        // saturazione della velocità prima di darla al robot
        if (velocity > velocity_saturation)
        {
            velocity = velocity_saturation;
        }
        else if (velocity < -velocity_saturation)
        {
            velocity = -velocity_saturation;
        }

        // last_vel=velocity;

        // creo il messaggio di controllo e pubblico
        send_cmd(velocity);
    }
    rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);
}

// void service_cb(const std::shared_ptr<xenopkg_interfaces::srv::JointSrv::Request> request,std::shared_ptr<xenopkg_interfaces::srv::JointSrv::Response> response){

// if(!request->data){
// response->completed=false;
// RCLCPP_INFO(node->get_logger()," request cancel");
// return;

// }
// RCLCPP_INFO(node->get_logger()," request accepted");

//  response->completed=true;

// }
void encoder_callback(const xenopkg_interfaces::msg::Float32Header::SharedPtr message)
{

    encoder_angle = message->data[0];
    encoder_omega = message->data[1];
}
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{

    robot_angle = message->position[0];
    robot_omega = message->velocity[0];
    robot_angle = robot_angle - Center_position_robot;
}

void service_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{

    if (request->data)
    {
        if (runcontrol)
        {
            RCLCPP_INFO(node->get_logger(), " request cancel,lqr task already activate");
            response->success = false;
            return;
        }

        RCLCPP_INFO(node->get_logger(), " request accepted,start lqr task");
        runcontrol = true;
        response->success = true;
        return;
    }
    if (!runcontrol)
    {
        RCLCPP_INFO(node->get_logger(), " lqr task already deactive");
        response->success = false;
        return;
    }

    RCLCPP_INFO(node->get_logger(), " request accepted,stop lqr task");
    runcontrol = false;
    response->success = true;
    return;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("controller_node");

    pub = node->create_publisher<xenopkg_interfaces::msg::Float32Header>("cmd_vel", 1);
    auto subscription_enc = node->create_subscription<xenopkg_interfaces::msg::Float32Header>("/encoder_data", 1, encoder_callback);
    auto subscription_states = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, joint_state_callback);
    auto activate_service = node->create_service<std_srvs::srv::SetBool>("activate_lqr_srv", &service_cb);
    // joint_service=node->create_service<xenopkg_interfaces::srv::JointSrv>("JointPositionSrv",&service_cb);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "nodo creato");
    // node->declare_parameter<std::vector<double>>("K", {-112.913,-23.63,-0.2485,-2.6875});
    node->declare_parameter<std::vector<double>>("K", {-76.1378, -15.9335, -0.0223, -0.2422});
    node->declare_parameter<float>("theta_range", 0.15);
    node->declare_parameter<float>("MAX_position_robot", 1.34 * 0.5);
    node->declare_parameter<float>("MIN_position_robot", -1.34 * 0.5);
    node->declare_parameter<float>("Center_position_robot", -0.75);
    node->declare_parameter<bool>("runswingup", 0);

    node->get_parameter("K", K);
    node->get_parameter("theta_range", theta_range);
    node->get_parameter("MAX_position_robot", MAX_position_robot);
    node->get_parameter("MIN_position_robot", MIN_position_robot);
    node->get_parameter("Center_position_robot", Center_position_robot);
    node->get_parameter("runswingup", runswingup);

    printf("start task\n");

    rt_task_create(&controller_task, "controller_thread", 0, 90, T_JOINABLE);
    // rt_task_set_periodic(&controller_task,TM_NOW,1000000);
    rt_task_start(&controller_task, &controller_task_cb, 0);

    rt_task_create(&spin_task_lqr, "spin_thread_lqr", 0, 90, T_JOINABLE);
    rt_task_start(&spin_task_lqr, &spin_task_cb, 0);
    // rclcpp::spin(node);
    rt_task_join(&spin_task_lqr);

    rt_task_join(&controller_task);

    rt_task_delete(&controller_task);
    // rt_task_delete(&spin_task_lqr);

    rclcpp::shutdown();
    return 0;
}
