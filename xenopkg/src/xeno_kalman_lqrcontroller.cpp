#include <eigen3/Eigen/Dense>
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
#include <xenopkg_interfaces/srv/set_mode.hpp>

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
float theta_range_in = 0.15;
float theta_range_out = 0.20;
float MAX_position_robot = 1.34 * 0.5;  // rispetto al centro
float MIN_position_robot = -1.34 * 0.5; // rispetto al centro
float Center_position_robot = -0.75;
bool runswingup = 0; // deve stare a 0
bool control_state = 1;

// matrici per kalman
Eigen::MatrixXd GdA = Eigen::MatrixXd::Zero(11, 11);
Eigen::MatrixXd GdB = Eigen::MatrixXd::Zero(11, 1);
Eigen::MatrixXd GdC = Eigen::MatrixXd::Zero(4, 11);
Eigen::MatrixXd GdD = Eigen::MatrixXd::Zero(4, 1);
Eigen::MatrixXd W = Eigen::MatrixXd::Zero(11, 11);
Eigen::MatrixXd Vv = Eigen::MatrixXd::Zero(4, 4);
Eigen::Vector4d v(9e-6, 0.25, 1e-6, 4e-6);
Eigen::MatrixXd Phat = Eigen::MatrixXd::Zero(11, 11);
Eigen::MatrixXd S = Eigen::MatrixXd::Zero(4, 4);
Eigen::MatrixXd Kgain = Eigen::MatrixXd::Zero(11, 11);
Eigen::VectorXd xhat(11);
Eigen::VectorXd yhat(4);
Eigen::VectorXd y(4);

Eigen::VectorXd eigenKd;

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

    float J = 7.8e-4;
    float l = 0.27;
    float L0 = 0.45;
    float m = 0.036;
    float beta = 2.55e-4 * 1.0;
    float g = 9.81;
    float gamma_swingup = 8;

    float E = 0.5 * (J + m * l * l) * 0.7 * (encoder_omega * encoder_omega) + m * g * l * (cos(encoder_angle) - 1);

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

double kalmanobs()
{

    y(0) = encoder_angle;
    y(1) = encoder_omega;
    y(2) = robot_angle;
    y(3) = robot_omega;

    double ulin = -eigenKd.dot(xhat);

    xhat = GdA * xhat + GdB * ulin;
    Phat = GdA * Phat * GdA.transpose() + W;
    yhat = GdC * xhat + GdD * ulin;

    S = GdC * Phat * GdC.transpose() + Vv;
    Kgain = Phat * GdC.transpose() * S.inverse();

    xhat = xhat + Kgain * (y - yhat);

    Phat = Phat - Kgain * GdC * Phat;

    return ulin;
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

        if ((fabs(encoder_angle) < theta_range_in) && control_state == 1)
        {
            control_state = 0;
            xhat(0) = encoder_angle;
            xhat(1) = encoder_omega;
            xhat(2) = robot_angle;
            xhat(3) = robot_omega / (float)GdC(3, 3);
        }
        else if ((fabs(encoder_angle) > theta_range_out) && control_state == 0)
        {
            // velocity=0;
            control_state = 1;
        }

        if (control_state == 0)
        {

            velocity = kalmanobs();
            // velocity=-velocity;
            // saturazione della velocità prima di darla al robot
            if (velocity > velocity_saturation)
            {
                velocity = velocity_saturation;
            }
            else if (velocity < -velocity_saturation)
            {
                velocity = -velocity_saturation;
            }

            // creo il messaggio di controllo e pubblico
            send_cmd(velocity);
        }
        else
        {

            if (!runswingup)
            {
                velocity = 0.0;
                send_cmd(velocity);
                yhat(0) = 0.0;
                yhat(1) = 0.0;
                yhat(2) = 0.0;
                yhat(3) = 0.0;
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
                yhat(0) = 0.0;
                yhat(1) = 0.0;
                yhat(2) = 0.0;
                yhat(3) = 0.0;
            }
        }
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

void setswingup_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{

    if (request->data)
    {
        if (runswingup)
        {
            RCLCPP_INFO(node->get_logger(), " request cancel,swing up activate");
            response->success = true;
            return;
        }

        RCLCPP_INFO(node->get_logger(), " request accepted,start swingup");
        runswingup = true;
        response->success = true;
        return;
    }
    if (!runswingup)
    {
        RCLCPP_INFO(node->get_logger(), " swingup already deactive");
        response->success = true;
        return;
    }

    RCLCPP_INFO(node->get_logger(), " request accepted,stop swingup");
    runswingup = false;
    response->success = true;
    return;
}

void inizialize_kalman_parameters()
{

    eigenKd = Eigen::Map<Eigen::VectorXd>(K.data(), K.size());
    //  Eigen::MatrixXd GdA = Eigen::MatrixXd::Zero(11, 11);
    GdA(0, 0) = 1.000015357411153;
    GdA(1, 0) = 0.030714522485101;
    GdA(0, 1) = 9.999681563151184e-4;
    GdA(1, 1) = 0.999941432436320;
    GdA(2, 2) = 1.0;
    GdA(0, 3) = 2.189366167112056e-4;
    GdA(1, 3) = 0.434341095034569;
    GdA(2, 3) = -0.001780582631858;
    GdA(3, 3) = 0.942748480245388;
    GdA(4, 3) = 0.031079313467437;
    GdA(0, 4) = 6.558953137019665e-5;
    GdA(1, 4) = 0.127498728873708;
    GdA(2, 4) = 0.006399376573595;
    GdA(3, 4) = -0.049360352587324;
    GdA(4, 4) = 0.999202475256086;
    GdA(6, 5) = 1.0;
    GdA(7, 6) = 1.0;
    GdA(8, 7) = 1.0;
    GdA(9, 8) = 1.0;
    GdA(10, 9) = 1.0;
    GdA(0, 10) = -1.032446841840473e-5;
    GdA(1, 10) = -0.020069614344607;
    GdA(2, 10) = -7.327845638283564e-6;
    GdA(3, 10) = 0.007769828366859;
    GdA(4, 10) = 1.255386165966009e-4;
    // Eigen::MatrixXd GdB = Eigen::MatrixXd::Zero(11, 1);
    GdB(5, 0) = 1.0;
    // Eigen::MatrixXd GdC = Eigen::MatrixXd::Zero(4, 11);
    GdC(0, 0) = 1.0;
    GdC(1, 1) = 1.0;
    GdC(2, 2) = 1.0;
    GdC(3, 3) = -1.935974012365084;
    GdC(3, 4) = 6.352824059520067;
    // Eigen::MatrixXd GdD = Eigen::MatrixXd::Zero(4, 1);
    // Eigen::MatrixXd W = Eigen::MatrixXd::Zero(11, 11);
    W.diagonal().setConstant(1e-8);

    Vv.diagonal() = v;

    //  eigenKd=Eigen::Map<Eigen::VectorXd>(K.data(),K.size());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("controller_node");

    pub = node->create_publisher<xenopkg_interfaces::msg::Float32Header>("cmd_vel", 1);
    auto subscription_enc = node->create_subscription<xenopkg_interfaces::msg::Float32Header>("/encoder_data", 1, encoder_callback);
    auto subscription_states = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, joint_state_callback);
    auto activate_service = node->create_service<std_srvs::srv::SetBool>("activate_lqr_srv", &service_cb);
    auto set_swingup = node->create_service<std_srvs::srv::SetBool>("set_swingup_srv", &setswingup_cb);
    // auto setmode = node->create_client<xenopkg_interfaces::srv::SetMode>("set_mode_srv");
    // joint_service=node->create_service<xenopkg_interfaces::srv::JointSrv>("JointPositionSrv",&service_cb);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "nodo creato");
    // node->declare_parameter<std::vector<double>>("K", {10.6092,1.9020,-0.0703,6.6222,-13.2860,0.0111,0.0112,0.0113,0.0113,0.0114,0.0114});
    node->declare_parameter<std::vector<double>>("K", {11.0751, 1.9855, -0.3144, 6.8825, -13.9388, 0.0114, 0.0114, 0.0115, 0.0116, 0.0116, 0.0117});
    node->declare_parameter<float>("theta_range_in", 0.25);
    node->declare_parameter<float>("theta_range_out", 0.40);
    node->declare_parameter<float>("MAX_position_robot", 1.34 * 0.9);
    node->declare_parameter<float>("MIN_position_robot", -1.34 * 0.9);
    node->declare_parameter<float>("Center_position_robot", -0.75);
    node->declare_parameter<bool>("runswingup", 0);

    node->get_parameter("K", K);
    node->get_parameter("theta_range_in", theta_range_in);
    node->get_parameter("theta_range_out", theta_range_out);
    node->get_parameter("MAX_position_robot", MAX_position_robot);
    node->get_parameter("MIN_position_robot", MIN_position_robot);
    node->get_parameter("Center_position_robot", Center_position_robot);
    node->get_parameter("runswingup", runswingup);

    inizialize_kalman_parameters();

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
