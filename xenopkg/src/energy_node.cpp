#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
// #include <xenopkg/realtime_publisher_xeno.hpp>
#include <xenopkg_interfaces/srv/joint_srv.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <xenopkg_interfaces/msg/float32_header.hpp>
RT_TASK periodic_task;
std::shared_ptr<rclcpp::Node> node;
int n = 0;
rclcpp::Publisher<xenopkg_interfaces::msg::Float32Header>::SharedPtr pub;

xenopkg_interfaces::msg::Float32Header msg;

float encoder_angle = 0.0;
float encoder_omega = 0.0;

void periodic(void *arg)
{

    msg.data = {0.0};

    RTIME current_time_ticks;
   float J = 7.8e-4;
    float l = 0.27;
    float L0 = 0.45;
    float m = 0.036;
    float beta = 2.55e-4 * 1.0;
    float g = 9.81;
    float gamma_swingup = 8;

    while (true)
    {

        current_time_ticks = rt_timer_read();

        msg.header.stamp.sec = current_time_ticks / 1000000000;
        msg.header.stamp.nanosec = current_time_ticks % 1000000000;

        float E = 0.5 * (J + m * l * l) * 0.7 * (encoder_omega * encoder_omega) + m * g * l * (cos(encoder_angle) - 1);
        msg.data[0] = E;
        pub->publish(msg);
        rt_task_wait_period(NULL);
    }
}

void encoder_callback(const xenopkg_interfaces::msg::Float32Header::SharedPtr message)
{

    encoder_angle = message->data[0];
    encoder_omega = message->data[1];
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("energy_node");
    auto subscription_enc = node->create_subscription<xenopkg_interfaces::msg::Float32Header>("/encoder_data", 1, encoder_callback);
    pub = node->create_publisher<xenopkg_interfaces::msg::Float32Header>("energy_topic", 1);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "nodo creato");

    printf("start task\n");

    rt_task_create(&periodic_task, "periodic", 0, 90, T_JOINABLE);

    rt_task_set_periodic(&periodic_task, TM_NOW, 1000000);

    rt_task_start(&periodic_task, &periodic, 0);

    rclcpp::spin(node);
    rt_task_join(&periodic_task);
    rt_task_delete(&periodic_task);

    rclcpp::shutdown();
    return 0;
}
