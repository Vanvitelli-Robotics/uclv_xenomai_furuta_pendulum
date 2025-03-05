#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <realtime_tools/realtime_publisher.hpp>

std::shared_ptr<rclcpp::Node> node;
_Float64 n = 0;
rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub;
std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Temperature>> rtpub;

void periodic()
{

  sensor_msgs::msg::Temperature msg;

  // rclcpp::Time current_time = clock.now();

  // int32_t sec = static_cast<int32_t>(current_time.seconds());
  // int32_t nanosec = static_cast<int32_t>(current_time.nanoseconds());

  // RTIME current_time_ticks = rt_timer_read();

  // int32_t sec = current_time_ticks / 1000000000;
  // int32_t nanosec = current_time_ticks % 1000000000;

  // msg.header.stamp.sec = sec;
  // msg.header.stamp.nanosec = nanosec;
  msg.header.stamp = node->now();
  msg.temperature = n;

  // printf(" n=%f\n",n);
  n++;
  if (rtpub->trylock())
  {
    rtpub->msg_ = msg;
    rtpub->unlockAndPublish();
    // printf("Messaggio pubblicato!\n");
  }
  //  else {

  //         printf("Lock non acquisito, aspettando...\n");
  //     }
  // pub->publish(msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("xeno_node");

  pub = node->create_publisher<sensor_msgs::msg::Temperature>("float_prova", 1);

  rtpub = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Temperature>>(pub);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "nodo creato");

  // char *str = "periodic";

  printf("start task\n");

  auto timer = node->create_wall_timer(std::chrono::duration<double>(0.0005), periodic);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
