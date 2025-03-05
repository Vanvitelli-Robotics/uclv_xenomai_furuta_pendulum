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
RT_TASK rtpublish_task;
std::shared_ptr<rclcpp::Node> node;
int n = 0;
rclcpp::Publisher<xenopkg_interfaces::msg::Float32Header>::SharedPtr pub;
rclcpp::Publisher<xenopkg_interfaces::msg::Float32Header>::SharedPtr pub_freq;
// std::shared_ptr<xeno_tools::RealtimePublisher<sensor_msgs::msg::Temperature>> rtpub;
//  RT_MUTEX mutex;
xenopkg_interfaces::msg::Float32Header msg;
xenopkg_interfaces::msg::Float32Header msg_freq;
// rclcpp::Service<xenopkg_interfaces::srv::JointSrv>::SharedPtr joint_service;

void rtpublish(void *arg)
{
  // xenopkg_interfaces::msg::Float32Header msg2;
  while (true)
  {
    // rt_mutex_acquire(&mutex, TM_INFINITE);

    // msg2=msg;

    // rt_mutex_release(&mutex);
    pub->publish(msg);
    pub_freq->publish(msg_freq);

    rt_task_wait_period(NULL);
  }
}

void periodic(void *arg)
{

  msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  msg_freq.data = {0.0};

  RTIME current_time_ticks;
  // RTIME old_time_ticks;
  // double diff=0;
  // double diffmax=0;
  float ts = 1000.0;
  float freq = 0.2;
  float amp = 10.0;
  float time = 0.0;
  float time_step = 1.0 / ts;
  float f_start = 0.01;
  float f_end = 5;
  float duration = 100.0;
  float current_frequency;

  while (true)
  {

    // rclcpp::Time current_time = clock.now();

    // int32_t sec = static_cast<int32_t>(current_time.seconds());
    // int32_t nanosec = static_cast<int32_t>(current_time.nanoseconds());

    current_time_ticks = rt_timer_read();

    msg.header.stamp.sec = current_time_ticks / 1000000000;
    msg.header.stamp.nanosec = current_time_ticks % 1000000000;
    msg_freq.header = msg.header;
    // rt_mutex_acquire(&mutex, TM_INFINITE);
    //  if (n%2==0)
    //  {
    //    msg.data={0.0,0.0,0.0,0.0,0.0,5.0};
    //  }
    //  else{
    //    msg.data={0.0,0.0,0.0,0.0,0.0,10.0};
    //  }

    current_frequency = f_start + (f_end - f_start) * (time / duration);

    msg.data[0] = amp * std::sin(2.0 * M_PI * current_frequency * time);
    msg_freq.data[0] = current_frequency;

    //  if (time>=0.25&&time<=0.5)
    //  {
    //    msg.data[0]=1.0;
    //  }
    //  else{
    //   msg.data[0]=-1.0;
    //  }

    time = time + time_step;

    if (time >= duration)
    {
      msg.data[0] = 0;
      return;
    }

    // msg.header.stamp.sec = current_time_ticks / 1000000000;
    // msg.header.stamp.nanosec = current_time_ticks % 1000000000;
    // msg.temperature=n;
    // rt_mutex_release(&mutex);
    // msg.header.stamp = node->now();

    // printf(" n=%f\n",n);
    // n++;
    // if (rtpub->trylock()) {
    //               rtpub->msg_ = msg;
    //               rtpub->unlockAndPublish();
    //              printf("Messaggio pubblicato!\n");
    //           }
    //    else {

    //           printf("Lock non acquisito, aspettando...\n");
    //       }
    // pub->publish(msg);
    // if (n==1)
    // {
    //   old_time_ticks=current_time_ticks;
    // }

    // diff=current_time_ticks-old_time_ticks;
    // if (diff>diffmax)
    // {
    //   diffmax=diff;
    //    rt_printf("diffmax: %f \n", diffmax);
    // }

    // old_time_ticks=current_time_ticks;
    rt_task_wait_period(NULL);
  }
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

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("xeno_node");

  pub = node->create_publisher<xenopkg_interfaces::msg::Float32Header>("cmd_vel", 1);
  pub_freq = node->create_publisher<xenopkg_interfaces::msg::Float32Header>("current_freq", 1);
  // joint_service=node->create_service<xenopkg_interfaces::srv::JointSrv>("JointPositionSrv",&service_cb);
  // rtpub = std::make_shared<xeno_tools::RealtimePublisher<sensor_msgs::msg::Temperature>>(pub);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "nodo creato");

  // char *str = "periodic";

  printf("start task\n");
  // rt_mutex_create(&mutex, "mutex");
  rt_task_create(&periodic_task, "periodic", 0, 90, 0);
  rt_task_create(&rtpublish_task, "rtpublisher", 0, 90, 0);

  rt_task_set_periodic(&periodic_task, TM_NOW, 1000000);
  rt_task_set_periodic(&rtpublish_task, TM_NOW, 1000000);

  // cpu_set_t cpus;
  // CPU_ZERO(&cpus);
  // CPU_SET(0, &cpus);
  // CPU_SET(1, &cpus);
  // rt_task_set_affinity(&periodic_task,&cpus);
  // rt_task_set_affinity(&rtpublish_task,&cpus);

  rt_task_start(&periodic_task, &periodic, 0);
  rt_task_start(&rtpublish_task, &rtpublish, 0);

  // sensor_msgs::msg::Temperature msg;

  // while (rclcpp::ok())
  // {
  //     msg.data=n;
  //     pub->publish(msg);

  // }

  rclcpp::spin(node);
  rt_task_delete(&periodic_task);
  rt_task_delete(&rtpublish_task);
  // rt_mutex_delete(&mutex);

  rclcpp::shutdown();
  return 0;
}
