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
#include <std_srvs/srv/set_bool.hpp>

#include "serial/serial.h"
RT_TASK serial_task;
// RT_TASK rtpublish_task;
std::shared_ptr<rclcpp::Node> node;
int n = 0;
rclcpp::Publisher<xenopkg_interfaces::msg::Float32Header>::SharedPtr pub;
// std::shared_ptr<xeno_tools::RealtimePublisher<sensor_msgs::msg::Temperature>> rtpub;
// RT_MUTEX mutex;
xenopkg_interfaces::msg::Float32Header msg;
bool runserial = false;
// rclcpp::Service<xenopkg_interfaces::srv::JointSrv>::SharedPtr joint_service;
std::unique_ptr<serial::Serial> my_serial;

#define ERROR_COLOR "\033[1m\033[31m"   /* Bold Red */
#define WARN_COLOR "\033[1m\033[33m"    /* Bold Yellow */
#define SUCCESS_COLOR "\033[1m\033[32m" /* Bold Green */
#define CRESET "\033[0m"
#define CHAR_TO_SEND_START 's'
#define CHAR_TO_SEND_STOP 'a'

const int dim_buffer = sizeof(float) * 2 + sizeof(unsigned long);
uint8_t b2write[1], readBytes[dim_buffer];

// void rtpublish(void *arg)
// {
//   xenopkg_interfaces::msg::Float32Header msg2;
//     while (true) {
//        //rt_mutex_acquire(&mutex, TM_INFINITE);

//         msg2=msg;

//        // rt_mutex_release(&mutex);
//         pub->publish(msg2);
//         rt_task_wait_period(NULL);
// }
// }

void myflush()
{

  while (my_serial->read(readBytes, dim_buffer) != 0)
    ;
}

void start_data_streaming()
{
  b2write[0] = CHAR_TO_SEND_START;
  my_serial->write(b2write, 1);
}

void stop_data_streaming()
{
  b2write[0] = CHAR_TO_SEND_STOP;
  my_serial->write(b2write, 1);
  my_serial->flush();
  myflush();
  my_serial->flush();
}

void serial_task_cb(void *arg)
{

  msg.data = {0.0, 0.0};

  RTIME current_time_ticks;

  float *pos;
  float *vel;
  // unsigned long* arduino_time;

  stop_data_streaming();
  start_data_streaming();

  // size_t bytescritti = my_serial->write(b2write, 1);
  // size_t bytescritti=my_serial->write(b2write,1);

  while (runserial)
  {
    my_serial->read(readBytes, dim_buffer);

    current_time_ticks = rt_timer_read();

    msg.header.stamp.sec = current_time_ticks / 1000000000;
    msg.header.stamp.nanosec = current_time_ticks % 1000000000;
    // finger_voltages.header.stamp = node->get_clock()->now();

    pos = (float *)(&readBytes[0]);
    vel = (float *)(&readBytes[sizeof(float)]);
    // arduino_time=(unsigned long*)(&readBytes[2*sizeof(float)]);
    msg.data[0] = *pos;
    msg.data[1] = *vel;

    // msg.header.stamp.sec = (int32_t)*arduino_time/1000000;
    // msg.header.stamp.nanosec = (uint32_t)*arduino_time % 1000000;

    pub->publish(msg);
    // my_serial->flush();
  }
  stop_data_streaming();
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

using namespace std;
void set_serial_low_latency(const string &serial_port)
{
  cout << "Setting low_latency for " << WARN_COLOR << serial_port << CRESET << endl;
  string command = "setserial " + serial_port + " low_latency";
  int result = system(command.c_str());
  cout << "Setting low_latency for " << WARN_COLOR << serial_port << CRESET << " result:" << WARN_COLOR << result << CRESET << endl;
}

void service_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{

  if (request->data)
  {
    if (runserial)
    {
      RCLCPP_INFO(node->get_logger(), " request cancel,serial already activate");
      response->success = true;
      return;
    }

    RCLCPP_INFO(node->get_logger(), " request accepted");
    runserial = true;
    printf("start serial task\n");
    rt_task_create(&serial_task, "serial_task", 0, 90, T_JOINABLE);

    rt_task_start(&serial_task, &serial_task_cb, 0);
    // rt_task_join(&serial_task);

    response->success = true;
    return;
  }
  if (!runserial)
  {
    RCLCPP_INFO(node->get_logger(), " serial task not present");
    response->success = true;
    return;
  }

  RCLCPP_INFO(node->get_logger(), " request accepted,cancel serial task");
  runserial = false;
  rt_task_join(&serial_task);
  rt_task_delete(&serial_task);
  response->success = true;
  return;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("serial_node");

  pub = node->create_publisher<xenopkg_interfaces::msg::Float32Header>("encoder_data", 1);
  auto serial_service = node->create_service<std_srvs::srv::SetBool>("set_serial_srv", &service_cb);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "nodo creato");
  node->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  node->declare_parameter<int>("baud_rate", 2000000);
  node->declare_parameter<int>("serial_timeout", 1000);
  /**** CHECK PARAMS ****/
  std::string serial_port;
  int baud_rate, serial_timeout;

  node->get_parameter("serial_port", serial_port);
  node->get_parameter("baud_rate", baud_rate);
  node->get_parameter("serial_timeout", serial_timeout);

  /**** INIT SERIAL ****/
  set_serial_low_latency(serial_port);
  my_serial = std::make_unique<serial::Serial>(serial_port, baud_rate, serial::Timeout::simpleTimeout(serial_timeout));

  /**** CHECK ****/
  if (!my_serial->isOpen())
  {
    cout << ERROR_COLOR << "ERROR - SERIAL PORT " << WARN_COLOR << serial_port << ERROR_COLOR << " is not open!" << CRESET << endl;
    exit(-1);
  }
  cout << SUCCESS_COLOR << "SERIAL PORT " << WARN_COLOR << serial_port << SUCCESS_COLOR << " OPEN - OK" << CRESET << endl;

  // /**** ROS MAIN LOOP  ****/
  // while(rclcpp::ok())
  // {
  //     my_serial.write(b2write,1);
  //     my_serial.read(readBytes, dim_buffer);
  //     finger_voltages.header.stamp = node->get_clock()->now();

  //     for (int i = 0; i < voltages_count; i++)
  //     {
  //         finger_voltages.tactile.data[i] = (double)(readBytes[i*2] + (readBytes[i*2+1]&0b00001111)*256) * 3.3/4096.0;
  //     }

  //     pubTactile->publish(finger_voltages);
  // }

  //   printf("start serial task\n");
  //  // rt_mutex_create(&mutex, "mutex");
  //   rt_task_create(&serial_task,"serial_task", 0, 90, 0);
  //   //rt_task_create(&rtpublish_task,"rtpublisher", 0, 90, 0);

  //   // rt_task_set_periodic(&serial_task,TM_NOW,1000000);
  //   // rt_task_set_periodic(&rtpublish_task,TM_NOW,1000000);

  //   rt_task_start(&serial_task, &serial_task_cb,0);
  rclcpp::spin(node);
  //   rt_task_join(&serial_task);
  // rt_task_start(&rtpublish_task, &rtpublish,0);

  // sensor_msgs::msg::Temperature msg;

  // while (rclcpp::ok())
  // {
  //     msg.data=n;
  //     pub->publish(msg);

  // }

  // rclcpp::spin(node);
  //  rt_task_delete(&serial_task);
  // rt_task_delete(&rtpublish_task);
  // rt_mutex_delete(&mutex);

  rclcpp::shutdown();
  my_serial->close();
  return 0;
}
