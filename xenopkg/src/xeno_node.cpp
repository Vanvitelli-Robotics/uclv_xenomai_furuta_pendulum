#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include "Master.h"
// #include "Meca500.h"
#include "Controller.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xenopkg_interfaces/srv/joint_srv.hpp>
#include <xenopkg_interfaces/srv/set_mode.hpp>
#include <xenopkg_interfaces/srv/activate_vel.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <xenopkg_interfaces/msg/float32_header.hpp>

RT_TASK ethercat_task;
RT_TASK inizialize_task;
RT_TASK publisher_task;
RT_TASK service_task;
RT_TASK velocity_cmd_task;
RT_TASK spin_task;

std::shared_ptr<rclcpp::Node> node;
_Float64 n = 0;
uint16 k = 1;            // moveid
bool switch_control = 1; // variabile per proteggere la modalità di comando,defaul=1 perche accetta comandi in posizione,0 quando è in modalità velocità
bool disactiveVelCmd = 0;
bool checkActivate = 0;
bool vel_cmd_arrived_ = 0;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
rclcpp::Service<xenopkg_interfaces::srv::JointSrv>::SharedPtr joint_service;
sensor_msgs::msg::JointState msg;
xenopkg_interfaces::msg::Float32Header cmd_vel;
float service_joint[6] = {-89.5, 34.3, -13.6, -3.6, 59.14, 160.7};
float joint_velocities_cmd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<std::string> joint_names = {"meca_axis_1_joint",
                                        "meca_axis_2_joint",
                                        "meca_axis_3_joint",
                                        "meca_axis_4_joint",
                                        "meca_axis_5_joint",
                                        "meca_axis_6_joint"};
std::vector<double> joint_angles_vector = {0.0,
                                           0.0,
                                           0.0,
                                           0.0,
                                           0.0,
                                           0.0};
std::vector<double> joint_velocity_vector = {0.0,
                                             0.0,
                                             0.0,
                                             0.0,
                                             0.0,
                                             0.0};

// using namespace sun;
typedef int (sun::Meca500::*meca_fun_ptr)(uint16);
// using namespace std;
sun::Master *master;
sun::Meca500 *meca500;

void inizialize_cb(void *arg)
{

    bool as, hs, sm, es, pm, eob, eom;
    // float joint_angles[6];
    // float joint_velocities[6];
    // float joints[6] = {-89.5, 34.3, -13.6, -3.6, 59.14, 160.7};
    // float omega[6] = {0.0, 0.0, 0.0, 0.0, 0.0, -10.0};
    int activateRob, homeRob;

    // //parameters for control
    // float theta_0, theta_f = 0;

    // float tf = 10;
    // float Tc = 0.01; //sample time

    // int dim = tf / Tc;
    // float time_array[dim];
    // float theta_d[dim];
    // float b[6] = {6, -15, 10, 0, 0, 0};

    // time_array[0] = 0;
    // float tau = 0;

    // sleep(5);
    rt_task_sleep(5000000000);

    // rt_mutex_acquire(&mut, TM_INFINITE);
    meca500->getStatusRobot(as, hs, sm, es, pm, eob, eom);
    // rt_mutex_release(&mut);
    rt_printf("\nActivate: %d\n", as);
    rt_printf("Homed: %d\n", hs);
    rt_printf("Sim: %d\n", sm);
    rt_printf("Error: %d\n", es);

    // sleep(2);
    rt_task_sleep(2000000000);
    // rt_mutex_acquire(&mut, TM_INFINITE);
    meca500->resetError();
    // rt_mutex_release(&mut);
    rt_task_sleep(4000000000);
    // rt_mutex_acquire(&mut, TM_INFINITE);
    activateRob = meca500->activateRobot();
    // rt_mutex_release(&mut);
    rt_task_sleep(4000000000);

    if (activateRob == 0 || activateRob == 1)
    {
        if (activateRob == 1)
            std::cout << "Motors already activated.\n";
        else
            std::cout << "Motors activated.\n";

        rt_task_sleep(2000000000);
        // rt_mutex_acquire(&mut, TM_INFINITE);
        homeRob = meca500->home();
        // rt_mutex_release(&mut);
        rt_task_sleep(5000000000);
        // rt_mutex_acquire(&mut, TM_INFINITE);
        // checkhomeRob = meca500->checkhome();
        // rt_mutex_release(&mut);
        // rt_task_sleep(1000000000);

        if (homeRob == 0 || homeRob == 1)
        {
            if (homeRob == 1)
                std::cout << "Home already done.\n";
            else
                std::cout << "Home done.\n";
            // sleep(2);
            rt_task_sleep(2000000000);
            // rt_mutex_acquire(&mut, TM_INFINITE);
            //     meca500->getJoints(joint_angles);
            //    // rt_mutex_release(&mut);
            //     for (int i = 0; i < 6; i++)
            //     {
            //         std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
            //     }
            //     rt_printf("\n\n");
            // prova velocita

            // rt_task_sleep(2000000000);
            //  if (meca500->setPoint(1) == 0)
            // {
            //  rt_task_sleep(2000000000);
            //  meca500->moveJointsVel(omega,0);
            //  rt_task_sleep(20000000000);
            //  meca500->setPoint(0);
            // }

            // rt_task_set_periodic(NULL,TM_NOW,1000000);
            //   while (rclcpp::ok())
            //   {
            //    // rt_mutex_acquire(&mut, TM_INFINITE);
            //     meca500->getJoints(joint_angles);
            //     //rt_mutex_release(&mut);
            //     for (size_t i = 0; i < 6; i++)
            //     {
            //          joint_angles_vector[i]=joint_angles[i];
            //     }

            //     meca500->getJointsVelocities(joint_velocities);
            //     //rt_mutex_release(&mut);
            //     for (size_t i = 0; i < 6; i++)
            //     {
            //          joint_velocity_vector[i]=joint_velocities[i];
            //     }

            //     msg.name=joint_names;
            //     msg.position=joint_angles_vector;
            //     msg.velocity=joint_velocity_vector;
            //     joint_pub->publish(msg);
            //     //rt_printf("pub:");
            //     rt_task_wait_period(NULL);
            //   }

            // rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);

            // theta_0 = joint_angles[5];

            // for (int i = 1; i < dim; i++)
            // {
            //     time_array[i] = time_array[i - 1] + Tc;
            //     tau = time_array[i] / tf;
            //     theta_d[i] = theta_0 + (theta_f - theta_0) * (b[0] * pow(tau, 5) + b[1] * pow(tau, 4) + b[2] * pow(tau, 3));
            //     //std::cout << theta_d[i]<<"\n";
            //     //printf("%f\n", theta_d[i]);
            // }
            // sleep(2);

            //     if (meca500.setPoint(1) == 0)
            //     {

            //         meca500.moveJoints(joints);
            //         sleep(5);

            //         controller.startThread();

            //         controller.waitLoop(); //the user thread waits the end of thread controller.
            //         meca500.setPoint(0);

            //         sleep(2);
            //         meca500.getJoints(joint_angles);

            //         for (int i = 0; i < 6; i++)
            //         {
            //             std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
            //         }
            //         printf("\n\n");
            //     }

            //     else
            //         std::cout << "Invalid input!\n";
        }

        else
        {
            if (homeRob == -1)
                std::cout << "Motors must be activated to do home";
            else
                std::cout << "ERROR_Homing.\n";
        }
        // rclcpp::spin(node);
        rt_task_sleep(2000000000);
        // rt_mutex_acquire(&mut, TM_INFINITE);
        // deactivateRob = meca500->deactivateRobot();
        //  rt_mutex_release(&mut);
        //  rt_task_sleep(2000000000);
        //  checkdeactivateRob = meca500->checkdeactivateRobot();

        // if (deactivateRob == 0)
        //     std::cout << "Motors deactivated.\n";
    }
    else
    {
        std::cout << "ERROR_Activate.\n";
        rt_task_sleep(4000000000);
    }

    rt_task_sleep(2000000000);
}

void publisher_cb(void *arg)
{

    float joint_angles[6];
    float joint_velocities[6];
    int deactivateRob;
    msg.name = joint_names;
    RTIME current_time_ticks;
    rt_task_set_periodic(NULL, TM_NOW, 1000000);
    while (rclcpp::ok())
    {

        meca500->getJoints(joint_angles);
        meca500->getJointsVelocities(joint_velocities);
        for (size_t i = 0; i < 6; i++)
        {
            joint_angles_vector[i] = joint_angles[i] * M_PI / 180.0;
            joint_velocity_vector[i] = joint_velocities[i] * M_PI / 180.0;
        }

        current_time_ticks = rt_timer_read();
        msg.header.stamp.sec = current_time_ticks / 1000000000;
        msg.header.stamp.nanosec = current_time_ticks % 1000000000;
        msg.position = joint_angles_vector;
        msg.velocity = joint_velocity_vector;
        joint_pub->publish(msg);

        rt_task_wait_period(NULL);
    }

    rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);

    deactivateRob = meca500->deactivateRobot();
    if (deactivateRob == 0)
        rt_printf("Motors deactivated.\n");
}

void ethercat_cb(void *arg)
{
    long int time1;
    long int time2;
    long int cycle;
    volatile int wkc;
    long int toff = 0;
    // sensor_msgs::msg::Temperature msg;
    // RTIME current_time_ticks;
    // RTIME old_time_ticks=0;
    // double diff=0;
    master->cycletime = 1000000;
    master->shutdown = true;
    master->thread = true;
    master->createMutex(); // aggiungo la creazione del mutex nella libreria
    while (true)
    {

        time1 = ec_DCtime;
        // this->add_timespec(&ts, cycletime + toff);
        // clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        // mtx.lock();
        // rt_mutex_acquire(&mut, TM_INFINITE);
        master->mutex_down();
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        master->mutex_up();
        // rt_mutex_release(&mut);
        // mtx.unlock();

        master->ec_sync(ec_DCtime, 1000000, &toff);
        time2 = ec_DCtime;
        cycle = time2 - time1;

        // current_time_ticks = rt_timer_read();

        // msg.header.stamp.sec = current_time_ticks / 1000000000;
        // msg.header.stamp.nanosec = current_time_ticks % 1000000000;
        // //msg.header.stamp = node->now();
        // msg.temperature=n;

        // printf(" n=%f\n",n);
        // n++;
        // diff=current_time_ticks-old_time_ticks;
        // rt_printf("diff : %f \n", diff);
        // pub->publish(msg);
        // old_time_ticks=current_time_ticks;

        rt_task_wait_period(NULL);
    }
}

// void service_task_cb(void *arg)
// {
//    bool as, hs, sm, es, pm, eob=0, eom;

//     if (meca500->setPoint(1) == 0)
//         {
//         RCLCPP_INFO(node->get_logger()," moving...");
//         meca500->moveJoints(service_joint,k);
//         k++;
//         k=k%65500;
//         rt_task_sleep(10000000);
//         while (!eob)
//         {
//             rt_task_sleep(500000);
//             meca500->getStatusRobot(as, hs, sm, es, pm, eob, eom);
//         }
//         RCLCPP_INFO(node->get_logger()," Movement completed");
//         meca500->setPoint(0);
//         //rt_task_sleep(1000000000);
//         }

// }

void service_cb(const std::shared_ptr<xenopkg_interfaces::srv::JointSrv::Request> request, std::shared_ptr<xenopkg_interfaces::srv::JointSrv::Response> response)
{

    if (!switch_control)
    {
        response->completed = false;
        RCLCPP_INFO(node->get_logger(), " request cancel,velocity mode");
        return;
    }
    RCLCPP_INFO(node->get_logger(), " request accepted");
    if (request->qf.size() == 6)
    {
        for (size_t i = 0; i < 6; i++)
        {
            service_joint[i] = (request->qf[i]) * 180.0 / M_PI;
        }
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), " incorrect data");
        response->completed = false;
        return;
    }

    bool as, hs, sm, es, pm, eob = 0, eom;

    if (meca500->setPoint(1) == 0)
    {
        RCLCPP_INFO(node->get_logger(), " moving...");
        meca500->moveJoints(service_joint, k);
        k++;
        k = k % 65500;
        rt_task_sleep(30000000);
        while (!eob)
        {
            rt_task_sleep(500000);
            meca500->getStatusRobot(as, hs, sm, es, pm, eob, eom);
        }
        RCLCPP_INFO(node->get_logger(), " Movement completed");
        rt_task_sleep(20000000);
        meca500->setPoint(0);
        // rt_task_sleep(1000000000);
    }

    //  rt_task_create(&service_task,"service_thread", 0, 90,T_JOINABLE);
    //  rt_task_start(&service_task, &service_task_cb,0);

    //  rt_task_join(&service_task);
    //  rt_task_delete(&service_task);
    response->completed = true;
}

void setmode_cb(const std::shared_ptr<xenopkg_interfaces::srv::SetMode::Request> request, std::shared_ptr<xenopkg_interfaces::srv::SetMode::Response> response)
{

    if (request->mode == 1)
    {
        RCLCPP_INFO(node->get_logger(), " request accepted");
        switch_control = 1;
        meca500->setPoint(0);
        RCLCPP_INFO(node->get_logger(), " position mode activate,now you can call the position service");
        response->success = true;
        return;
    }
    else
    {
        if (request->mode == 2)
        {
            RCLCPP_INFO(node->get_logger(), " request accepted");
            switch_control = 0;
            if (meca500->setJoinVel(100) == 0)
            {
                RCLCPP_INFO(node->get_logger(), " joint vel 100");
            }

            RCLCPP_INFO(node->get_logger(), " velocity mode activate,now you can call the velocity service");
            response->success = true;
            return;
        }

        response->success = false;
        RCLCPP_INFO(node->get_logger(), " request cancel,error to insert mode command: set mode=1 for position mode or mode=2 for velocity mode");
        return;
    }
}

// void velocity_cmd_task_cb(void *arg)
// {

// //float joint_velocities_cmd[6];

// bool bool_vel=0;
// int n=0;
// RTIME current_time_ticks;
// float diffnanosec,diffsec,diffmax=0.0;
// float time_current;
// int time_current_sec;
// uint time_current_nanosec;
// rt_task_set_periodic(NULL,TM_NOW,100000);

//      while (!disactiveVelCmd)
//     {

//       // bool_vel = rclcpp::wait_for_message<std_msgs::msg::Float32MultiArray>(cmd_vel, node,"/cmd_vel", std::chrono::milliseconds(1));

//         // if (bool_vel)
//         // {
//         //     RCLCPP_INFO(node->get_logger()," message arrived");
//         //     for (size_t i = 0; i < 6; i++)
//         //     {
//         //          joint_velocities_cmd[i]=cmd_vel.data[i];
//         //     }

//         // }else{
//         //     RCLCPP_INFO(node->get_logger()," message not arrived, timeout ");
//         //    for (size_t j = 0; j < 6; j++)
//         //     {
//         //          joint_velocities_cmd[j]=0.0;
//         //     }

//         // }

//         //     if (n%2==0)
//         //     {

//         //    joint_velocities_cmd[5]=10.0;

//         //     }else{
//         //     joint_velocities_cmd[5]=9.0;
//         //     }
//            // n++;

//             if(vel_cmd_arrived_){
//             current_time_ticks = rt_timer_read();
//             time_current_sec=current_time_ticks / 1000000000;
//             time_current_nanosec=current_time_ticks % 1000000000;
//             // diffsec=time_current_sec-cmd_vel.header.stamp.sec;
//             diffnanosec=time_current_nanosec-cmd_vel.header.stamp.nanosec;
//             rt_printf("diffnanosec: %f \n", diffnanosec);
//             joint_velocities_cmd[0]=cmd_vel.data[0];
//             meca500->setPoint(1);
//             meca500->moveJointsVel(joint_velocities_cmd,0);
//             //meca500->setPoint(0);
//             vel_cmd_arrived_ = false;

//             }

//         rt_task_wait_period(NULL);
//         }
//         rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);

//         meca500->setPoint(0);

// }

// void activate_vel_cb(const std::shared_ptr<xenopkg_interfaces::srv::ActivateVel::Request> request,std::shared_ptr<xenopkg_interfaces::srv::ActivateVel::Response> response){

// if(switch_control){
// response->success=false;
// RCLCPP_INFO(node->get_logger()," request cancel,position mode");
// return;

// }
//  RCLCPP_INFO(node->get_logger()," request accepted");
//  if (request->data==true) {
//  RCLCPP_INFO(node->get_logger()," activate velocity topic command...");
//  if (checkActivate==0)
//  {
//  rt_task_create(&velocity_cmd_task,"velocity_command_thread", 0, 90,T_JOINABLE);
//  rt_task_start(&velocity_cmd_task, &velocity_cmd_task_cb,0);
//  response->success=true;
//  RCLCPP_INFO(node->get_logger()," activation successful");
//  checkActivate=1;
//  return;
//  }else{
//  RCLCPP_INFO(node->get_logger()," velocity topic already activate");
//  return;
//  }

// } else {

//     if (checkActivate==1)
//     {
//  RCLCPP_INFO(node->get_logger()," disactivate velocity topic command...");
//  disactiveVelCmd=1;
//  rt_task_join(&velocity_cmd_task);
//  rt_task_delete(&velocity_cmd_task);
//  response->success=true;
//  RCLCPP_INFO(node->get_logger()," disactivation successful");
//  checkActivate=0;
//  return;
//     }else{
//   RCLCPP_INFO(node->get_logger()," velocity topic already disactivate");
//   return;
//     }

// }

// }

void topic_callback(const xenopkg_interfaces::msg::Float32Header::SharedPtr message)
{
    // RCLCPP_INFO(node->get_logger()," message arrived");
    // for (size_t i = 0; i < 6; i++)
    // {
    //      joint_velocities_cmd[i]=message->data[i];
    // }
    if (switch_control)
    {
        RCLCPP_INFO(node->get_logger(), " request cancel,position mode");
        return;
    }
    // RTIME current_time_ticks;
    // float diffnanosec,messnano;
    // float time_current_nanosec,nanomess;
    // auto mess=message.get();
    // current_time_ticks = rt_timer_read();
    // time_current_nanosec=current_time_ticks % 1000000000;
    // diffsec=time_current_sec-cmd_vel.header.stamp.sec;

    // messnano=mess->header.stamp.nanosec;
    // diffnanosec=time_current_nanosec-messnano;
    // rt_printf("diffnanosec: %f \n", diffnanosec);
    joint_velocities_cmd[0] = (message->data[0]) * 180.0 / M_PI;
    joint_velocities_cmd[1] = (message->data[1]) * 180.0 / M_PI;
    joint_velocities_cmd[2] = (message->data[2]) * 180.0 / M_PI;
    joint_velocities_cmd[3] = (message->data[3]) * 180.0 / M_PI;
    joint_velocities_cmd[4] = (message->data[4]) * 180.0 / M_PI;
    joint_velocities_cmd[5] = (message->data[5]) * 180.0 / M_PI;
    meca500->setPoint(1);
    meca500->moveJointsVel(joint_velocities_cmd, 0);

    // cmd_vel.header=message->header;
    // cmd_vel.data=message->data;

    // vel_cmd_arrived_ = true;
}

void spin_cb(void *arg)
{
    // rt_task_set_periodic(NULL,TM_NOW,100000);
    //      while (rclcpp::ok())
    //      {
    //         rclcpp::spin_some(node);
    //         rt_task_wait_period(NULL);
    //      }
    //        rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);
    rclcpp::spin(node);
}

// using namespace sun;
// typedef int (Meca500::*meca_fun_ptr)(uint16);
// using namespace std;
//  Master *master;
//  Meca500 *meca500;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("driver_node");

    joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    joint_service = node->create_service<xenopkg_interfaces::srv::JointSrv>("joint_position_srv", &service_cb);
    auto mode_service = node->create_service<xenopkg_interfaces::srv::SetMode>("set_mode_srv", &setmode_cb);
    // auto activate_vel_service=node->create_service<xenopkg_interfaces::srv::ActivateVel>("activate_vel_srv",&activate_vel_cb);
    auto subscription = node->create_subscription<xenopkg_interfaces::msg::Float32Header>("/cmd_vel", 1, topic_callback);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "nodo creato");

    std::cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    char *ifname = "enp2s0";
    uint16 state_check;

    try
    {
        long time_new_packet[500];
        master = new sun::Master(ifname, FALSE, EC_TIMEOUT_TO_SAFE_OP);
        meca500 = new sun::Meca500(1, master);
        // Meca500 meca500(1, master);
        //  Controller controller(&meca500, 0.5);

        master->setupSlave(meca500->getPosition(), sun::Meca500::setup_static);

        master->configDC();
        master->configMap();

        meca500->assign_pointer_struct();

        try
        {
            master->movetoState(meca500->getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
            printf("start ethercat thread\n");

            rt_task_create(&ethercat_task, "ethercat_thread", 0, 99, T_JOINABLE);

            rt_task_set_periodic(&ethercat_task, TM_NOW, 1000000);

            rt_task_start(&ethercat_task, &ethercat_cb, 0);

            // master.createThread(1000000);

            try
            {
                master->movetoState(meca500->getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);

                try
                {

                    // MECA500_CODE
                    printf("start inizialize thread\n");

                    rt_task_create(&inizialize_task, "inizialize_thread", 0, 90, T_JOINABLE);

                    rt_task_start(&inizialize_task, &inizialize_cb, 0);

                    rt_task_join(&inizialize_task);
                    rt_task_delete(&inizialize_task);
                    printf("inizialization terminated\n");
                    printf("start publisher thread\n");

                    rt_task_create(&publisher_task, "publisher_thread", 0, 90, T_JOINABLE);

                    rt_task_start(&publisher_task, &publisher_cb, 0);

                    rt_task_create(&spin_task, "spin_thread", 0, 90, T_JOINABLE);

                    rt_task_start(&spin_task, &spin_cb, 0);
                    rt_task_join(&spin_task);

                    //   rclcpp::executors::MultiThreadedExecutor executor;
                    //   executor.add_node(node);
                    //   executor.spin();
                    // rclcpp::spin(node);

                    rt_task_join(&publisher_task);
                    // rt_task_join(&ethercat_task);
                    rt_task_delete(&publisher_task);
                    rt_task_delete(&ethercat_task);

                    // rt_task_delete(&service_task);
                    //  rt_mutex_delete(&mut);
                    master->deleteMutex();
                    // bool as, hs, sm, es, pm, eob, eom;
                    // float joint_angles[6];
                    // float joints[6] = {0, 0, 0, 0, 90, 0};
                    // float omega[6] = {0, 0, 0, 0, 0, -30};
                    // int activateRob, deactivateRob, homeRob;

                    // // //parameters for control
                    // // float theta_0, theta_f = 0;

                    // // float tf = 10;
                    // // float Tc = 0.01; //sample time

                    // // int dim = tf / Tc;
                    // // float time_array[dim];
                    // // float theta_d[dim];
                    // // float b[6] = {6, -15, 10, 0, 0, 0};

                    // // time_array[0] = 0;
                    // // float tau = 0;

                    // sleep(5);

                    // meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                    // printf("\nActivate: %d\n", as);
                    // printf("Homed: %d\n", hs);
                    // printf("Sim: %d\n", sm);
                    // printf("Error: %d\n", es);

                    // sleep(2);
                    // meca500.resetError();
                    // sleep(2);

                    // activateRob = meca500.activateRobot();
                    // if (activateRob == 0 || activateRob == 1)
                    // {
                    //     if (activateRob == 1)
                    //         std::cout << "Motors already activated.\n";
                    //     else
                    //         std::cout << "Motors activated.\n";

                    //     sleep(2);

                    //     homeRob = meca500.home();
                    //     if (homeRob == 0 || homeRob == 1)
                    //     {
                    //         if (activateRob == 1)
                    //             std::cout << "Home already done.\n";
                    //         else
                    //             std::cout << "Home done.\n";
                    //         sleep(2);

                    //         meca500.getJoints(joint_angles);

                    //         for (int i = 0; i < 6; i++)
                    //         {
                    //             std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                    //         }
                    //         printf("\n\n");
                    //           rclcpp::Rate loop_rate(10);
                    //           while (rclcpp::ok())
                    //           {
                    //             meca500.getJoints(joint_angles);
                    //             for (size_t i = 0; i < 6; i++)
                    //             {
                    //                  joint_angles_vector[i]=joint_angles[i];
                    //             }

                    //             printf("ciclo");
                    //             msg.name=joint_names;
                    //             msg.position=joint_angles_vector;
                    //             joint_pub->publish(msg);
                    //             loop_rate.sleep();
                    //           }

                    //         // theta_0 = joint_angles[5];

                    //         // for (int i = 1; i < dim; i++)
                    //         // {
                    //         //     time_array[i] = time_array[i - 1] + Tc;
                    //         //     tau = time_array[i] / tf;
                    //         //     theta_d[i] = theta_0 + (theta_f - theta_0) * (b[0] * pow(tau, 5) + b[1] * pow(tau, 4) + b[2] * pow(tau, 3));
                    //         //     //std::cout << theta_d[i]<<"\n";
                    //         //     //printf("%f\n", theta_d[i]);
                    //         // }
                    //         sleep(2);

                    //     //     if (meca500.setPoint(1) == 0)
                    //     //     {

                    //     //         meca500.moveJoints(joints);
                    //     //         sleep(5);

                    //     //         controller.startThread();

                    //     //         controller.waitLoop(); //the user thread waits the end of thread controller.
                    //     //         meca500.setPoint(0);

                    //     //         sleep(2);
                    //     //         meca500.getJoints(joint_angles);

                    //     //         for (int i = 0; i < 6; i++)
                    //     //         {
                    //     //             std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                    //     //         }
                    //     //         printf("\n\n");
                    //     //     }

                    //     //     else
                    //     //         std::cout << "Invalid input!\n";

                    //     }

                    //     else
                    //     {
                    //         if (homeRob == -1)
                    //             std::cout << "Motors must be activated to do home";
                    //         else
                    //             std::cout << "ERROR_Homing.\n";
                    //     }
                    //     //rclcpp::spin(node);
                    //     sleep(2);

                    //     deactivateRob = meca500.deactivateRobot();
                    //     if (deactivateRob == 0)
                    //         std::cout << "Motors deactivated.\n";
                    // }
                    // else
                    // {
                    //     std::cout << "ERROR_Activate.\n";
                    //     sleep(8);

                    //     rclcpp::shutdown();
                    //     rt_task_delete(&ethercat_task);
                    // }

                    // sleep(2);

                    // // meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                    // // printf("\nActivate: %d\n", as);
                    // // printf("Homed: %d\n", hs);
                    // // printf("Sim: %d\n", sm);
                    // // printf("Error: %d\n", es);
                }
                catch (const std::runtime_error &e)
                {
                    std::cerr << e.what();
                }

                osal_usleep(5000000);
                try
                {
                    master->close_master();
                    // master.stampa();
                }
                catch (const std::runtime_error &e)
                {
                    std::cout << "Error close_master\n";
                }
                // master.waitThread();
            }
            catch (const std::runtime_error &e)
            {
                std::cout << "Error state_transition SAFE_OP-> OP\n";
            }
        }
        catch (const std::runtime_error &e)
        {
            std::cout << "Error state_transition PRE_OP-> SAFE_OP\n";
        }
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << e.what();
        return -1;
    }

    rclcpp::shutdown();
    delete meca500;
    delete master;
    return 0;
}
