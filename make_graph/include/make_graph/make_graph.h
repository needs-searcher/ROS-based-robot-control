/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */
/* Revised by suho : https://suho0515.blogspot.com/2019/11/pick-and-place-using-inceptionv3-in-ros.html
 * Revised by juhyeonglee. for make data graph */

#ifndef RECORD_H_
#define RECORD_H_
#include <termios.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <geometry_msgs/Vector3.h>


class Make
{
 public:
  Make();
  ~Make();

  void recording(char ch);
  bool kbhit();
  void printText();


 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;


  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  //std::vector<double> present_joint_angle_;
  double present_joint1_angle_;
  double present_joint2_angle_;
  double present_joint3_angle_;
  double present_myo_x_;
  double present_myo_y_;
  double present_myo_z_;


  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();


  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber myo_ori_sub_;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void myooriCallback(const geometry_msgs::Vector3::ConstPtr &msg);

  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/

  /*****************************************************************************
  ** Others
  *****************************************************************************/


};
#endif RECORD_H_


