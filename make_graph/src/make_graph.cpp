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
 * Revised by juhyeonglee. for making data graph
 */

#include "make_graph/make_graph.h"

std::ofstream file;

Make::Make()
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint1_angle_ = 0; // variable for get current joint1 state from controller
  present_joint2_angle_ = 0;
  present_joint3_angle_ = 0;

  present_myo_x_ = 0;
  present_myo_y_ = 0;
  present_myo_z_ = 0;

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  ROS_INFO("record pkg start");
}

Make::~Make()
{
  ROS_INFO("Make pkg close");
  ros::shutdown();
}

// Subscribe joint_states from open_manipulator_controller
void Make::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &Make::jointStatesCallback, this);
  myo_ori_sub_ = node_handle_.subscribe("myo_raw/myo_ori", 10, &Make::myooriCallback, this);

}

void Make::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  present_joint1_angle_ = msg->position.at(0);
  present_joint2_angle_ = msg->position.at(1);
  present_joint3_angle_ = msg->position.at(2);
}

// myo-ori data callback function
void Make::myooriCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    //ROS_INFO("!!!Success Myo_ori!!!");
    present_myo_x_ = msg->x;
    present_myo_y_ = msg->y;
    present_myo_z_ = msg->z;
}

// save trajectory data from joint_state_variables to text_file
void Make::recording(char ch)
{
  if(ch == '1')
  {
    if(file.is_open()) {
        file << present_myo_x_ << " ";
        file << present_myo_y_ << " ";
        file << present_myo_z_ << " ";
        file << present_joint1_angle_ << " ";
        file << present_joint2_angle_ << " ";
        file << present_joint3_angle_ << " ";
        file << "\n";
    }
    else {
       printf("fail");
    }
  }

  else if(ch == '2')
  {
       file.close();
       printf("file closed\n");
  }
}

void Make::printText()
{
  printf("push '1' : start recording\n");
  printf("push '2' : stop and close file\n");
}

// chech if char is entered or not
// source : https://corsa.tistory.com/18 [CORSA]
bool Make::kbhit()
{
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "Make");
  Make make;
  make.printText();
  char ch;
  ch = std::getchar();

  // using ros_timer_publisher to make callbackfunction at custom hertz is also good idea
  // ros::Timer publish_timer = node_handle.createTimer(ros::Duration(open_manipulator_master.getServiceCallPeriod()), &OpenManipulatorMaster::publishCallback, &open_manipulator_master);

  file.open("/home/juhyeong/make_graph.txt");
  while (ros::ok())
  {
     ros::spinOnce();
     make.recording(ch);
     if(make.kbhit()){
         ch = std::getchar();
     }
  }
  file.close();
  return 0;
}
