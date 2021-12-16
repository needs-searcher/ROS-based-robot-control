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
 * Revised by juhyeonglee. for remoting Openmanipulator by using myo armband and recording a trajectory.
 * version3.
 * version4 : Add a save_term. To save long trajectory. if saving hz is too high, txt file cant contain long trajectory.
 * version5 : revise to match version with other pkg*/

#include "record_pkg/record_pkg.h"

std::ofstream file;

Record::Record()
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint1_angle_ = 0; // variable for get current joint1 state from controller
  present_joint2_angle_ = 0;
  present_joint3_angle_ = 0;
  present_joint4_angle_ = 0;
  present_gripper_angle_ = 0;
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  ROS_INFO("record pkg start");
}

Record::~Record()
{
  ROS_INFO("record pkg close");
  ros::shutdown();
}

// Subscribe joint_states from open_manipulator_controller
void Record::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &Record::jointStatesCallback, this);
}

void Record::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  present_joint1_angle_ = msg->position.at(0);
  present_joint2_angle_ = msg->position.at(1);
  present_joint3_angle_ = msg->position.at(2);
  //present_joint4_angle_ = msg->position.at(3);
  present_gripper_angle_ = msg->position.at(4);
}

// save trajectory data from joint_state_variables to text_file
void Record::recording(char ch)
{
  if(ch == '1')
  {
    if(file.is_open()) {
        file << present_joint1_angle_ << " ";
        file << present_joint2_angle_ << " ";
        file << present_joint3_angle_ << " ";
        //file << present_joint4_angle_ << " ";
        file << present_gripper_angle_ << " ";
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

void Record::printText()
{
  printf("push '1' : start recording\n");
  printf("push '2' : stop and close file\n");
}

// chech if char is entered or not
// source : https://corsa.tistory.com/18 [CORSA]
bool Record::kbhit()
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
  // To check while loop frequency
  clock_t start, end;
  double result;

  // Init ROS node
  ros::init(argc, argv, "record");
  Record record;
  record.printText();
  char ch;
  ch = std::getchar();

  // using ros_timer_publisher to make callbackfunction at custom hertz is also good idea
  // ros::Timer publish_timer = node_handle.createTimer(ros::Duration(open_manipulator_master.getServiceCallPeriod()), &OpenManipulatorMaster::publishCallback, &open_manipulator_master);

  file.open("/home/juhyeong/test.txt");

  start = clock();
  while (ros::ok())
  {
     ros::spinOnce();
     record.recording(ch);
     if(record.kbhit()){
         ch = std::getchar();
     }
  }
  end = clock();
  result = (double)(end-start);
  printf("%f", result);
  file.close();

  return 0;
}
