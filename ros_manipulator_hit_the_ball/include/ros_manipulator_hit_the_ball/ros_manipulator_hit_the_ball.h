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

#ifndef ROS_MANIPULATOR_HIT_THE_BALL_H
#define ROS_MANIPULATOR_HIT_THE_BALL_H

#include <ros/ros.h>
#include <termios.h>
#include <sys/ioctl.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/KinematicsPose.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "ball_track_alvar_msgs/AlvarBalls.h"
#include "sensor_msgs/JointState.h"

#define NUM_OF_JOINT_AND_TOOL 5
#define HOME_POSE   1
#define HIT_START  2
#define STOP   3

typedef struct _BallMarker
{
  uint32_t id;
  double position[3];
} BallMarker;

class ROSManipulatorHittheBall
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient goal_task_space_path_client_;

  ros::Subscriber ros_manipulator_states_sub_;
  ros::Subscriber ros_manipulator_joint_states_sub_;
  ros::Subscriber ros_manipulator_kinematics_pose_sub_;
  ros::Subscriber ball_pose_sub_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  std::vector<std::string> joint_name_;
  bool ros_manipulator_is_moving_;
  std::vector<BallMarker> ball_pose;

  uint8_t mode_state_;
  uint8_t demo_count_;
  uint8_t hit_ball_id_;

 public:
  ROSManipulatorHittheBall();
  ~ROSManipulatorHittheBall();

  void initServiceClient();
  void initSubscribe();

  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void ballPoseCallback(const ball_track_alvar_msgs::AlvarBalls::ConstPtr &msg);

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setToolControl(std::vector<double> joint_angle);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, std::vector<double> kienmatics_orientation, double path_time);

  void publishCallback(const ros::TimerEvent&);
  void setModeState(char ch);
  void demoSequence();

  void printText();
  bool kbhit();
};

#endif //ROS_MANIPULATOR_HIT_THE_BALL_H
