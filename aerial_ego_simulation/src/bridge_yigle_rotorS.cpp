/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2022, NMMI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>

#include <tf/transform_broadcaster.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>

#include <std_srvs/Empty.h>

#define ETA ((double)(0.048 / 0.021))

using namespace std;

std::string ns;
ros::Publisher pub_joint0_left, pub_joint1_left, pub_joint2_left, pub_joint3_left, pub_joint4_left,
    pub_joint0_right, pub_joint1_right, pub_joint2_right, pub_joint3_right, pub_joint4_right,
    pub_joint1_neck, pub_joint2_neck;
bool hold_cmd = false, buf_button1 = false, buf_button_reset = false, reset_cmd = false;
void motorPosiCallback_rightArm(const sensor_msgs::JointState &msg)
{
  Eigen::VectorXd joint_state_cmd;
  joint_state_cmd.resize(5);
  joint_state_cmd(0) = msg.position[0];
  joint_state_cmd(4) = msg.position[5];

  joint_state_cmd(1) = msg.position[1] / ETA;
  joint_state_cmd(2) = msg.position[2] / ETA;
  joint_state_cmd(3) = msg.position[4] / ETA;

  if (hold_cmd)
  {
  }
  else
  {
    std_msgs::Float64 ref_msg;
    ref_msg.data = joint_state_cmd(0);
    pub_joint0_right.publish(ref_msg);
    ref_msg.data = joint_state_cmd(1);
    pub_joint1_right.publish(ref_msg);
    ref_msg.data = joint_state_cmd(2);
    pub_joint2_right.publish(ref_msg);
    ref_msg.data = joint_state_cmd(3);
    pub_joint3_right.publish(ref_msg);
    ref_msg.data = joint_state_cmd(4);
    pub_joint4_right.publish(ref_msg);
  }
}
void motorPosiCallback_leftArm(const sensor_msgs::JointState &msg)
{
  Eigen::VectorXd joint_state_cmd;
  joint_state_cmd.resize(5);
  joint_state_cmd(0) = msg.position[0];
  joint_state_cmd(4) = msg.position[5];

  joint_state_cmd(1) = msg.position[1] / ETA;
  joint_state_cmd(2) = msg.position[2] / ETA;
  joint_state_cmd(3) = msg.position[4] / ETA;

  if (hold_cmd)
  {
  }
  else
  {
    std_msgs::Float64 ref_msg;
    ref_msg.data = joint_state_cmd(0);
    pub_joint0_left.publish(ref_msg);
    ref_msg.data = joint_state_cmd(1);
    pub_joint1_left.publish(ref_msg);
    ref_msg.data = joint_state_cmd(2);
    pub_joint2_left.publish(ref_msg);
    ref_msg.data = joint_state_cmd(3);
    pub_joint3_left.publish(ref_msg);
    ref_msg.data = joint_state_cmd(4);
    pub_joint4_left.publish(ref_msg);
  }
}
void neckPosiCallback(const sensor_msgs::JointState &msg)
{
  Eigen::VectorXd joint_state_cmd;
  joint_state_cmd.resize(2);
  joint_state_cmd(0) = msg.position[0];
  joint_state_cmd(1) = msg.position[1];

  std_msgs::Float64 ref_msg;
  ref_msg.data = joint_state_cmd(0);
  pub_joint1_neck.publish(ref_msg);
  ref_msg.data = joint_state_cmd(1);
  pub_joint2_neck.publish(ref_msg);
}

void holdCmdCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data == true && buf_button1 == false)
  {
    hold_cmd = !hold_cmd;
  }
  buf_button1 = msg->data;
}

void resetCmdCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data == true && buf_button_reset == false)
  {
    reset_cmd = true;
  }
  buf_button_reset = msg->data;
}

int main(int argc, char **argv)
{

  ros::Subscriber subHandler_posi_motor_rightArm, subHandler_posi_motor_leftArm, subHandler_ref_neck, subHandler_hold_command;

  // ------------------------------------------------------------------------------------- Init node
  ros::init(argc, argv, "bridge_yigle_rotorS");
  ros::NodeHandle n;
  ns = ros::this_node::getNamespace();
  ns = ns.substr(1, ns.length() - 1);

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("gazebo/reset_world");

  subHandler_hold_command = n.subscribe("/Button_X", 1, resetCmdCallback);
  subHandler_hold_command = n.subscribe("/Button_A", 1, holdCmdCallback);
  subHandler_posi_motor_rightArm = n.subscribe("/right_arm/motor_position", 1, motorPosiCallback_rightArm);
  subHandler_posi_motor_leftArm = n.subscribe("/left_arm/motor_position", 1, motorPosiCallback_leftArm);
  //    subHandler_closure_hand_rightArm = nodeHandler.subscribe("/right_arm/hand_closure",1,&QbManager::handClosureCallback_rightArm,this);
  //    subHandler_closure_hand_leftArm = nodeHandler.subscribe("/left_arm/hand_closure",1,&QbManager::handClosureCallback_leftArm,this);
  subHandler_ref_neck = n.subscribe("/ref_neck", 1, neckPosiCallback);

  pub_joint0_left = n.advertise<std_msgs::Float64>("/yigle/joint0_position_controller_left/command", 1);
  pub_joint1_left = n.advertise<std_msgs::Float64>("/yigle/joint1_position_controller_left/command", 1);
  pub_joint2_left = n.advertise<std_msgs::Float64>("/yigle/joint2_position_controller_left/command", 1);
  pub_joint3_left = n.advertise<std_msgs::Float64>("/yigle/joint3_position_controller_left/command", 1);
  pub_joint4_left = n.advertise<std_msgs::Float64>("/yigle/joint4_position_controller_left/command", 1);

  pub_joint0_right = n.advertise<std_msgs::Float64>("/yigle/joint0_position_controller_right/command", 1);
  pub_joint1_right = n.advertise<std_msgs::Float64>("/yigle/joint1_position_controller_right/command", 1);
  pub_joint2_right = n.advertise<std_msgs::Float64>("/yigle/joint2_position_controller_right/command", 1);
  pub_joint3_right = n.advertise<std_msgs::Float64>("/yigle/joint3_position_controller_right/command", 1);
  pub_joint4_right = n.advertise<std_msgs::Float64>("/yigle/joint4_position_controller_right/command", 1);

  pub_joint1_neck = n.advertise<std_msgs::Float64>("/yigle/joint_position_controller_neck_1/command", 1);
  pub_joint2_neck = n.advertise<std_msgs::Float64>("/yigle/joint_position_controller_neck_2/command", 1);

  while (ros::ok())
  {
    if (reset_cmd == true)
    {
      std_srvs::Empty srv;
      client.call(srv);
      reset_cmd = false;
    }

    ros::spinOnce();
    /* code */
  }
  return 0;
}