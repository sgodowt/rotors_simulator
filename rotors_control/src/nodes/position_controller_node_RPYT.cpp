/*
 * Modifications Copyright 2022 Fanyi Kong, NMMI, Italy

 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "position_controller_node_RPYT.h"

#include "rotors_control/parameters_ros.h"

#define _TUNE_PARAMETERS_ 1

namespace rotors_control
{

  PositionControllerRPYTNode::PositionControllerRPYTNode(
      const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
      : nh_(nh),
        private_nh_(private_nh)
  {
    InitializeParams();

    cmd_pose_sub_ = nh_.subscribe(
        mav_msgs::default_topics::COMMAND_POSE, 1,
        &PositionControllerRPYTNode::CommandPoseCallback, this);

    cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
        &PositionControllerRPYTNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                  &PositionControllerRPYTNode::OdometryCallback, this);

    attitude_thrust_reference_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
        kDefaultCommandAttitudeThrustTopic, 1);

    command_timer_ = nh_.createTimer(ros::Duration(0), &PositionControllerRPYTNode::TimedCommandCallback, this,
                                     true, false);
  }

  PositionControllerRPYTNode::~PositionControllerRPYTNode() {}

  void PositionControllerRPYTNode::InitializeParams()
  {

    // Read parameters from rosparam.
    GetRosParameter(private_nh_, "position_gain/x",
                    position_controller_.controller_parameters_.position_gain_.x(),
                    &position_controller_.controller_parameters_.position_gain_.x());
    GetRosParameter(private_nh_, "position_gain/y",
                    position_controller_.controller_parameters_.position_gain_.y(),
                    &position_controller_.controller_parameters_.position_gain_.y());
    GetRosParameter(private_nh_, "position_gain/z",
                    position_controller_.controller_parameters_.position_gain_.z(),
                    &position_controller_.controller_parameters_.position_gain_.z());
    GetRosParameter(private_nh_, "velocity_gain/x",
                    position_controller_.controller_parameters_.velocity_gain_.x(),
                    &position_controller_.controller_parameters_.velocity_gain_.x());
    GetRosParameter(private_nh_, "velocity_gain/y",
                    position_controller_.controller_parameters_.velocity_gain_.y(),
                    &position_controller_.controller_parameters_.velocity_gain_.y());
    GetRosParameter(private_nh_, "velocity_gain/z",
                    position_controller_.controller_parameters_.velocity_gain_.z(),
                    &position_controller_.controller_parameters_.velocity_gain_.z());
    GetVehicleParameters(private_nh_, &position_controller_.vehicle_parameters_);
    position_controller_.InitializeParameters();
  }
  void PositionControllerRPYTNode::Publish()
  {
  }

  void PositionControllerRPYTNode::CommandPoseCallback(
      const geometry_msgs::PoseStampedConstPtr &pose_msg)
  {
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
    commands_.push_front(eigen_reference);

    position_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();
  }

  void PositionControllerRPYTNode::MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
  {
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    const size_t n_commands = msg->points.size();

    if (n_commands < 1)
    {
      ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
      return;
    }

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
    commands_.push_front(eigen_reference);

    for (size_t i = 1; i < n_commands; ++i)
    {
      const trajectory_msgs::MultiDOFJointTrajectoryPoint &reference_before = msg->points[i - 1];
      const trajectory_msgs::MultiDOFJointTrajectoryPoint &current_reference = msg->points[i];

      mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

      commands_.push_back(eigen_reference);
      command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
    }

    // We can trigger the first command immediately.
    position_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();

    if (n_commands > 1)
    {
      command_timer_.setPeriod(command_waiting_times_.front());
      command_waiting_times_.pop_front();
      command_timer_.start();
    }
  }

  void PositionControllerRPYTNode::TimedCommandCallback(const ros::TimerEvent &e)
  {

    if (commands_.empty())
    {
      ROS_WARN("Commands empty, this should not happen here");
      return;
    }

    const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
    position_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();
    command_timer_.stop();
    if (!command_waiting_times_.empty())
    {
      command_timer_.setPeriod(command_waiting_times_.front());
      command_waiting_times_.pop_front();
      command_timer_.start();
    }
  }

  void PositionControllerRPYTNode::OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg)
  {

    ROS_INFO_ONCE("PositionControllerRPYT got first odometry message.");
    
    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    position_controller_.SetOdometry(odometry);

#if (_TUNE_PARAMETERS_)
    InitializeParams();
#endif
    mav_msgs::RollPitchYawrateThrust ref_atti_thrust;

    //ref_atti_thrust is in FLU coordinate!!
    position_controller_.CalculateAttiThrust(&ref_atti_thrust);
    ref_atti_thrust.header.stamp = odometry_msg->header.stamp;
    
    attitude_thrust_reference_pub_.publish(ref_atti_thrust);

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::PositionControllerRPYTNode position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
