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

#include <thread>
#include <chrono>

#include <cmath>
#include <Eigen/Core>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/PosYaw.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <dynamic_reconfigure/server.h>
#include <rotors_gazebo/pos_tuneConfig.h>

double x = 0, y = 0, z = 2, yaw = 0;
double vx = 0, vy = 0, vz = 0, yawrate = 0;

void zCallback(const std_msgs::Float64::ConstPtr &msg)
{
  vz = msg->data;
};
void yawrateCallback(const std_msgs::Float64::ConstPtr &msg)
{
  yawrate = -msg->data;
};
void pitchCallback(const std_msgs::Float64::ConstPtr &msg)
{
  vx = msg->data;
};
void rollCallback(const std_msgs::Float64::ConstPtr &msg)
{
  vy = -msg->data;
};

void referencePoseCallback(const mav_msgs::PosYawConstPtr &msg)
{
  x=msg->position.x;
  y=msg->position.y;
  z=msg->position.z;
  yaw= msg->yaw;
};

void position_tune_callback(rotors_gazebo::pos_tuneConfig &config, uint32_t level) {

  x=config.pos_x;
  y=config.pos_y;
  z=config.pos_z;
  yaw=config.pos_yaw;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  dynamic_reconfigure::Server<rotors_gazebo::pos_tuneConfig> server;
  dynamic_reconfigure::Server<rotors_gazebo::pos_tuneConfig>::CallbackType f;

  f = boost::bind(&position_tune_callback, _1, _2);
  server.setCallback(f);

  ros::Publisher pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>(
          mav_msgs::default_topics::COMMAND_POSE, 10);

  ros::Subscriber z_sub = nh.subscribe("/LH_joy_y", 1, &zCallback);
  ros::Subscriber yawrate_sub = nh.subscribe("/LH_joy_x", 1, &yawrateCallback);
  ros::Subscriber pitch_sub = nh.subscribe("/RH_joy_y", 1, &pitchCallback);
  ros::Subscriber roll_sub = nh.subscribe("/RH_joy_x", 1, &rollCallback);

  ros::Subscriber reference_pose_sub = nh.subscribe("/reference_pose", 1, &referencePoseCallback);


  ros::Rate loop_rate(50);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused)
  {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused)
  {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else
  {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  // ros::Duration(5.0).sleep();

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  x=0;
  y=0;
  z=2;
  yaw=0;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", x, x);
  nh_private.param("y", y, y);
  nh_private.param("z", z, z);
  nh_private.param("yaw", yaw, yaw);

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;  
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), x,
           y, z);
  pose_pub.publish(pose_msg);

  while (ros::ok())
  {

    double vx_n = 0, vy_n = 0;

    vx_n = vx * sin(yaw) + vy * cos(yaw);
    vy_n = vy * sin(yaw) - vx * cos(yaw);
    x += vx_n * 0.1 * 0.02;
    y += vy_n * 0.1 * 0.02;

    z += vz * 0.1 * 0.02;
    yaw += yawrate * 0.1 * 0.02;

    pose_msg.header.stamp = ros::Time::now();

  q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;  
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();

    pose_pub.publish(pose_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
