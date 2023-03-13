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

#ifndef ROTORS_CONTROL_ATTITUDE_THRUST_CONTROLLER_NODE_H
#define ROTORS_CONTROL_ATTITUDE_THRUST_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/attitude_thrust_controller.h"

namespace rotors_control
{

  class AttitudeThrustControllerNode
  {
  public:
    AttitudeThrustControllerNode();
    ~AttitudeThrustControllerNode();

    void InitializeParams();

  private:
    AttitudeThrustController attitude_thrust_controller_;

    std::string namespace_;

    // subscribers
    ros::Subscriber cmd_attitude_thrust_sub_;
    ros::Subscriber odometry_sub_;
#if (_DEBUG_TORQUE_THRUST_)
    ros::Publisher torque_thrust_reference_pub_;
#endif
    ros::Publisher motor_velocity_reference_pub_;

    void AttitudeThrustCallback(
        const mav_msgs::RollPitchYawrateThrustConstPtr &attitude_thrust_reference_msg);

    void OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);
  };
}

#endif 
