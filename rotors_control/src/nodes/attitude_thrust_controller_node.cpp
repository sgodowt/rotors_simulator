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

#include "attitude_thrust_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control
{

  AttitudeThrustControllerNode::AttitudeThrustControllerNode()
  {
    InitializeParams();

    ros::NodeHandle nh;

    cmd_attitude_thrust_sub_ = nh.subscribe(kDefaultCommandAttitudeThrustTopic, 2,
                                                      &AttitudeThrustControllerNode::AttitudeThrustCallback, this);
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 10,
                                 &AttitudeThrustControllerNode::OdometryCallback, this);
#if (_DEBUG_TORQUE_THRUST_)
    torque_thrust_reference_pub_ = nh.advertise<mav_msgs::TorqueThrust>(
        kDefaultCommandTorqueThrustTopic, 2);
#endif

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
        kDefaultCommandMotorSpeedTopic, 1);
  }

  AttitudeThrustControllerNode::~AttitudeThrustControllerNode() {}

  void AttitudeThrustControllerNode::InitializeParams()
  {
    ros::NodeHandle pnh("~");

    // Read parameters from rosparam.
    GetRosParameter(pnh, "attitude_gain/x",
                    attitude_thrust_controller_.controller_parameters_.attitude_gain_.x(),
                    &attitude_thrust_controller_.controller_parameters_.attitude_gain_.x());
    GetRosParameter(pnh, "attitude_gain/y",
                    attitude_thrust_controller_.controller_parameters_.attitude_gain_.y(),
                    &attitude_thrust_controller_.controller_parameters_.attitude_gain_.y());
    GetRosParameter(pnh, "attitude_gain/z",
                    attitude_thrust_controller_.controller_parameters_.attitude_gain_.z(),
                    &attitude_thrust_controller_.controller_parameters_.attitude_gain_.z());
    GetRosParameter(pnh, "angular_rate_gain/x",
                    attitude_thrust_controller_.controller_parameters_.angular_rate_gain_.x(),
                    &attitude_thrust_controller_.controller_parameters_.angular_rate_gain_.x());
    GetRosParameter(pnh, "angular_rate_gain/y",
                    attitude_thrust_controller_.controller_parameters_.angular_rate_gain_.y(),
                    &attitude_thrust_controller_.controller_parameters_.angular_rate_gain_.y());
    GetRosParameter(pnh, "angular_rate_gain/z",
                    attitude_thrust_controller_.controller_parameters_.angular_rate_gain_.z(),
                    &attitude_thrust_controller_.controller_parameters_.angular_rate_gain_.z());
    GetVehicleParameters(pnh, &attitude_thrust_controller_.vehicle_parameters_);
    attitude_thrust_controller_.InitializeParameters();
  }

  void AttitudeThrustControllerNode::AttitudeThrustCallback(
      const mav_msgs::RollPitchYawrateThrustConstPtr &attitude_thrust_reference_msg)
  {
    mav_msgs::EigenRollPitchYawrateThrust attitude_thrust;
    mav_msgs::eigenRollPitchYawrateThrustFromMsg(*attitude_thrust_reference_msg, &attitude_thrust);
    attitude_thrust_controller_.SetAttitudeThrust(attitude_thrust);
  }

  void AttitudeThrustControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg)
  {

    ROS_INFO_ONCE("AttitudeThrustController got first odometry message.");


    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    attitude_thrust_controller_.SetOdometry(odometry);

#if (_DEBUG_TORQUE_THRUST_)
    Eigen::Vector4d ref_torque_thrust;
    attitude_thrust_controller_.CalculateTorqueThrust(&ref_torque_thrust);

    // IMPORTANT!!: torque_thrust_msg is in FLU frame
    mav_msgs::TorqueThrustPtr torque_thrust_msg(new mav_msgs::TorqueThrust);

    torque_thrust_msg->torque.x = ref_torque_thrust(0);
    torque_thrust_msg->torque.y = ref_torque_thrust(1);
    torque_thrust_msg->torque.z = ref_torque_thrust(2);
    torque_thrust_msg->thrust.x = 0;
    torque_thrust_msg->thrust.y = 0;
    torque_thrust_msg->thrust.z = ref_torque_thrust(3);
    torque_thrust_msg->header.stamp = odometry_msg->header.stamp;
    
    torque_thrust_reference_pub_.publish(torque_thrust_msg);
#endif

#if (!_DEBUG_TORQUE_THRUST_)
    Eigen::VectorXd ref_rotor_velocities;
    attitude_thrust_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    // Todo(ffurrer): Do this in the conversions header.
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
      actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    actuator_msg->header.stamp = odometry_msg->header.stamp;

    motor_velocity_reference_pub_.publish(actuator_msg);
#endif
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_thrust_controller_node");

  rotors_control::AttitudeThrustControllerNode attitude_thrust_controller_node;

  ros::spin();

  return 0;
}
