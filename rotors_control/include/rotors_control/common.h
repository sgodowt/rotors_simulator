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

#ifndef INCLUDE_ROTORS_CONTROL_COMMON_H_
#define INCLUDE_ROTORS_CONTROL_COMMON_H_

#define _DEBUG_TORQUE_THRUST_ 1

#include <assert.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>

#include "rotors_control/parameters.h"

namespace rotors_control
{

  // Default values.
  static const std::string kDefaultNamespace = "";
  static const std::string kDefaultCommandMotorSpeedTopic =
      mav_msgs::default_topics::COMMAND_ACTUATORS; // "command/motor_speed";
  static const std::string kDefaultCommandMultiDofJointTrajectoryTopic =
      mav_msgs::default_topics::COMMAND_TRAJECTORY; // "command/trajectory"
  static const std::string kDefaultCommandRollPitchYawrateThrustTopic =
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST;
  static const std::string kDefaultCommandAttitudeThrustTopic =
      mav_msgs::default_topics::COMMAND_ATTITUDE_THRUST;
  // "command/roll_pitch_yawrate_thrust"
  static const std::string kDefaultImuTopic =
      mav_msgs::default_topics::IMU; // "imu
  static const std::string kDefaultOdometryTopic =
      mav_msgs::default_topics::ODOMETRY; // "odometry"

#if (_DEBUG_TORQUE_THRUST_)
  static const std::string kDefaultCommandTorqueThrustTopic =
      "command/torque_thrust"; // "command/torque_thrust";
#endif

  struct EigenOdometry
  {
    EigenOdometry()
        : position(0.0, 0.0, 0.0),
          orientation(Eigen::Quaterniond::Identity()),
          velocity(0.0, 0.0, 0.0),
          angular_velocity(0.0, 0.0, 0.0){};

    EigenOdometry(const Eigen::Vector3d &_position,
                  const Eigen::Quaterniond &_orientation,
                  const Eigen::Vector3d &_velocity,
                  const Eigen::Vector3d &_angular_velocity)
    {
      position = _position;
      orientation = _orientation;
      velocity = _velocity;
      angular_velocity = _angular_velocity;
    };

    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
    Eigen::Vector3d angular_velocity;
  };

  inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr &msg,
                                   EigenOdometry *odometry)
  {
    odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
    odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
    odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
    odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
  }

  inline void calculateAllocationMatrix(const RotorConfiguration &rotor_configuration,
                                        Eigen::Matrix4Xd *allocation_matrix)
  {
    assert(allocation_matrix != nullptr);
    allocation_matrix->resize(4, rotor_configuration.rotors.size());
    unsigned int i = 0;
    for (const Rotor &rotor : rotor_configuration.rotors)
    {
      // Set first row of allocation matrix. in FLU frame
      (*allocation_matrix)(0, i) = sin(rotor.angle) * rotor.arm_length * rotor.rotor_force_constant;
      // Set second row of allocation matrix.
      (*allocation_matrix)(1, i) = -cos(rotor.angle) * rotor.arm_length * rotor.rotor_force_constant;
      // Set third row of allocation matrix.
      (*allocation_matrix)(2, i) = -rotor.direction * rotor.rotor_force_constant * rotor.rotor_moment_constant;
      // Set forth row of allocation matrix. Thrust
      (*allocation_matrix)(3, i) = rotor.rotor_force_constant;
      ++i;
    }
    Eigen::FullPivLU<Eigen::Matrix4Xd> lu(*allocation_matrix);
    // Setting the threshold for when pivots of the rank calculation should be considered nonzero.
    lu.setThreshold(1e-9);
    int rank = lu.rank();
    if (rank < 4)
    {
      std::cout << "The rank of the allocation matrix is " << lu.rank()
                << ", it should have rank 4, to have a fully controllable system,"
                << " check your configuration." << std::endl;
    }
  }

  inline void skewMatrixFromVector(Eigen::Vector3d &vector, Eigen::Matrix3d *skew_matrix)
  {
    *skew_matrix << 0, -vector.z(), vector.y(),
        vector.z(), 0, -vector.x(),
        -vector.y(), vector.x(), 0;
  }

  inline void vectorFromSkewMatrix(Eigen::Matrix3d &skew_matrix, Eigen::Vector3d *vector)
  {
    *vector << skew_matrix(2, 1), skew_matrix(0, 2), skew_matrix(1, 0);
  }

inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                         Eigen::Vector3d* euler_angles) {
    assert(euler_angles != NULL);

    *euler_angles << std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
        std::asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
        std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}


// inline void getRotationMatrixFromEulerAngles(const Eigen::Vector3d& euler_angles,
//                                         Eigen::Matrix3d *R) {
//     assert(R != NULL);
//     Eigen::Matrix3d R_x, R_y, R_z;
//     R_z << cos(euler_angles.z()),-sin(euler_angles.z()),0,sin(euler_angles.z()),cos(euler_angles.z()),0,0,0,1;
//     R_y << cos(euler_angles.y()),0,sin(euler_angles.y()),0,1,0,-sin(euler_angles.y()),0,cos(euler_angles.y());
//     R_x <<1,0,0,0,cos(euler_angles.x()),-sin(euler_angles.x()),0,sin(euler_angles.x()),cos(euler_angles.x());
//     *R = R_z*R_y*R_x;
// }


// inline void getEulerAnglesFromRotationMatrix(const Eigen::Matrix3d R,
//                                         Eigen::Vector3d *eular_angles) {
//     //did not complete all of the possible angles
//     assert(eular_angles != NULL);
//     double yaw = atan2(R(1,0),R(0,0));
//     double pitch = asin(-R(2,0));
//     double roll = atan2(R(2,1),R(2,2));
//     *eular_angles = Eigen::Vector3d(roll, pitch ,yaw);
// }

}

#endif /* INCLUDE_ROTORS_CONTROL_COMMON_H_ */
