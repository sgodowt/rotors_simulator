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

#include "rotors_control/position_controller_RPYT.h"

namespace rotors_control
{

  PositionControllerRPYT::PositionControllerRPYT()
      : initialized_params_(false),
        controller_active_(false)
  {
    InitializeParameters();
  }

  PositionControllerRPYT::~PositionControllerRPYT() {}

  void PositionControllerRPYT::InitializeParameters()
  {
    calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
    // To make the tuning independent of the inertia matrix we divide here.
    normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose() * vehicle_parameters_.inertia_.inverse();
    // To make the tuning independent of the inertia matrix we divide here.
    normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose() * vehicle_parameters_.inertia_.inverse();

    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
    I(3, 3) = 1;
    angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
    // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
    // A^{ \dagger} = A^T*(A*A^T)^{-1}
    angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose() * (controller_parameters_.allocation_matrix_ * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
    initialized_params_ = true;
  }

  void PositionControllerRPYT::SetOdometry(const EigenOdometry &odometry)
  {
    odometry_ = odometry;
  }

  void PositionControllerRPYT::SetTrajectoryPoint(
      const mav_msgs::EigenTrajectoryPoint &command_trajectory)
  {
    command_trajectory_ = command_trajectory;
    controller_active_ = true;
  }

  void PositionControllerRPYT::ComputeDesiredAcceleration(Eigen::Vector3d *acceleration) const
  {
    assert(acceleration);

    Eigen::Vector3d position_error;
    position_error = odometry_.position - command_trajectory_.position_W;

    // Transform velocity to world frame.
    const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
    Eigen::Vector3d velocity_W = R_W_I * odometry_.velocity;
    Eigen::Vector3d velocity_error;
    velocity_error = velocity_W - command_trajectory_.velocity_W;

    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
    /*compared with paper of LEE(15), the reference frame in paper is FRD, here is FLU, thus reverse gravity*/
    *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_) + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_ - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
  }

#if (_DEBUG_TORQUE_THRUST_)

  void PositionControllerRPYT::ComputeDesiredAttitude(const Eigen::Vector3d &acceleration, Eigen::Vector3d *desired_angle) const
  {
    assert(desired_angle);

    // Get the desired rotation matrix.
    Eigen::Vector3d b1_des;

    double yaw = command_trajectory_.getYaw();

    b1_des << cos(yaw), sin(yaw), 0; // yaw

    Eigen::Vector3d b3_des;
    b3_des = -acceleration / acceleration.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des.normalize();

    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des);
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;
    // in FLU


    Eigen::Quaterniond Q_des(R_des);
    Eigen::Vector3d RPY;
    getEulerAnglesFromQuaternion(Q_des,&RPY);

    *desired_angle = RPY;

  }

  void PositionControllerRPYT::CalculateAttiThrust(mav_msgs::RollPitchYawrateThrust *atti_thrust) const
  {
    assert(atti_thrust);
    assert(initialized_params_);

    // Return 0 velocities on all rotors, until the first command is received.
    if (!controller_active_)
    {
      atti_thrust->pitch = 0;
      atti_thrust->roll=0;
      atti_thrust->yaw_rate=0;
      atti_thrust->thrust.z = 0;
      return;
    }
    Eigen::Vector3d acceleration;
    ComputeDesiredAcceleration(&acceleration);

    Eigen::Vector3d angle;
    ComputeDesiredAttitude(acceleration, &angle);

    // Project thrust onto body z axis.
    double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));
    
    atti_thrust->roll = angle(0);
    atti_thrust->pitch = angle(1);
    atti_thrust->yaw_rate = angle(2);
    atti_thrust->thrust.z = thrust;
  }


#endif

}
