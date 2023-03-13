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

#include "rotors_control/position_controller.h"

namespace rotors_control
{

  PositionController::PositionController()
      : initialized_params_(false),
        controller_active_(false)
  {
    InitializeParameters();
  }

  PositionController::~PositionController() {}

  void PositionController::InitializeParameters()
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

  void PositionController::SetOdometry(const EigenOdometry &odometry)
  {
    odometry_ = odometry;
  }

  void PositionController::SetTrajectoryPoint(
      const mav_msgs::EigenTrajectoryPoint &command_trajectory)
  {
    command_trajectory_ = command_trajectory;
    controller_active_ = true;
  }

  void PositionController::ComputeDesiredAcceleration(Eigen::Vector3d *acceleration) const
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

  // Implementation from the T. Lee et al. paper
  // Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
  void PositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d &acceleration,
                                                       Eigen::Vector3d *angular_acceleration) const
  {
    assert(angular_acceleration);

    Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

    // Get the desired rotation matrix.
    Eigen::Vector3d b1_des;
    double yaw = command_trajectory_.getYaw();
    b1_des << cos(yaw), sin(yaw), 0;

    Eigen::Vector3d b3_des;
    b3_des = -acceleration / acceleration.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des.normalize();

    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des);
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;

    // Angle error according to lee et al.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d angle_error;
    vectorFromSkewMatrix(angle_error_matrix, &angle_error);

    // TODO(burrimi) include angular rate references at some point.
    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = command_trajectory_.getYawRate();

    Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

    *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_) - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_) + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
  }

#if (_DEBUG_TORQUE_THRUST_)
  void PositionController::CalculateRPYThrust(mav_msgs::RollPitchYawrateThrust *rpy_thrust) const
  {
    assert(rpy_thrust);
    assert(initialized_params_);

    // Return 0 velocities on all rotors, until the first command is received.
    if (!controller_active_)
    {
      rpy_thrust->pitch=0;
      rpy_thrust->roll=0;
      rpy_thrust->yaw_rate=0;
      rpy_thrust->thrust.z = 0;
      return;
    }
    Eigen::Vector3d acceleration;
    ComputeDesiredAcceleration(&acceleration);

    Eigen::Vector3d angle;
    ComputeDesiredAngle(acceleration, &angle);

    // Project thrust onto body z axis.
    double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));
    
    rpy_thrust->roll=angle(0);
    rpy_thrust->pitch=angle(1);
    rpy_thrust->yaw_rate=angle(2);

    rpy_thrust->thrust.z = thrust;
  }

  void PositionController::ComputeDesiredAngle(const Eigen::Vector3d &acceleration, Eigen::Vector3d *desired_angle) const
  {
    assert(desired_angle);

    // Get the desired rotation matrix.
    Eigen::Vector3d b1_des;
    double yaw = odometry_.orientation.toRotationMatrix().eulerAngles(2, 1, 0)(0);
    b1_des << cos(yaw), sin(yaw), 0; 

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

    Eigen::Vector3d YPR = R_des.eulerAngles(2, 1, 0);

    //std::cout << "posi R_des:" << R_des <<std::endl;
    std::cout << "pitch:" << YPR(1)  << "roll:" << YPR(2)  << "yaw:" << YPR(0)  << "yaw_cal:" << yaw <<std::endl;

    // TODO(burrimi) include angular rate references at some point.
    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = command_trajectory_.getYawRate();

    Eigen::Vector3d RPYawrate;
    RPYawrate(0)=YPR(2);
    RPYawrate(1)=YPR(1);    
    RPYawrate(2)=angular_rate_des[2]; 
    *desired_angle=RPYawrate;

  }
#endif

}
