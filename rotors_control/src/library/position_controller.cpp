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
    Eigen::Vector3d position_error;
    position_error = command_trajectory_.position_W - odometry_.position ;

    // Transform velocity to world frame.
    const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
    Eigen::Vector3d velocity_W = R_W_I * odometry_.velocity;
    Eigen::Vector3d velocity_error;
    velocity_error = command_trajectory_.velocity_W-velocity_W ;

    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
    /*compared with paper of LEE(15), the reference frame in paper is FRD, here is FLU, thus reverse gravity*/
    acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_) + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_ + vehicle_parameters_.gravity_ * e_3 + command_trajectory_.acceleration_W;

   //this comment codes below will cause unstable, althought the yaw is the same.
    // Eigen::Vector3d b1_des;
    // double yaw = odometry_.orientation.toRotationMatrix().eulerAngles(2, 1, 0)(0);
    // b1_des << cos(yaw), sin(yaw), 0; 

    // Get the desired rotation matrix.
    /*compared with paper of LEE(15), the eular order is 312, here is 321, thus use b2_des*/
    Eigen::Vector3d b2_des;
    double yaw_d = atan2(odometry_.orientation.toRotationMatrix().col(0)(1),odometry_.orientation.toRotationMatrix().col(0)(0)) ;
    b2_des << -sin(yaw_d), cos(yaw_d), 0;  //321

    Eigen::Vector3d b3_des;
    b3_des = acceleration.normalized();
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des).normalized();
    R_des.col(1) = b3_des.cross(R_des.col(0)).normalized();
    R_des.col(2) = b3_des;
    // in FLU

    //Eigen::Vector3d b1_d = R_des.col(0);
    //double yaw_des = atan2(b1_d(1),b1_d(0));

    Eigen::Quaterniond Q_des(R_des);
    Eigen::Vector3d RPY_des;
    getEulerAnglesFromQuaternion(Q_des,&RPY_des);
    //std::cout << "posi R_des:" << R_des <<std::endl;
    //std::cout << "yaw_feed:" << yaw_d << "yaw_d:" << yaw_des <<std::endl;

    // TODO(burrimi) include angular rate references at some point.

    // Project thrust onto body z axis.
    double thrust = vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));
    
    rpy_thrust->roll=RPY_des(0);
    rpy_thrust->pitch=RPY_des(1);
    rpy_thrust->yaw_rate=command_trajectory_.getYawRate(); 

    rpy_thrust->thrust.z = thrust;
  }

#endif

}
