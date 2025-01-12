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
      pos_err_integrater(initialIntegration)

  {  
    f = boost::bind(&rotors_control::PositionControllerRPYT::tune_callback,this, _1, _2);
    server.setCallback(f);

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

  }


#if (_DEBUG_TORQUE_THRUST_)

  void PositionControllerRPYT::tune_callback(rotors_control::tuneConfig &config, uint32_t level) {
     
     
    controller_parameters_.integration_gain_.x() = config.i_x;
    controller_parameters_.integration_gain_.y() = config.i_y;
    controller_parameters_.integration_gain_.z() = config.i_z;     

    controller_parameters_.position_gain_.x() = config.p_x;
    controller_parameters_.position_gain_.y() = config.p_y;
    controller_parameters_.position_gain_.z() = config.p_z;

    controller_parameters_.velocity_gain_.x() = config.d_x;
    controller_parameters_.velocity_gain_.y() = config.d_y;
    controller_parameters_.velocity_gain_.z() = config.d_z;
    //std::cot << "input:" << position_controller_.controller_parameters_.position_gain_.z() <<std::endl;


  }


  void PositionControllerRPYT::CalculateAttiThrust(mav_msgs::RollPitchYawrateThrust *atti_thrust)
  {
    assert(atti_thrust);
    assert(initialized_params_);

    // Return 0 velocities on all rotors, until the first command is received.
    // if (!controller_active_)
    // {
    //   atti_thrust->pitch = 0;
    //   atti_thrust->roll=0;
    //   atti_thrust->yaw_rate=0;
    //   atti_thrust->thrust.z = 0;
    //   return;
    // }
    Eigen::Vector3d acceleration;

    Eigen::Vector3d position_error;
    position_error = command_trajectory_.position_W - odometry_.position;

    // Transform velocity to world frame.
    const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
    Eigen::Vector3d velocity_W = R_W_I * odometry_.velocity;
    Eigen::Vector3d velocity_error;
    velocity_error = command_trajectory_.velocity_W - velocity_W;

    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
    /*compared with paper of LEE(15), the reference frame in paper is FRD, here is FLU, thus reverse gravity*/
    acceleration = (pos_err_integrater.cwiseProduct(controller_parameters_.integration_gain_)+position_error.cwiseProduct(controller_parameters_.position_gain_) + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_ + vehicle_parameters_.gravity_ * e_3 + command_trajectory_.acceleration_W;
    pos_err_integrater += position_error;

    Eigen::Vector3d desired_angle;
    // Get the desired rotation matrix.

    double yaw = command_trajectory_.getYaw();

    // Eigen::Vector3d b1_des;
    // b1_des << cos(yaw), sin(yaw), 0; //this makes the yaw command is aligned with yaw in 312 eular order

    Eigen::Vector3d b2_des;
    b2_des << -sin(yaw), cos(yaw), 0;  //this makes the yaw command is aligned with yaw in 321 eular order

    Eigen::Vector3d b3_des;
    b3_des = acceleration.normalized();
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des).normalized();
    R_des.col(1) = b3_des.cross(R_des.col(0)).normalized();
    R_des.col(2) = b3_des;
    // in FLU
    // Eigen::Matrix3d R_test;

    // R_test.col(1) = b3_des.cross(b1_des).normalized();
    // R_test.col(0) = R_test.col(1).cross(b3_des).normalized();
    // R_test.col(2) = b3_des;

    Eigen::Quaterniond Q_des(R_des);
    Eigen::Vector3d RPY;
    getEulerAnglesFromQuaternion(Q_des,&RPY);

    // Eigen::Quaterniond Q_test(R_test);
    // Eigen::Vector3d RPY_test;
    // getEulerAnglesFromQuaternion(Q_test,&RPY_test);
    // std::cout<< "test" <<  RPY_test(2)-yaw << "---" << RPY(2)-yaw <<std::endl;

    desired_angle = RPY;

    // Project thrust onto body z axis.
    double thrust = vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));
    
    atti_thrust->roll = desired_angle(0);
    atti_thrust->pitch = desired_angle(1);
    atti_thrust->yaw_rate = desired_angle(2);
    atti_thrust->thrust.z = thrust;
  }


#endif

}
