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

#ifndef ROTORS_CONTROL_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

#include <dynamic_reconfigure/server.h>
#include <rotors_control/tuneConfig.h>

namespace rotors_control
{

  // Default values for the lee position controller and the Asctec Firefly.
  static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
  static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
  static const Eigen::Vector3d kDefaultIntegrationGain = Eigen::Vector3d(0, 0, 0);
  static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
  static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);
  static const Eigen::Vector3d initialIntegration = Eigen::Vector3d(0, 0, 0);

  class PositionControllerRPYTParameters
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PositionControllerRPYTParameters()
        : position_gain_(kDefaultPositionGain),
          velocity_gain_(kDefaultVelocityGain),
          integration_gain_(kDefaultIntegrationGain),
          attitude_gain_(kDefaultAttitudeGain),
          angular_rate_gain_(kDefaultAngularRateGain)
    {
      calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
    }

    Eigen::Matrix4Xd allocation_matrix_;
    Eigen::Vector3d integration_gain_;
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d angular_rate_gain_;
    RotorConfiguration rotor_configuration_;
  };

  class PositionControllerRPYT
  {
  public:
    PositionControllerRPYT();
    ~PositionControllerRPYT();

    void InitializeParameters();
    void CalculateRotorVelocities(Eigen::VectorXd *rotor_velocities) const;

#if (_DEBUG_TORQUE_THRUST_)
    void CalculateAttiThrust(mav_msgs::RollPitchYawrateThrust *atti_thrust);
    void ComputeDesiredAttitude(const Eigen::Vector3d &acceleration, Eigen::Vector3d *desired_angle) const;
#endif
    void SetOdometry(const EigenOdometry &odometry);

    void tune_callback(rotors_control::tuneConfig &config, uint32_t level);

    void SetTrajectoryPoint(
        const mav_msgs::EigenTrajectoryPoint &command_trajectory);

    PositionControllerRPYTParameters controller_parameters_;
    VehicleParameters vehicle_parameters_;

    dynamic_reconfigure::Server<rotors_control::tuneConfig> server;
    dynamic_reconfigure::Server<rotors_control::tuneConfig>::CallbackType f;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    bool initialized_params_;
    
    Eigen::Vector3d pos_err_integrater;
    Eigen::Vector3d normalized_attitude_gain_;
    Eigen::Vector3d normalized_angular_rate_gain_;
    Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

    mav_msgs::EigenTrajectoryPoint command_trajectory_;
    EigenOdometry odometry_;

    void ComputeDesiredAngularAcc(const Eigen::Vector3d &acceleration,
                                  Eigen::Vector3d *angular_acceleration) const;
    void ComputeDesiredAcceleration(Eigen::Vector3d *acceleration) const;
  };
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
