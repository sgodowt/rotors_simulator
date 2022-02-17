#ifndef ROTORS_GAZEBO_PLUGINS_ARM_ACTUATOR_INTERFACE_H
#define ROTORS_GAZEBO_PLUGINS_ARM_ACTUATOR_INTERFACE_H

#include <Eigen/Eigen>

#include <boost/shared_ptr.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h> 

#include "JointState.pb.h"

#include "rotors_gazebo_plugins/common.h"


#define ETA                 ((double)(0.048/0.021))

namespace gazebo {

static const std::string kDefaultCommandArmActuatorSubTopic_left = "left_arm/motor_position";
static const std::string kDefaultCommandArmActuatorSubTopic_right = "right_arm/motor_position";

typedef const boost::shared_ptr<const gz_sensor_msgs::JointState>
    GzJointStateMsgPtr;

class GazeboArmActuatorInterface: public ModelPlugin {
    public:
    GazeboArmActuatorInterface()
        : ModelPlugin(),
        namespace_(kDefaultNamespace),
        node_handle_(NULL),
        pubs_and_subs_created_(false),
        prev_sim_time_(0.0),
        sampling_time_(0.01),
        command_arm_actuator_sub_topic_left_(kDefaultCommandArmActuatorSubTopic_left),
        command_arm_actuator_sub_topic_right_(kDefaultCommandArmActuatorSubTopic_right)
        {}

    ~GazeboArmActuatorInterface();

    protected:
    /// \brief Load the plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Called when the world is updated.
    /// \param[in] _info Update timing information.
    void OnUpdate(const common::UpdateInfo& /*_info*/);

    private:
    /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
    ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
    bool pubs_and_subs_created_;

    /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
    /// \details  Call this once the first time OnUpdate() is called (can't
    ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
    ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
    void CreatePubsAndSubs();

    event::ConnectionPtr update_connection_;

    physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    physics::JointPtr joint_;

    std::vector<std::string> link_name_left_={"Link_shoulder_left",
          "Link_one_left",
          "Link_two_left",
          "Link_three_left",
          "Link_hand_left"
          };
    std::vector<std::string> link_name_right_={"Link_shoulder_right",
          "Link_one_right",
          "Link_two_right",
          "Link_three_right",
          "Link_hand_right"
          };

    std::vector<std::string> joint_name_left_={"Joint_shoulder_left",
          "Joint_1_left",
          "Joint_2_left",
          "Joint_3_left",
          "Joint_wrist_left"
          };
    std::vector<std::string> joint_name_right_={"Joint_shoulder_right",
          "Joint_1_right",
          "Joint_2_right",
          "Joint_3_right",
          "Joint_wrist_right"
          };

    Eigen::MatrixXd  Transform_qR2qF;

    Eigen::MatrixXd Transform_qM2qR,Transform_qR2qM;

    Eigen::MatrixXd Transform_tauM2tauMR;

    Eigen::MatrixXd MAX_TORQUE,MAX_TORQUE_QR,NOMINAL_TORQUE_QR;
    Eigen::MatrixXd Matrix_physicalLimit;

    physics::Link_V link_left_, link_right_;
    physics::Joint_V joint_left_, joint_right_;
    std::string namespace_;

    std::string command_arm_actuator_sub_topic_left_;
    std::string command_arm_actuator_sub_topic_right_;

    gazebo::transport::SubscriberPtr command_arm_actuator_sub_left_;
    gazebo::transport::SubscriberPtr command_arm_actuator_sub_right_;

    void CommandArmActuatorCallback_right(GzJointStateMsgPtr& joint_state_msg);
    void CommandArmActuatorCallback_left(GzJointStateMsgPtr& joint_state_msg);
    void CalculateJointTorque();
    void PublishJointTorque();

    Eigen::VectorXd motor_position_input_left,motor_position_input_right,tau_qF_left,tau_qF_right,tau_qR_left,tau_qR_right;
  
    gazebo::transport::NodePtr node_handle_;

    double prev_sim_time_;
    double sampling_time_;

    common::PID pid_right_0,pid_right_1,pid_right_2,pid_right_3,pid_right_4,pid_right_5;
    common::PID pid_left_0,pid_left_1,pid_left_2,pid_left_3,pid_left_4,pid_left_5; 
	common::PID pid_right_qR_0,pid_right_qR_1,pid_right_qR_2,pid_right_qR_3,pid_right_qR_4;
	common::PID pid_left_qR_0,pid_left_qR_1,pid_left_qR_2,pid_left_qR_3,pid_left_qR_4;
};


}

#endif