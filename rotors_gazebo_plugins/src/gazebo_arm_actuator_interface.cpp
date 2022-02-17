#include "rotors_gazebo_plugins/gazebo_arm_actuator_interface.h"

#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboArmActuatorInterface::~GazeboArmActuatorInterface(){
    
}

void GazeboArmActuatorInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf/*_sdf*/) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
      
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_,
                           true);

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  for(int i=0;i<link_name_right_.size();i++){
    link_ = model_->GetLink(link_name_right_[i]);
    if (link_ == NULL){
      gzthrow("[gazebo_arm_model] Couldn't find specified link \"" << link_name_right_[i]
                                                               << "\".");
    }else{
      link_right_.push_back(link_);
    };                                                 
  }

  for(int i=0;i<link_name_left_.size();i++){
    link_ = model_->GetLink(link_name_left_[i]);
    if (link_ == NULL){
      gzthrow("[gazebo_arm_model] Couldn't find specified link \"" << link_name_left_[i]
                                                               << "\".");
    }else{
      link_left_.push_back(link_);
    };                                                 
  }

  for(int i=0;i<joint_name_right_.size();i++){
    joint_ = model_->GetJoint(joint_name_right_[i]);
    if (joint_ == NULL){
      gzthrow("[gazebo_arm_model] Couldn't find specified link \"" << joint_name_right_[i]
                                                               << "\".");
    }else{
      joint_right_.push_back(joint_);
    };                                                 
  }

  for(int i=0;i<joint_name_left_.size();i++){
    joint_ = model_->GetJoint(joint_name_left_[i]);
    if (link_ == NULL){
      gzthrow("[gazebo_arm_model] Couldn't find specified link \"" << joint_name_left_[i]
                                                               << "\".");
    }else{
      joint_left_.push_back(joint_);
    };                                                 
  };

  pid_right_0.Init(0.05,0,0.01,0,0,0.15,-0.15);
  pid_right_1.Init(0.01,0,0.0001,0,0,0.15,0);
  pid_right_2.Init(0.01,0,0.0001,0,0,0.15,0);
  pid_right_3.Init(0.01,0,0.0001,0,0,0,-0.15);
  pid_right_4.Init(0.01,0,0.0001,0,0,0.15,0);
  pid_right_5.Init(0.05,0,0.01,0,0,0.15,-0.15);

  pid_left_0.Init(1,0,0,0,0,1.0,-1.0);
  pid_left_1.Init(1,0,0,0,0,1.0,0);
  pid_left_2.Init(1,0,0,0,0,1.0,0);
  pid_left_3.Init(1,0,0,0,0,0,-1.0);
  pid_left_4.Init(1,0,0,0,0,1.0,0);
  pid_left_5.Init(1,0,0,0,0,1.0,-1.0);
  

  motor_position_input_left = Eigen::VectorXd::Zero(6);
  motor_position_input_right = Eigen::VectorXd::Zero(6);
  tau_qF_left = Eigen::VectorXd::Zero(8);
  tau_qF_right = Eigen::VectorXd::Zero(8);

  Transform_qR2qF = Eigen::Matrix<double, 5, 8>::Zero();
  Transform_qR2qF<< 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 1;

  Transform_qM2qR=Eigen::Matrix<double, 6, 5>::Zero();
  Transform_qM2qR<< 1, 0, 0, 0, 0,
                    0, 1, 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0,-1,-1, 0, 0,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1;

  Transform_tauM2tauMR= Eigen::Matrix<double, 6, 6>::Zero();
  Transform_tauM2tauMR<<  1, 0, 0, 0, 0, 0,
                          0, ETA, 0, 0, 0, 0,
                          0, 0, ETA, 0, 0, 0,
                          0, 0, 0, ETA, 0, 0,
                          0, 0, 0, 0, ETA, 0,
                          0, 0, 0, 0, 0, 1;

  MAX_TORQUE= Eigen::Matrix<double, 1, 6>::Zero();
  MAX_TORQUE<< 8.383, 20.705, 20.705, 20.705, 20.705, 1.3986;

  Matrix_physicalLimit=Eigen::Matrix<double, 8, 8>::Zero();
  Matrix_physicalLimit<<  0, 0, 0, 0, 0, 0, 0, 0,
                          0, -100, 100, 0, 0, 0, 0, 0,
                          0, 100, -100, 0, 0, 0, 0, 0,
                          0, 0, 0, -100, 100, 0, 0, 0,
                          0, 0, 0, 100, -100, 0, 0, 0,
                          0, 0, 0, 0, 0, -100, 100, 0,
                          0, 0, 0, 0, 0, 100, -100, 0,
                          0, 0, 0, 0, 0, 0, 0, 0;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboArmActuatorInterface::OnUpdate, this, _1));

}

    // Called by the world update start event
void GazeboArmActuatorInterface::OnUpdate(const common::UpdateInfo& _info){
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
  
  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();

  CalculateJointTorque();

  PublishJointTorque();

}

void GazeboArmActuatorInterface::CalculateJointTorque(){

  Eigen::VectorXd tau_motor_right = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd tau_qR_right = Eigen::VectorXd::Zero(5);
  
  Eigen::VectorXd tau_elastic_right = Eigen::VectorXd::Zero(5);
  //update joint states from gazebo
  Eigen::VectorXd qF_fb_right = Eigen::VectorXd::Zero(8);
  Eigen::VectorXd qR_fb_right = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd motor_position_fb_right = Eigen::VectorXd::Zero(6);

  for(int i=0;i<joint_right_.size();i++){
    if (joint_right_.at(i)->Position() != ignition::math::NAN_D){
      qF_fb_right(i)=joint_right_.at(i)->Position();
    }else{
      gzthrow("[gazebo_arm_model] Couldn't get joint position \"" << i
                                                               << "\".");
    };                                                 
  }

  gzdbg << "qF_fb_right" << ":"<< qF_fb_right(0)<< "," << qF_fb_right(1)<< "," << qF_fb_right(2)<< "," << qF_fb_right(3) << ","
  << qF_fb_right(4)<< "," << qF_fb_right(5)<< "," << qF_fb_right(6) << ","<< qF_fb_right(7)  << std::endl;


  qR_fb_right = Transform_qR2qF * qF_fb_right;
  motor_position_fb_right = Transform_qM2qR * qR_fb_right;

  //calculate motor position errors
  Eigen::VectorXd err_right =  motor_position_fb_right - motor_position_input_right;
  
  gzdbg << "err_right" << ":"<< err_right(0)<< "," << err_right(1)<< "," << err_right(2)<< "," << err_right(3) << ","
  << err_right(4)<< "," << err_right(5)<< std::endl;


  Eigen::VectorXd pid_right=Eigen::VectorXd::Zero(6);

  pid_right(0)=pid_right_0.Update(err_right(0), sampling_time_);
  pid_right(1)=pid_right_1.Update(err_right(1), sampling_time_);
  pid_right(2)=pid_right_2.Update(err_right(2), sampling_time_);
  pid_right(3)=pid_right_3.Update(err_right(3), sampling_time_);
  pid_right(4)=pid_right_4.Update(err_right(4), sampling_time_);
  pid_right(5)=pid_right_5.Update(err_right(5), sampling_time_);
  gzdbg << "pid_right " << ":"<< pid_right(0)<< "," << pid_right(1)<< "," << pid_right(2)<< "," << pid_right(3) << ","
  << pid_right(4)<< "," << pid_right(5) << std::endl;
  for (int i = 0; i < pid_right.size(); i++){
    tau_motor_right(i) = MAX_TORQUE(i)*pid_right(i);
  }



  // for (int i = 0; i < pids_right_.size(); i++){
  //   tau_motor_right(i) = MAX_TORQUE(i)*pids_right_.at(i)->Update(err_right(i), sampling_time_);
  // }
  tau_elastic_right(3)=-0.024*(5*qR_fb_right(3)/M_PI_2);

  //from motor torque to fake joint torque
  tau_qR_right=Transform_qM2qR.transpose()* Transform_tauM2tauMR *tau_motor_right+tau_elastic_right;
  tau_qF_right=Transform_qR2qF.transpose()*tau_qR_right; 

  //tau_qF_right += Matrix_physicalLimit * qF_fb_right; //+physical limit
  

/*=================================================================*/

  Eigen::VectorXd tau_motor_left = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd tau_qR_left = Eigen::VectorXd::Zero(5);
  
  Eigen::VectorXd tau_elastic_left = Eigen::VectorXd::Zero(5);
  //update joint states from gazebo
  Eigen::VectorXd qF_fb_left = Eigen::VectorXd::Zero(8);
  Eigen::VectorXd qR_fb_left = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd motor_position_fb_left = Eigen::VectorXd::Zero(6);

  for(int i=0;i<joint_left_.size();i++){
    if (joint_left_.at(i)->Position() != ignition::math::NAN_D){
      qF_fb_left(i)=joint_left_.at(i)->Position();
    }else{
      gzthrow("[gazebo_arm_model] Couldn't get joint position \"" << i
                                                               << "\".");
    };                                                 
  }
  qR_fb_left = Transform_qR2qF * qF_fb_left;
  motor_position_fb_left = Transform_qM2qR * qR_fb_left;

  //calculate motor position errors
  Eigen::VectorXd err_left = motor_position_fb_left - motor_position_input_left;
  
  Eigen::VectorXd pid_left=Eigen::VectorXd::Zero(6);

  pid_left(0)=pid_left_0.Update(err_left(0), sampling_time_);
  pid_left(1)=pid_left_1.Update(err_left(1), sampling_time_);
  pid_left(2)=pid_left_2.Update(err_left(2), sampling_time_);
  pid_left(3)=pid_left_3.Update(err_left(3), sampling_time_);
  pid_left(4)=pid_left_4.Update(err_left(4), sampling_time_);
  pid_left(5)=pid_left_5.Update(err_left(5), sampling_time_);

  for (int i = 0; i < pid_left.size(); i++){
    tau_motor_left(i) = MAX_TORQUE(i)*pid_left(i);
  }

  tau_elastic_left(3)=-0.024*(5*qR_fb_right(3)/M_PI_2);
  // for (int i = 0; i < pids_left_.size(); i++){
  //   tau_motor_left(i) = MAX_TORQUE(i)*pids_left_.at(i)->Update(err_left(i), sampling_time_);
  // }
  //from motor torque to fake joint torque
  tau_qR_left=Transform_qM2qR.transpose()* Transform_tauM2tauMR *tau_motor_left+tau_elastic_left;
  tau_qF_left=Transform_qR2qF.transpose()* tau_qR_left; 

  //tau_qF_left += Matrix_physicalLimit * qF_fb_left; //+physical limit

}

void GazeboArmActuatorInterface::PublishJointTorque(){
  gzdbg << "link_right_torque" << ":"<< tau_qF_right(0)<< "," << tau_qF_right(1)<< "," << tau_qF_right(2)<< "," << tau_qF_right(3) << ","
  << tau_qF_right(4)<< "," << tau_qF_right(5)<< "," << tau_qF_right(6) << ","<< tau_qF_right(7)  << std::endl;

  
  link_right_.at(0)->AddRelativeTorque(ignition::math::Vector3d (0, 0, tau_qF_right(0)));
  link_right_.at(7)->AddRelativeTorque(ignition::math::Vector3d (0, 0, tau_qF_right(7)));
  link_right_.at(1)->AddRelativeTorque(ignition::math::Vector3d (tau_qF_right(1), 0, 0));
  link_right_.at(2)->AddRelativeTorque(ignition::math::Vector3d (tau_qF_right(2), 0, 0));

  link_right_.at(5)->AddRelativeTorque(ignition::math::Vector3d (tau_qF_right(5), 0, 0));
  link_right_.at(6)->AddRelativeTorque(ignition::math::Vector3d (tau_qF_right(6), 0, 0));

  for(int i=0;i<link_right_.size();i++){

   //link_right_.at(i)->AddRelativeTorque((i==0||i==7)?(ignition::math::Vector3d (0, 0, tau_qF_right(i))):(ignition::math::Vector3d (tau_qF_right(i), 0, 0)));
  }
  for(int i=0;i<link_left_.size();i++){
    //link_left_.at(i)->AddRelativeTorque((i==0||i==7)?(ignition::math::Vector3d (0, 0, tau_qF_left(i))):(ignition::math::Vector3d (tau_qF_left(i), 0, 0)));
  }

}


void GazeboArmActuatorInterface::CreatePubsAndSubs() {
  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  // = ACTUATOR COMMAND MSG SETUP (ROS->GAZEBO) = //
  // ============================================ //

  command_arm_actuator_sub_left_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + command_arm_actuator_sub_topic_left_,
      &GazeboArmActuatorInterface::CommandArmActuatorCallback_left, this);

  // Connect to ROS
    connect_ros_to_gazebo_topic_msg.set_ros_topic(
      "/left_arm/motor_position");
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + command_arm_actuator_sub_topic_left_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::JOINT_STATE);
  gz_connect_ros_to_gazebo_topic_pub->Publish(
      connect_ros_to_gazebo_topic_msg, true);

  command_arm_actuator_sub_left_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + command_arm_actuator_sub_topic_right_,
      &GazeboArmActuatorInterface::CommandArmActuatorCallback_right, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(
      "/right_arm/motor_position");
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + command_arm_actuator_sub_topic_right_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::JOINT_STATE);
  gz_connect_ros_to_gazebo_topic_pub->Publish(
      connect_ros_to_gazebo_topic_msg, true);

}

void GazeboArmActuatorInterface::CommandArmActuatorCallback_right(GzJointStateMsgPtr& joint_state_msg){

  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  motor_position_input_right.resize(joint_state_msg->position_size());
  for (int i = 0; i < joint_state_msg->position_size(); ++i) {
    motor_position_input_right(i) = joint_state_msg->position(i);
  }
  
}


void GazeboArmActuatorInterface::CommandArmActuatorCallback_left(GzJointStateMsgPtr& joint_state_msg){
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  motor_position_input_left.resize(joint_state_msg->position_size());
  for (int i = 0; i < joint_state_msg->position_size(); ++i) {
    motor_position_input_left(i) = joint_state_msg->position(i);
  }
  
}

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboArmActuatorInterface);
}