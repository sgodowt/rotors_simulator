#include "rotors_gazebo_plugins/gazebo_arm_actuator_interface_nofakelink.h"

#include "ConnectRosToGazeboTopic.pb.h"


#include <ros/ros.h>

ros::NodeHandle* ros_node_handle_;

namespace gazebo {

GazeboArmActuatorInterface::~GazeboArmActuatorInterface(){
    
}

void GazeboArmActuatorInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf/*_sdf*/) {

   ros_node_handle_ = new ros::NodeHandle();

  double p = 0;
double i = 0;
double d = 0;
double iMax = 0;
double iMin = 0;
double cmdMax = 0;
  double cmdMin = 0;
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
      
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_,
                           true);

    if (_sdf->HasElement("gazebo_joint_control_pid")) {
      sdf::ElementPtr pid = _sdf->GetElement("gazebo_joint_control_pid");

       gzdbg<< "gazebo_joint_control_pid found!";

      if (pid->HasElement("p"))
        p = pid->Get<double>("p");
      
      if (pid->HasElement("i"))
        i = pid->Get<double>("i");
      
      if (pid->HasElement("d"))
        d = pid->Get<double>("d");
      
      if (pid->HasElement("iMax"))
        iMax = pid->Get<double>("iMax");
      
      if (pid->HasElement("iMin"))
        iMin = pid->Get<double>("iMin");
      
      if (pid->HasElement("cmdMax"))
        cmdMax = pid->Get<double>("cmdMax");
    
      if (pid->HasElement("cmdMin"))
        cmdMin = pid->Get<double>("cmdMin");
    } else {
      gzerr << "[gazebo_motor_model] PID values not found, Setting all values "
               "to zero!\n";
    }

                           
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

  pid_right_0.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_1.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_2.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_3.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_4.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_5.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);

  pid_left_0.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_left_1.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_left_2.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_left_3.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_left_4.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_left_5.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  
  pid_right_qR_0.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_qR_1.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_qR_2.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_qR_3.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
  pid_right_qR_4.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);

  pid_left_qR_0.Init(1,0,0,0,0,0.15,-0.15);
  pid_left_qR_1.Init(1,0,0,0,0,0.15,0);
  pid_left_qR_2.Init(1,0,0,0,0,0.15,0);
  pid_left_qR_3.Init(1,0,0,0,0,0,-0.15);
  pid_left_qR_4.Init(1,0,0,0,0,0.15,0);
  
  MAX_TORQUE_QR= Eigen::VectorXd::Zero(5);
  MAX_TORQUE_QR<< 8.383, 20.705*ETA, 20.705*ETA, 20.705*ETA, 1.3986;

  NOMINAL_TORQUE_QR= Eigen::VectorXd::Zero(5);
  NOMINAL_TORQUE_QR<< 1.2, 6.4*ETA, 6.4*ETA, 6.4*ETA, 0.5883;

  motor_position_input_left = Eigen::VectorXd::Zero(6);
  motor_position_input_right = Eigen::VectorXd::Zero(6);

  tau_qF_left = Eigen::VectorXd::Zero(8);
  tau_qF_right = Eigen::VectorXd::Zero(8);
  
  tau_qR_left = Eigen::VectorXd::Zero(5);
  tau_qR_right = Eigen::VectorXd::Zero(5);


  Transform_qR2qF = Eigen::Matrix<double, 5, 8>::Zero();
  Transform_qR2qF<< 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 1, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 1;

  Transform_qM2qR=Eigen::Matrix<double, 6, 5>::Zero();
  Transform_qM2qR<< 1, 0, 0, 0, 0,
                    0, ETA, 0, 0, 0,
                    0, 0, ETA, 0, 0,
                    0,-ETA,-ETA, 0, 0,
                    0, 0, 0, ETA, 0,
                    0, 0, 0, 0, 1;

  Transform_qR2qM=Eigen::Matrix<double, 5, 6>::Zero();
  Transform_qR2qM <<       1.0000,         0,         0,         0,         0,         0,
         0,    0.6667/ETA,   -0.3333/ETA,   -0.3333/ETA,         0,         0,
         0,   -0.3333/ETA,    0.6667/ETA,   -0.3333/ETA,         0,         0,
         0,         0,         0,         0,    1.0000/ETA,         0,
         0,         0,         0,         0,         0,    1.0000;

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

  double p,i,d,imax,imin,cmdmax,cmdmin;
  ros_node_handle_->getParam("p_0",p);
  ros_node_handle_->getParam("i_0",i);
  ros_node_handle_->getParam("d_0",d);
  ros_node_handle_->getParam("imax_0",imax);
  ros_node_handle_->getParam("imin_0",imin);
  ros_node_handle_->getParam("cmdmax_0",cmdmax);
  ros_node_handle_->getParam("cmdmin_0",cmdmin);

  //   gzdbg << "pid_right_qR_0 " << ":"<< p << "," << i<< "," << d<< "," << imax << ","
  // << imin << std::endl;

  pid_right_qR_0.SetPGain(p);
  pid_right_qR_0.SetIGain(i);
  pid_right_qR_0.SetDGain(d);
  pid_right_qR_0.SetIMax(imax);
  pid_right_qR_0.SetIMin(imin);
  pid_right_qR_0.SetCmdMax(cmdmax);
  pid_right_qR_0.SetCmdMin(cmdmin);

  ros_node_handle_->getParam("p_1",p);
  ros_node_handle_->getParam("i_1",i);
  ros_node_handle_->getParam("d_1",d);
  ros_node_handle_->getParam("imax_1",imax);
  ros_node_handle_->getParam("imin_1",imin);
  ros_node_handle_->getParam("cmdmax_1",cmdmax);
  ros_node_handle_->getParam("cmdmin_1",cmdmin);

  pid_right_qR_1.SetPGain(p);
  pid_right_qR_1.SetIGain(i);
  pid_right_qR_1.SetDGain(d);
  pid_right_qR_1.SetIMax(imax);
  pid_right_qR_1.SetIMin(imin);
  pid_right_qR_1.SetCmdMax(cmdmax);
  pid_right_qR_1.SetCmdMin(cmdmin);

    ros_node_handle_->getParam("p_2",p);
  ros_node_handle_->getParam("i_2",i);
  ros_node_handle_->getParam("d_2",d);
  ros_node_handle_->getParam("imax_2",imax);
  ros_node_handle_->getParam("imin_2",imin);
  ros_node_handle_->getParam("cmdmax_2",cmdmax);
  ros_node_handle_->getParam("cmdmin_2",cmdmin);

    pid_right_qR_2.SetPGain(p);
  pid_right_qR_2.SetIGain(i);
  pid_right_qR_2.SetDGain(d);
  pid_right_qR_2.SetIMax(imax);
  pid_right_qR_2.SetIMin(imin);
  pid_right_qR_2.SetCmdMax(cmdmax);
  pid_right_qR_2.SetCmdMin(cmdmin);

    ros_node_handle_->getParam("p_3",p);
  ros_node_handle_->getParam("i_3",i);
  ros_node_handle_->getParam("d_3",d);
  ros_node_handle_->getParam("imax_3",imax);
  ros_node_handle_->getParam("imin_3",imin);
  ros_node_handle_->getParam("cmdmax_3",cmdmax);
  ros_node_handle_->getParam("cmdmin_3",cmdmin);
  
    pid_right_qR_3.SetPGain(p);
  pid_right_qR_3.SetIGain(i);
  pid_right_qR_3.SetDGain(d);
  pid_right_qR_3.SetIMax(imax);
  pid_right_qR_3.SetIMin(imin);
  pid_right_qR_3.SetCmdMax(cmdmax);
  pid_right_qR_3.SetCmdMin(cmdmin);

  ros_node_handle_->getParam("p_4",p);
  ros_node_handle_->getParam("i_4",i);
  ros_node_handle_->getParam("d_4",d);
  ros_node_handle_->getParam("imax_4",imax);
  ros_node_handle_->getParam("imin_4",imin);
  ros_node_handle_->getParam("cmdmax_4",cmdmax);
  ros_node_handle_->getParam("cmdmin_4",cmdmin);

  pid_right_qR_4.SetPGain(p);
  pid_right_qR_4.SetIGain(i);
  pid_right_qR_4.SetDGain(d);
  pid_right_qR_4.SetIMax(imax);
  pid_right_qR_4.SetIMin(imin);
  pid_right_qR_4.SetCmdMax(cmdmax);
  pid_right_qR_4.SetCmdMin(cmdmin);

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();

  CalculateJointTorque();

  PublishJointTorque();

}

void GazeboArmActuatorInterface::CalculateJointTorque(){

  Eigen::VectorXd tau_motor_right = Eigen::VectorXd::Zero(6);
  
  Eigen::VectorXd tau_elastic_right = Eigen::VectorXd::Zero(5);
  //update joint states from gazebo
  Eigen::VectorXd qF_fb_right = Eigen::VectorXd::Zero(8);
  Eigen::VectorXd qR_fb_right = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd motor_position_fb_right = Eigen::VectorXd::Zero(6);

  for(int i=0;i<joint_right_.size();i++){
    if (joint_right_.at(i)->Position() != ignition::math::NAN_D){
      qR_fb_right(i)=joint_right_.at(i)->Position();
    }else{
      gzthrow("[gazebo_arm_model] Couldn't get joint position \"" << i
                                                               << "\".");
    };                                                 
  }

  // gzdbg << "qR_fb_right" << ":"<< qR_fb_right(0)<< "," << qR_fb_right(1)<< "," << qR_fb_right(2)<< "," << qR_fb_right(3) << ","
  // << qR_fb_right(4) << std::endl;

/************************************************************************/
  // motor_position_fb_right = Transform_qM2qR * qR_fb_right;

  // //calculate motor position errors
  // Eigen::VectorXd err_motor_right =  motor_position_fb_right - motor_position_input_right;
  
  // gzdbg << "err_motor_right" << ":"<< err_motor_right(0)<< "," << err_motor_right(1)<< "," << err_motor_right(2)<< "," << err_motor_right(3) << ","
  // << err_motor_right(4)<< "," << err_motor_right(5)<< std::endl;


  // Eigen::VectorXd pid_right=Eigen::VectorXd::Zero(6);

  // pid_right(0)=pid_right_0.Update(err_motor_right(0), sampling_time_);
  // pid_right(1)=pid_right_1.Update(err_motor_right(1), sampling_time_);
  // pid_right(2)=pid_right_2.Update(err_motor_right(2), sampling_time_);
  // pid_right(3)=pid_right_3.Update(err_motor_right(3), sampling_time_);
  // pid_right(4)=pid_right_4.Update(err_motor_right(4), sampling_time_);
  // pid_right(5)=pid_right_5.Update(err_motor_right(5), sampling_time_);
  // gzdbg << "pid_right " << ":"<< pid_right(0)<< "," << pid_right(1)<< "," << pid_right(2)<< "," << pid_right(3) << ","
  // << pid_right(4)<< "," << pid_right(5) << std::endl;
  // for (int i = 0; i < pid_right.size(); i++){
  //   tau_motor_right(i) = MAX_TORQUE(i)*pid_right(i);
  // }

  // // for (int i = 0; i < pids_right_.size(); i++){
  // //   tau_motor_right(i) = MAX_TORQUE(i)*pids_right_.at(i)->Update(err_right(i), sampling_time_);
  // // }
  // tau_elastic_right(3)=-0.024*(5*qR_fb_right(3)/M_PI_2);

  // //from motor torque to fake joint torque
  // tau_qR_right=Transform_qM2qR.transpose()* Transform_tauM2tauMR *tau_motor_right+tau_elastic_right;

/************************************************************************/

  Eigen::VectorXd qR_input_right = Transform_qR2qM * motor_position_input_right;
  // gzdbg << "qR_input_right" << ":"<< qR_input_right(0)<< "," << qR_input_right(1)<< "," << qR_input_right(2)<< "," << qR_input_right(3) << ","
  // << qR_input_right(4)<< std::endl;

  //calculate motor position errors
  Eigen::VectorXd err_qR_right =  qR_fb_right - qR_input_right;
  
  gzdbg << "err_qR_right" << ":"<< err_qR_right(0)<< ","<< std::endl << err_qR_right(1)<< ","<< std::endl << err_qR_right(2)<< "," << std::endl << err_qR_right(3) << ","
  << std::endl << err_qR_right(4)<< std::endl;


  Eigen::VectorXd pid_right=Eigen::VectorXd::Zero(5);

  pid_right(0)=pid_right_qR_0.Update(err_qR_right(0), sampling_time_);
  pid_right(1)=pid_right_qR_1.Update(err_qR_right(1), sampling_time_);
  pid_right(2)=pid_right_qR_2.Update(err_qR_right(2), sampling_time_);
  pid_right(3)=pid_right_qR_3.Update(err_qR_right(3), sampling_time_);
  pid_right(4)=pid_right_qR_4.Update(err_qR_right(4), sampling_time_);
  gzdbg <<"pid_right " << ":"<< pid_right(0)<< "," << pid_right(1)<< "," << pid_right(2)<< "," << pid_right(3) << ","
  << pid_right(4) << std::endl;
  for (int i = 0; i < pid_right.size(); i++){
    tau_qR_right(i) = NOMINAL_TORQUE_QR(i)*pid_right(i);
  }

  tau_elastic_right(3)= 0;//-0.024*(5*qR_fb_right(3)/M_PI_2);

  //from motor torque to fake joint torque
  tau_qR_right=tau_qR_right+tau_elastic_right;
  
/*=================================================================*/

  // Eigen::VectorXd tau_motor_left = Eigen::VectorXd::Zero(6);
  
  // Eigen::VectorXd tau_elastic_left = Eigen::VectorXd::Zero(5);
  // //update joint states from gazebo
  // Eigen::VectorXd qF_fb_left = Eigen::VectorXd::Zero(8);
  // Eigen::VectorXd qR_fb_left = Eigen::VectorXd::Zero(5);
  // Eigen::VectorXd motor_position_fb_left = Eigen::VectorXd::Zero(6);

  // for(int i=0;i<joint_left_.size();i++){
  //   if (joint_left_.at(i)->Position() != ignition::math::NAN_D){
  //     qF_fb_left(i)=joint_left_.at(i)->Position();
  //   }else{
  //     gzthrow("[gazebo_arm_model] Couldn't get joint position \"" << i
  //                                                              << "\".");
  //   };                                                 
  // }
  // qR_fb_left = Transform_qR2qF * qF_fb_left;
  // motor_position_fb_left = Transform_qM2qR * qR_fb_left;

  // //calculate motor position errors
  // Eigen::VectorXd err_left = motor_position_fb_left - motor_position_input_left;
  
  // Eigen::VectorXd pid_left=Eigen::VectorXd::Zero(6);

  // pid_left(0)=pid_left_0.Update(err_left(0), sampling_time_);
  // pid_left(1)=pid_left_1.Update(err_left(1), sampling_time_);
  // pid_left(2)=pid_left_2.Update(err_left(2), sampling_time_);
  // pid_left(3)=pid_left_3.Update(err_left(3), sampling_time_);
  // pid_left(4)=pid_left_4.Update(err_left(4), sampling_time_);
  // pid_left(5)=pid_left_5.Update(err_left(5), sampling_time_);

  // for (int i = 0; i < pid_left.size(); i++){
  //   tau_motor_left(i) = MAX_TORQUE(i)*pid_left(i);
  // }

  // tau_elastic_left(3)=-0.024*(5*qR_fb_right(3)/M_PI_2);
  // // for (int i = 0; i < pids_left_.size(); i++){
  // //   tau_motor_left(i) = MAX_TORQUE(i)*pids_left_.at(i)->Update(err_left(i), sampling_time_);
  // // }
  // //from motor torque to fake joint torque
  // tau_qR_left=Transform_qM2qR.transpose()* Transform_tauM2tauMR *tau_motor_left+tau_elastic_left;
  // tau_qF_left=Transform_qR2qF.transpose()* tau_qR_left; 

  // //tau_qF_left += Matrix_physicalLimit * qF_fb_left; //+physical limit

}

void GazeboArmActuatorInterface::PublishJointTorque(){
  // gzdbg << "link_right_torque" << ":"<< tau_qR_right(0)<< "," << tau_qR_right(1)<< "," << tau_qR_right(2)<< "," << tau_qR_right(3) << ","
  // << tau_qR_right(4) << std::endl;

  
  link_right_.at(0)->AddRelativeTorque(ignition::math::Vector3d (0, 0, tau_qR_right(0)));
  link_right_.at(4)->AddRelativeTorque(ignition::math::Vector3d (0, 0, tau_qR_right(4)));
  link_right_.at(1)->AddRelativeTorque(ignition::math::Vector3d (tau_qR_right(1), 0, 0));
  link_right_.at(2)->AddRelativeTorque(ignition::math::Vector3d (tau_qR_right(2), 0, 0));

  link_right_.at(3)->AddRelativeTorque(ignition::math::Vector3d (tau_qR_right(3), 0, 0));

  for(int i=0;i<link_right_.size();i++){

   //link_right_.at(i)->AddRelativeTorque((i==0||i==4)?(ignition::math::Vector3d (0, 0, tau_qR_right(i))):(ignition::math::Vector3d (tau_qF_right(i), 0, 0)));
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