#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>

#include <tf/transform_broadcaster.h>

#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>

#define ETA                 ((double)(0.048/0.021))

using namespace std;


std::string ns;
ros::Publisher    pub_joint0_left,pub_joint1_left,pub_joint2_left,pub_joint3_left,pub_joint4_left,
                  pub_joint0_right,pub_joint1_right,pub_joint2_right,pub_joint3_right,pub_joint4_right,
                  pub_joint1_neck,pub_joint2_neck ;

void motorPosiCallback_rightArm(const sensor_msgs::JointState& msg){
  Eigen::VectorXd joint_state_cmd;
  joint_state_cmd.resize(5);
  joint_state_cmd(0)=msg.position[0];
  joint_state_cmd(4)=msg.position[5];

  joint_state_cmd(1)=msg.position[1]/ETA;
  joint_state_cmd(2)=msg.position[2]/ETA;
  joint_state_cmd(3)=msg.position[4]/ETA;

  std_msgs::Float64 ref_msg;
  ref_msg.data=joint_state_cmd(0);
  pub_joint0_right.publish(ref_msg);
  ref_msg.data=joint_state_cmd(1);
  pub_joint1_right.publish(ref_msg);
  ref_msg.data=joint_state_cmd(2);
  pub_joint2_right.publish(ref_msg);
  ref_msg.data=joint_state_cmd(3);
  pub_joint3_right.publish(ref_msg);
  ref_msg.data=joint_state_cmd(4);
  pub_joint4_right.publish(ref_msg);      
    
}
void motorPosiCallback_leftArm(const sensor_msgs::JointState& msg){
  Eigen::VectorXd joint_state_cmd;
  joint_state_cmd.resize(5);
  joint_state_cmd(0)=msg.position[0];
  joint_state_cmd(4)=msg.position[5];

  joint_state_cmd(1)=msg.position[1]/ETA;
  joint_state_cmd(2)=msg.position[2]/ETA;
  joint_state_cmd(3)=msg.position[4]/ETA;

  std_msgs::Float64 ref_msg;
  ref_msg.data=joint_state_cmd(0);
  pub_joint0_left.publish(ref_msg);
  ref_msg.data=joint_state_cmd(1);
  pub_joint1_left.publish(ref_msg);
  ref_msg.data=joint_state_cmd(2);
  pub_joint2_left.publish(ref_msg);
  ref_msg.data=joint_state_cmd(3);
  pub_joint3_left.publish(ref_msg);
  ref_msg.data=joint_state_cmd(4);
  pub_joint4_left.publish(ref_msg);   

}
void neckPosiCallback(const sensor_msgs::JointState& msg){
  Eigen::VectorXd joint_state_cmd;
  joint_state_cmd.resize(2);
  joint_state_cmd(0)=msg.position[0];
  joint_state_cmd(1)=msg.position[1];

  std_msgs::Float64 ref_msg;
  ref_msg.data=joint_state_cmd(0);
  pub_joint1_neck.publish(ref_msg);
  ref_msg.data=joint_state_cmd(1);
  pub_joint2_neck.publish(ref_msg);

}

int main(int argc, char **argv)
{

  ros::Subscriber 	subHandler_posi_motor_rightArm,subHandler_posi_motor_leftArm,subHandler_ref_neck;

	// ------------------------------------------------------------------------------------- Init node
  ros::init(argc, argv, "bridge_yigle_rotorS");
  ros::NodeHandle n;
  ns = ros::this_node::getNamespace();
	ns = ns.substr(1,ns.length()-1);



  subHandler_posi_motor_rightArm = n.subscribe("/right_arm/motor_position",1,motorPosiCallback_rightArm);
  subHandler_posi_motor_leftArm = n.subscribe("/left_arm/motor_position",1,motorPosiCallback_leftArm);
//    subHandler_closure_hand_rightArm = nodeHandler.subscribe("/right_arm/hand_closure",1,&QbManager::handClosureCallback_rightArm,this);
//    subHandler_closure_hand_leftArm = nodeHandler.subscribe("/left_arm/hand_closure",1,&QbManager::handClosureCallback_leftArm,this);
  subHandler_ref_neck = n.subscribe("/ref_neck",1,neckPosiCallback);

  pub_joint0_left=n.advertise<std_msgs::Float64>("/yigle/joint0_position_controller_left/command", 1);
  pub_joint1_left=n.advertise<std_msgs::Float64>("/yigle/joint1_position_controller_left/command", 1);
  pub_joint2_left=n.advertise<std_msgs::Float64>("/yigle/joint2_position_controller_left/command", 1);
  pub_joint3_left=n.advertise<std_msgs::Float64>("/yigle/joint3_position_controller_left/command", 1);
  pub_joint4_left=n.advertise<std_msgs::Float64>("/yigle/joint4_position_controller_left/command", 1);

  pub_joint0_right=n.advertise<std_msgs::Float64>("/yigle/joint0_position_controller_right/command", 1);
  pub_joint1_right=n.advertise<std_msgs::Float64>("/yigle/joint1_position_controller_right/command", 1);
  pub_joint2_right=n.advertise<std_msgs::Float64>("/yigle/joint2_position_controller_right/command", 1);
  pub_joint3_right=n.advertise<std_msgs::Float64>("/yigle/joint3_position_controller_right/command", 1);
  pub_joint4_right=n.advertise<std_msgs::Float64>("/yigle/joint4_position_controller_right/command", 1);

  pub_joint1_neck=n.advertise<std_msgs::Float64>("/yigle/joint_position_controller_neck_1/command", 1);
  pub_joint2_neck=n.advertise<std_msgs::Float64>("/yigle/joint_position_controller_neck_2/command", 1);
  

	ros::spin();
  
  return 0;
}