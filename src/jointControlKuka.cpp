/*
 * jointControlKuka.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: michael
 */
#include "kukaControl.h"
#include<ros/ros.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<geometry_msgs/Twist.h>
#include<iiwa_msgs/JointPosition.h>
#include<sensor_msgs/JointState.h>
#include<kdl/chain.hpp>
#include "Eigen/Core"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>
#include <cmath>

sensor_msgs::JointState joints;
bool initialized = false;
//callback for reading joint values
void get_joints(const sensor_msgs::JointState & data) {
  for (int i = 0; i < data.position.size(); ++i) {
    // if this is not the first time the callback function is read, obtain the joint positions
    if (initialized) {
      joints.position[i] = data.position[i];
      // otherwise initilize them with 0.0 values
    } else {
      joints.position.push_back(0.0);
    }
  }
  initialized = true;
}
/*
 // read the reference trajectory from the reflexxes node e.g. ref xyz-rpy
 bool ref_received = false;
 geometry_msgs::Twist ref;
 void get_ref(const geometry_msgs::Twist & data) {
 ref = data;
 ref_received = true;
 }
 */
double threshold=0.001;
bool checkIfArrived(const KDL::JntArray& joint_ref) {
  float d0, d1, d2, d3, d4, d5, d6;
  d0 = joint_ref(0) - joints.position[0];
  d0 = fabs(d0);
  d1 = joint_ref(1) - joints.position[1];
  d1 = fabs(d1);
  d2 = joint_ref(2) - joints.position[2];
  d2 = fabs(d2);
  d3 = joint_ref(3) - joints.position[3];
  d3 = fabs(d3);
  d4 = joint_ref(4) - joints.position[4];
  d4 = fabs(d4);
  d5 = joint_ref(5) - joints.position[5];
  d5 = fabs(d5);
  d6 = joint_ref(6) - joints.position[6];
  d6 = fabs(d6);
  if (d0 < threshold && d1 < threshold && d2 < threshold && d3 < threshold && d4 < threshold
      && d5 < threshold && d6 < threshold) {
    return true;
  }
  return true;
}

struct position {
  float x;
  float y;
  float z;
} test1;

int main(int argc, char * argv[]) {

  kukaControl kc;
  KDL::Chain chain = kc.LWR();
  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(
      chain);  // define the forward kinematic solver via the defined chain
  KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);  //Inverse velocity solver
  KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-6);  //Maximum 100 iterations, stop at accuracy 1e-6
  unsigned int nj = chain.getNrOfJoints();  // get the number of joints from the chain
  KDL::JntArray jointpositions = KDL::JntArray(nj);  // define a joint array in KDL format for the joint positions
  KDL::JntArray jointpositions_new = KDL::JntArray(nj);  // define a joint array in KDL format for the next joint positions
  KDL::JntArray joint_ref = KDL::JntArray(nj);
  KDL::JntArray manual_joint_cmd = KDL::JntArray(nj);  // define a manual joint command array for debugging
  ros::init(argc, argv, "joint");
  ros::NodeHandle nh_;
  ros::NodeHandle home("~");
  trajectory_msgs::JointTrajectory joint_cmd;
  trajectory_msgs::JointTrajectoryPoint pt;
  kc.initialize_points(pt, nj, 0.0);
  ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(
      "iiwa/PositionJointInterface_trajectory_controller/command", 10);
  ros::Subscriber joints_sub = nh_.subscribe("/iiwa/joint_states", 10,
                                             get_joints);
  // ros::Subscriber ref_sub = nh_.subscribe("/reftraj", 10, get_ref);
  int loop_freq = 10;
  float dt = (float) 1 / loop_freq;
  ros::Rate loop_rate(loop_freq);
  double roll, pitch, yaw, x, y, z;
  KDL::Frame cartpos;
  kc.name_joints(joint_cmd, nj);
  kc.initialize_joints(jointpositions, nj, 0.2);
  ROS_INFO("Load current joint configuration");
  ROS_INFO("J1= %f", jointpositions(0));
  ROS_INFO("J2= %f", jointpositions(1));
  ROS_INFO("J3= %f", jointpositions(2));
  ROS_INFO("J4= %f", jointpositions(3));
  ROS_INFO("J5= %f", jointpositions(4));
  ROS_INFO("J6= %f", jointpositions(5));
  ROS_INFO("J7= %f\n", jointpositions(6));
  geometry_msgs::Twist xyz;
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
  ROS_INFO("Get FK result");
  if (kinematics_status >= 0) {
    ROS_INFO("FK_x= %f", cartpos.p[0]);
    ROS_INFO("FK_y= %f", cartpos.p[1]);
    ROS_INFO("FK_z= %f", cartpos.p[2]);
    cartpos.M.GetRPY(roll, pitch, yaw);
    ROS_INFO("FK_Rx= %f", roll);
    ROS_INFO("FK_Ry= %f", pitch);
    ROS_INFO("FK_Rz= %f\n", yaw);
  }

  ROS_INFO("Set command cartpos configuration");
  home.getParam("roll", roll);
  home.getParam("pitch", pitch);
  home.getParam("yaw", yaw);
  home.getParam("x", x);
  home.getParam("y", y);
  home.getParam("z", z);
  ROS_INFO("cmd_x= %f", x);
  ROS_INFO("cmd_y= %f", y);
  ROS_INFO("cmd_z= %f", z);
  ROS_INFO("cmd_rx= %f", roll);
  ROS_INFO("cmd_ry= %f", pitch);
  ROS_INFO("cmd_rz= %f\n", yaw);

  pt.time_from_start = ros::Duration(3.0);
  KDL::Rotation rpy = KDL::Rotation::RPY(roll, pitch, yaw);  //Rotation built from Roll-Pitch-Yaw angles
  cartpos.p[0] = x;
  cartpos.p[1] = y;
  cartpos.p[2] = z;
  cartpos.M = rpy;  // 2017.12.10 Bug here!!!!!!! Causing IK failed...... because you did not initialize it.

  KDL::Twist xyzr;
  ROS_INFO("Set current joint configuration before IK");
  ROS_INFO("J1= %f", jointpositions(0));
  ROS_INFO("J2= %f", jointpositions(1));
  ROS_INFO("J3= %f", jointpositions(2));
  ROS_INFO("J4= %f", jointpositions(3));
  ROS_INFO("J5= %f", jointpositions(4));
  ROS_INFO("J6= %f", jointpositions(5));
  ROS_INFO("J7= %f\n", jointpositions(6));
  int ik_error = iksolver.CartToJnt(jointpositions, cartpos,
                                    jointpositions_new);
  ROS_INFO("ik_error= %d", ik_error);
  kc.eval_points(pt, jointpositions_new, nj);
  pt.time_from_start = ros::Duration(3.0);
  joint_cmd.points.push_back(pt);
  pt.time_from_start = ros::Duration(3.0);
  joint_cmd.points[0] = pt;

  ROS_INFO("Set current joint configuration after IK");
  ROS_INFO("J1= %f", jointpositions_new(0));
  ROS_INFO("J2= %f", jointpositions_new(1));
  ROS_INFO("J3= %f", jointpositions_new(2));
  ROS_INFO("J4= %f", jointpositions_new(3));
  ROS_INFO("J5= %f", jointpositions_new(4));
  ROS_INFO("J6= %f", jointpositions_new(5));
  ROS_INFO("J7= %f\n", jointpositions_new(6));
  joint_cmd.header.stamp = ros::Time::now();

  kinematics_status = fksolver.JntToCart(jointpositions_new, cartpos);
  ROS_INFO("Get FK result");
  if (kinematics_status >= 0) {
    ROS_INFO("FK_x= %f", cartpos.p[0]);
    ROS_INFO("FK_y= %f", cartpos.p[1]);
    ROS_INFO("FK_z= %f", cartpos.p[2]);
    cartpos.M.GetRPY(roll, pitch, yaw);
    ROS_INFO("FK_Rx= %f", roll);
    ROS_INFO("FK_Ry= %f", pitch);
    ROS_INFO("FK_Rz= %f\n", yaw);
  }

  std::vector<position> test_ref;
  float y_test = (-0.2);
  for (int i = 0; i < 5; i++) {
    test1.x = 0.6;
    test1.y = y_test;
    test1.z = 0.2;
    test_ref.push_back(test1);
    y_test = y_test + 0.1;
  }

  joint_ref = jointpositions_new;
  std::vector<position>::iterator now_cmd = test_ref.begin();
  while (ros::ok()) {
    while (now_cmd != test_ref.end()) {
      // if joint error<0.001
      if (checkIfArrived(joint_ref)) {  //{ take out element
        cartpos.p[0] = now_cmd->x;
        cartpos.p[1] = now_cmd->y;
        cartpos.p[2] = now_cmd->z;
        /*for (int k = 0; k < nj; k++) {
          jointpositions(k) = joints.position[k];
        }*/
      }
/*
       // IK }
        int ik_error = iksolver.CartToJnt(jointpositions, cartpos,
                                          jointpositions_new);
        pt.time_from_start = ros::Duration(3.0);
        kc.eval_points(pt, jointpositions_new, nj);
        joint_cmd.points[0] = pt;
        joint_ref = jointpositions_new;
        ++now_cmd;
      }*/
      // send command
      cmd_pub.publish(joint_cmd);
    }
    cmd_pub.publish(joint_cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;

}

