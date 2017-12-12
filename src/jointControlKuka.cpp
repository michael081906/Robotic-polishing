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
  for (int i = 0; i < data.position.size(); i++) {
    // if this is not the first time the callback function is read, obtain the joint positions
    if (initialized) {
      joints.position[i] = data.position[i];
      // otherwise initilize them with 0.0 values
    } else {
      joints.position.push_back(0.0);
    }
  }
  initialized = true;
  /* ROS_INFO("data.position[0]= %f", data.position[0]);
   ROS_INFO("data.position[1]= %f", data.position[1]);
   ROS_INFO("data.position[2]= %f", data.position[2]);
   ROS_INFO("data.position[3]= %f", data.position[3]);
   ROS_INFO("data.position[4]= %f", data.position[4]);
   ROS_INFO("data.position[5]= %f", data.position[5]);
   ROS_INFO("data.position[6]= %f", data.position[6]);

   ROS_INFO("joints.position[0]= %f", joints.position[0]);
   ROS_INFO("joints.position[1]= %f", joints.position[1]);
   ROS_INFO("joints.position[2]= %f", joints.position[2]);
   ROS_INFO("joints.position[3]= %f", joints.position[3]);
   ROS_INFO("joints.position[4]= %f", joints.position[4]);
   ROS_INFO("joints.position[5]= %f", joints.position[5]);
   ROS_INFO("joints.position[6]= %f\n", joints.position[6]);
   */

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
double threshold = 0.01;
bool checkIfArrived(const KDL::JntArray& joint_ref) {
  double d0, d1, d2, d3, d4, d5, d6;
  double d01, d02, d03, d04, d05, d06, d07;
  d0 = joint_ref(0) - joints.position[0];
  d01 = fabs(d0);

  d1 = joint_ref(1) - joints.position[1];
  d02 = fabs(d1);

  d2 = joint_ref(2) - joints.position[2];
  d03 = fabs(d2);

  d3 = joint_ref(3) - joints.position[3];
  d04 = fabs(d3);

  d4 = joint_ref(4) - joints.position[4];
  d05 = fabs(d4);

  d5 = joint_ref(5) - joints.position[5];
  d06 = fabs(d5);

  d6 = joint_ref(6) - joints.position[6];
  d07 = fabs(d6);
  /* ROS_INFO("joints.position[0]= %f", joints.position[0]);
   ROS_INFO("joints.position[1]= %f", joints.position[1]);
   ROS_INFO("joints.position[2]= %f", joints.position[2]);
   ROS_INFO("joints.position[3]= %f", joints.position[3]);
   ROS_INFO("joints.position[4]= %f", joints.position[4]);
   ROS_INFO("joints.position[5]= %f", joints.position[5]);
   ROS_INFO("joints.position[6]= %f", joints.position[6]);


   ROS_INFO("d0= %f", d01);
   ROS_INFO("d1= %f", d02);
   ROS_INFO("d2= %f", d03);
   ROS_INFO("d3= %f", d04);
   ROS_INFO("d4= %f", d05);
   ROS_INFO("d5= %f", d06);
   ROS_INFO("d6= %f", d07);*/
  if (d01 < threshold && d02 < threshold && d03 < threshold && d04 < threshold
      && d05 < threshold && d06 < threshold && d07 < threshold) {
    return true;
  }
  return false;
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
  /*home.getParam("roll", roll);
   home.getParam("pitch", pitch);
   home.getParam("yaw", yaw);
   home.getParam("x", x);
   home.getParam("y", y);
   home.getParam("z", z);*/
  x = 0.6;
  y = -0.2;
  z = 0.2;
  roll = 0;
  pitch = 1.57;
  yaw = 0;

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

  ROS_INFO("command show");
  ROS_INFO("FK_Rz= %f", test_ref[0].x);
  ROS_INFO("FK_Rz= %f", test_ref[0].y);
  ROS_INFO("FK_Rz= %f", test_ref[0].z);

  ROS_INFO("FK_Rz= %f", test_ref[1].x);
  ROS_INFO("FK_Rz= %f", test_ref[1].y);
  ROS_INFO("FK_Rz= %f", test_ref[1].z);

  ROS_INFO("FK_Rz= %f", test_ref[2].x);
  ROS_INFO("FK_Rz= %f", test_ref[2].y);
  ROS_INFO("FK_Rz= %f", test_ref[2].z);

  ROS_INFO("FK_Rz= %f", test_ref[3].x);
  ROS_INFO("FK_Rz= %f", test_ref[3].y);
  ROS_INFO("FK_Rz= %f", test_ref[3].z);

  ROS_INFO("FK_Rz= %f", test_ref[4].x);
  ROS_INFO("FK_Rz= %f", test_ref[4].y);
  ROS_INFO("FK_Rz= %f", test_ref[4].z);

  ROS_INFO("command show end ");

  joints.position.push_back(0.0);
  // ROS_INFO("joints.position[0]= %f\n", joints.position[0]);
  joint_ref = jointpositions_new;
  ROS_INFO("joint_ref(0)= %f", joint_ref(0));
  ROS_INFO("joint_ref(1)= %f", joint_ref(1));
  ROS_INFO("joint_ref(2)= %f", joint_ref(2));
  ROS_INFO("joint_ref(3)= %f", joint_ref(3));
  ROS_INFO("joint_ref(4)= %f", joint_ref(4));
  ROS_INFO("joint_ref(5)= %f", joint_ref(5));
  ROS_INFO("joint_ref(6)= %f", joint_ref(6));
  std::vector<position>::iterator now_cmd = test_ref.begin();
  bool arrived = false;

  double d0, d1, d2, d3, d4, d5, d6;
  double d01, d02, d03, d04, d05, d06, d07;

  while (ros::ok()) {

    if (now_cmd != test_ref.end()) { // 2017.12.12 Can't add while inside the while
     // if joint error<0.001
      ROS_INFO("joints.position[0]= %f", joints.position[0]);
       ROS_INFO("joints.position[1]= %f", joints.position[1]);
       ROS_INFO("joints.position[2]= %f", joints.position[2]);
       ROS_INFO("joints.position[3]= %f", joints.position[3]);
       ROS_INFO("joints.position[4]= %f", joints.position[4]);
       ROS_INFO("joints.position[5]= %f", joints.position[5]);
       ROS_INFO("joints.position[6]= %f\n", joints.position[6]);

       d0 = joint_ref(0) - joints.position[0];
       d01 = fabs(d0);

       d1 = joint_ref(1) - joints.position[1];
       d02 = fabs(d1);

       d2 = joint_ref(2) - joints.position[2];
       d03 = fabs(d2);

       d3 = joint_ref(3) - joints.position[3];
       d04 = fabs(d3);

       d4 = joint_ref(4) - joints.position[4];
       d05 = fabs(d4);

       d5 = joint_ref(5) - joints.position[5];
       d06 = fabs(d5);

       d6 = joint_ref(6) - joints.position[6];
       d07 = fabs(d6);

       ROS_INFO("d0= %f", d01);
       ROS_INFO("d1= %f", d02);
       ROS_INFO("d2= %f", d03);
       ROS_INFO("d3= %f", d04);
       ROS_INFO("d4= %f", d05);
       ROS_INFO("d5= %f", d06);
       ROS_INFO("d6= %f", d07);

       if (d01 < threshold && d02 < threshold && d03 < threshold && d04 < threshold
           && d05 < threshold && d06 < threshold && d07 < threshold) {
         arrived = true;
       } else {
         arrived = false;
       }
     if (arrived) {  //{ take out element
     cartpos.p[0] = now_cmd->x;
     cartpos.p[1] = now_cmd->y;
     cartpos.p[2] = now_cmd->z;
     ROS_INFO("cartpos.p[0]= %f", cartpos.p[0]);
     ROS_INFO("cartpos.p[1]= %f", cartpos.p[1]);
     ROS_INFO("cartpos.p[2]= %f", cartpos.p[2]);
     cartpos.M = rpy;
     for (int k = 0; k < nj; k++) {
     jointpositions(k) = joints.position[k];
     }
     // IK }
     int ik_error = iksolver.CartToJnt(jointpositions, cartpos,
     jointpositions_new);

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

     pt.time_from_start = ros::Duration(1.0);
     kc.eval_points(pt, jointpositions_new, nj);
     joint_cmd.points[0] = pt;
     joint_ref = jointpositions_new;
     ++now_cmd;
     }
     // send command
    //  cmd_pub.publish(joint_cmd);

     }
    cmd_pub.publish(joint_cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;

}

