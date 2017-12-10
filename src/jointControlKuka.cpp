/*
 * jointControlKuka.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: michael
 */
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

KDL::Chain LWR() {

  KDL::Chain chain;

  //base
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::None),
                   KDL::Frame::DH_Craig1989(0, 0, 0.33989, 0)));

  //joint 1
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));

  //joint 2
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40011, 0)));

  //joint 3
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));

  //joint 4
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0.40003, 0)));

  //joint 5
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));

  //joint 6
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));

  //joint 7 (with flange adapter)
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
  //KDL::Frame::DH_Craig1989(0,0,0.098,0)));
//    KDL::Frame::DH_Craig1989(0,0,0.088,0))); //AS
      KDL::Frame::DH_Craig1989(0, 0, 0.12597, 0)));

  return chain;

}
//reading the kuka lwr joint positions
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

// initialize the joint positions with a non-zero value to be used in the solvers
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init) {
  for (int i = 0; i < _nj; ++i)
    _jointpositions(i) = _init;
}

// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj,
                       float _init) {
  for (int i = 0; i < _nj; ++i)
    _pt.positions.push_back(_init);
}

//defines the joint names for the robot (used in the jointTrajectory messages)
void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj) {
  for (int i = 1; i <= _nj; ++i) {
    std::ostringstream joint_name;
    joint_name << "iiwa_joint_";
    joint_name << i;
    _cmd.joint_names.push_back(joint_name.str());
  }
}

// loads the joint space points to be sent as a command to the robot
void eval_points(trajectory_msgs::JointTrajectoryPoint & _point,
                 KDL::JntArray & _jointpositions, int _nj) {
  for (int i = 0; i < _nj; ++i)
    _point.positions[i] = _jointpositions(i);

}

// read the reference trajectory from the reflexxes node e.g. ref xyz-rpy
bool ref_received = false;
geometry_msgs::Twist ref;
void get_ref(const geometry_msgs::Twist & data) {
  ref = data;
  ref_received = true;
}

int main(int argc, char * argv[]) {

  KDL::Chain chain = LWR();
  // define the forward kinematic solver via the defined chain
  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(
      chain);
  // define the inverse kinematics solver
  KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);  //Inverse velocity solver
  KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-6);  //Maximum 100 iterations, stop at accuracy 1e-6

  // get the number of joints from the chain
  unsigned int nj = chain.getNrOfJoints();
  // define a joint array in KDL format for the joint positions
  KDL::JntArray jointpositions = KDL::JntArray(nj);
  // define a joint array in KDL format for the next joint positions
  KDL::JntArray jointpositions_new = KDL::JntArray(nj);
  // define a manual joint command array for debugging
  KDL::JntArray manual_joint_cmd = KDL::JntArray(nj);

  ros::init(argc, argv, "joint");
  ros::NodeHandle nh_;
  ros::NodeHandle home("~");
  trajectory_msgs::JointTrajectory joint_cmd;
  trajectory_msgs::JointTrajectoryPoint pt;

  initialize_points(pt, nj, 0.0);

  ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(
      "iiwa/PositionJointInterface_trajectory_controller/command", 10);

  ros::Subscriber joints_sub = nh_.subscribe("/iiwa/joint_states", 10,
                                             get_joints);

  ros::Subscriber ref_sub = nh_.subscribe("/reftraj", 10, get_ref);

  int loop_freq = 10;
  float dt = (float) 1 / loop_freq;
  ros::Rate loop_rate(loop_freq);

  double roll, pitch, yaw, x, y, z;
  home.getParam("roll", roll);
  home.getParam("pitch", pitch);
  home.getParam("yaw", yaw);
  home.getParam("x", x);
  home.getParam("y", y);
  home.getParam("z", z);
  pt.time_from_start = ros::Duration(1.0);

  name_joints(joint_cmd, nj);
  initialize_joints(jointpositions, nj, 0.2);
  KDL::Frame cartpos;
  KDL::Rotation rpy = KDL::Rotation::RPY(roll, pitch, yaw);  //Rotation built from Roll-Pitch-Yaw angles
  cartpos.p[0] = x;
  cartpos.p[1] = y;
  cartpos.p[2] = z;
  cartpos.M = rpy;
/*
    if (initialized) {
      for (int k = 0; k < 7; ++k) {
        jointpositions(k) = joints.position[k];
      }
*/
    int ret = iksolver.CartToJnt(jointpositions, cartpos, jointpositions_new);
    eval_points(pt, jointpositions_new, nj);
    pt.time_from_start = ros::Duration(dt);
    joint_cmd.points.push_back(pt);
//    }
    joint_cmd.header.stamp = ros::Time::now();
    cmd_pub.publish(joint_cmd);



  return 0;

}

