/*
 * kukaControl.cpp
 *
 *  Created on: Dec 11, 2017
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

kukaControl::kukaControl() {
}

// initialize the joint positions with a non-zero value to be used in the solvers
void kukaControl::initialize_joints(KDL::JntArray & _jointpositions, int _nj,
                                    float _init) {
  /*for (int i = 0; i < _nj; ++i)
   _jointpositions(i) = _init;*/
  _jointpositions(0) = 0;
  _jointpositions(1) = 0;
  _jointpositions(2) = 0;
  _jointpositions(3) = -1.57;
  _jointpositions(4) = 0;
  _jointpositions(5) = 1.57;
  _jointpositions(6) = 0;

}

// initialize a joint command point
void kukaControl::initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt,
                                    int _nj, float _init) {
  for (int i = 0; i < _nj; ++i)
    _pt.positions.push_back(_init);
}

//defines the joint names for the robot (used in the jointTrajectory messages)
void kukaControl::name_joints(trajectory_msgs::JointTrajectory & _cmd,
                              int _nj) {
  for (int i = 1; i <= _nj; ++i) {
    std::ostringstream joint_name;
    joint_name << "iiwa_joint_";
    joint_name << i;
    _cmd.joint_names.push_back(joint_name.str());
  }
}

// loads the joint space points to be sent as a command to the robot
void kukaControl::eval_points(trajectory_msgs::JointTrajectoryPoint & _point,
                              KDL::JntArray & _jointpositions, int _nj) {
  for (int i = 0; i < _nj; ++i)
    _point.positions[i] = _jointpositions(i);

}

KDL::Chain kukaControl::LWR() {

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




