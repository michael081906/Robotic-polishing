/*
 * kukaControl.h
 *
 *  Created on: Dec 11, 2017
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
#ifndef ROBOTIC_POLISHING_INCLUDE_KUKACONTROL_H_
#define ROBOTIC_POLISHING_INCLUDE_KUKACONTROL_H_

class kukaControl{
private:



public:
  kukaControl();
  void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init);
  void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj,float _init);
  void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj);
  void eval_points(trajectory_msgs::JointTrajectoryPoint & _point,
                   KDL::JntArray & _jointpositions, int _nj);

};




#endif /* ROBOTIC_POLISHING_INCLUDE_KUKACONTROL_H_ */
