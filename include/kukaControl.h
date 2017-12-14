// "Copyright [2017] <Michael Kam>"
/** @file kukaControl.h
 *  @brief This kukaControl.h is a header file of controlling the iiwa kuka arm
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  kukaControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  kukaControl is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with kukaControl.  If not, see <http://www.gnu.org/licenses/>.
 *
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
  /**constructor */
  kukaControl();
  /**@brief initialize_joints sets the initial value for the IK solver
   * @param[in] _jointpositions vector that store joint position value
   * @return none     */
  void initialize_joints(KDL::JntArray & _jointpositions);
  /**@brief initialize_points() set the _init into _pt
   * @param[in] _pt trajectory_msgs::JointTrajectoryPoint
   * @param[in] _nj number of joint of the robot
   * @param[in] _init initial value want to set
   * @return none     */
  void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj,float _init);
  /**@brief name_joints() name the joint
   * @param[in] _cmd stores the command message to control iiwa
   * @param[in] _nj number of joint of the robot
   * @return none     */
  void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj);
  /**@brief eval_points set the _jointpositions value into _point
   * @param[in] _point stores the command message to control iiwa
   * @param[in] _jointpositions vector that store joint position value
   * @param[in] _nj number of joint of the robot
   * @return none     */
  void eval_points(trajectory_msgs::JointTrajectoryPoint & _point,
                   KDL::JntArray & _jointpositions, int _nj);
  /**@brief the function defined the kinematic chain of the iiwa robot
   * @param[in] noe
   * @return KDL::Chain     */
  KDL::Chain LWR();
};




#endif /* ROBOTIC_POLISHING_INCLUDE_KUKACONTROL_H_ */
