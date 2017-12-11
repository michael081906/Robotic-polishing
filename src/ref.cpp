/*
 * ref.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: michael
 */
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char * argv[])
{

    ros::init(argc,argv,"ref");
    ros::NodeHandle nh_;
    int loop_freq = 10;
      float dt = (float) 1/loop_freq;
      ros::Rate loop_rate(loop_freq);
      ros::Publisher reflexxes_pub = nh_.advertise<geometry_msgs::Twist>("/reftraj",1);
      geometry_msgs::Twist ref;
      while(ros::ok()){

      ref.linear.x = 0.4;
      ref.linear.y = 0;
      ref.linear.z = 0.4;
      ref.angular.x = 3.14;
      ref.angular.y = 0;
      ref.angular.z = -3.14;
      reflexxes_pub.publish(ref);
      }



    return 0;
}


