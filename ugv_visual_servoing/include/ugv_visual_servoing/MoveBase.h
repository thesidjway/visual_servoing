/*
  Optimal Visual Servoing
  Copyright (C) 2018  Siddharth Jha, Aashay Bhise

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>



void sendGoal() {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("ovs_move_base", true);
  move_base_msgs::MoveBaseGoal mbgoal;
  geometry_msgs::Point pt;
  pt.x = 0.643;
  pt.y = 4.72;
  pt.z = 0;
  geometry_msgs::Quaternion q;
  q.x =0.000;
  q.y = 0.000;
  q.z = 0.223;
  q.w = 0.975;
  geometry_msgs::Pose ps;
  ps.orientation = q;
  ps.position = pt;
  mbgoal.target_pose.pose = ps;
  mbgoal.target_pose.header.frame_id = "map";
  mbgoal.target_pose.header.stamp = ros::Time::now();
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();
  ac.sendGoal(mbgoal);
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
}
