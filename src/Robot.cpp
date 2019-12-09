/**
 * BSD 3-Clause License
 * @copyright (c) 2019, Sandeep Kota and Satyarth Praveen
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @file    Robot.cpp
 * @author  Sandeep Kota and Satyarth Praveen
 * @version 1.0
 * @brief Publisher node
 * @section DESCRIPTION
 * C++ Program regarding the properties and functions of robot.
 */


#include <Robot.h>


Robot::Robot() {
	// tell the action client that we want to spin a thread by default
	ac = MoveBaseClient("move_base", true);

	// wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	// we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();


	/// Saving the current pose
	ros::NodeHandler nh;
	robot_pose_subscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &Robot::saveRobotCurrentPoseCallback, this);
}


Robot::~Robot() {}


void Robot::saveRobotCurrentPoseCallback(nav_msgs::OdometryConstPtr &odom_msg) {
	current_pose = odom_msg->pose.pose;
}


void Robot::setPose(geometry_msgs::Pose &goal_pose) {
	goal.target_pose.pose = goal_pose;
}


geometry_msgs::Pose Robot::getCurrentPose() {
	return current_pose;
}


bool Robot::moveRobotToGoal() {
	ROS_INFO("Robot moving to goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Hooray, the base is moving towards goal.");
		return true;
	} else {
		ROS_INFO("The base failed to move towards goal.");
		return false;
	}
}
