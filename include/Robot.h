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
 * @section DESCRIPTION
 * C++ Program regarding the properties and functions of robot.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief      Robot Class consists of robot properties and member functions..
 */
class Robot {
private: 
	ros::Subscriber robot_pose_subscriber;

	geometry_msgs::Pose current_pose;

	MoveBaseClient ac;
	move_base_msgs::MoveBaseGoal goal;

public:

	/**
     * @brief      Constructs a new instance.
     * 
     * @param      none
     * @return     none
     */
	Robot();

	/**
     * @brief      Destroys the object.
     * 
     * @ param     none
     * @ return    none
     */
	~Robot();

	/**
     * @brief      Saves the current pose of the robot.
     *
     * @param      odom_msg  The odometry message
     * @return     none
     */
    void saveRobotCurrentPoseCallback(nav_msgs::OdometryConstPtr &odom_msg) ;

    /**
     * @brief      Sets the pose.
     *
     * @param      goal_pose  The goal pose
     * @return     none
     */
    void setPose(geometry_msgs::Pose &goal_pose);

    /**
     * @brief Return true if successfully executed
     *
     * @ param     none
     * @return     returns true if the move was executed successfully
     */
    bool moveRobotToGoal();

    /**
     * @brief      Gets the current pose.
     * 
     * @param      none
     * @return     The current pose.
     */
    geometry_msgs::Pose getCurrentPose();

    /**
     * @brief Return true if successfully executed
     *
     * @ param     none
     * @return     returns true if the move was executed successfully
     */
    bool moveRobotToGoal();

};

#endif
