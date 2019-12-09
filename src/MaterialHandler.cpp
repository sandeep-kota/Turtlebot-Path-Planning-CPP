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
 * @file    MaterialHandler.cpp
 * @author  Sandeep Kota and Satyarth Praveen
 * @version 1.0
 * @brief Publisher node
 * @section DESCRIPTION
 * C++ Program regarding the properties and functions of the Material Handling system.
 */


#include <MaterialHandler.h>
#include <MapClass.h>
#include <Robot.h>

/**
 * @brief      Constructs a new instance.
 * 
 * @param      none
 * @return     none
 */
MaterialHandler::MaterialHandler() {
	/// Reading parameters from the launch file
	node_handle.getParam("/pick_model_name", pick_model_name); 
	node_handle.getParam("/place_model_name", place_model_name);

	ros::Rate loop_rate(30);
	while(ros::ok()) {
		ros::SpinOnce();
		loop_rate.sleep();
	}
}



std::string MaterialHandler::getDeleteModel() {
	return delete_model_name;
}


void MaterialHandler::performTask() {
	pick();
	place();
}


void MaterialHandler::pick() {
	/// Make the robot move places for the task
	std::vector<std::string>::iterator it = std::find(map_models.begin(), map_models.end(), pick_model_name);
	if (it != map_models.end()) {
		int index = std::distance(map_models.begin(), it);
		goal_pose = map_models.pose[index];
	}

	robot_obj.setPose(goal_pose);
	// robot_obj = Robot(goal_pose);
	bool successful = robot_obj.moveRobotToGoal();
	if (successful) {
		std::sstream delete_cmd;
		delete_cmd << "gz model -d -m " << pick_model_name;
		std::system(delete_cmd.str());

		geometry_msgs::Pose curr_pose = robot_obj.getCurrentPose();
		std::sstream spawn_cmd;
		spawn_cmd << "gz model -m " << pick_model_name << " -f data/models/red_box/model.sdf -x " << curr_pose.position.x << " -y " << curr_pose.position.y << " -z " << curr_pose.position.z << " -R 0 -P 0 -Y 0";
		std::system(spawn_cmd.str());
		std::terminate();
	}
}


void MaterialHandler::place() {
	/// Make the robot move places for the task
	std::vector<std::string>::iterator it = std::find(map_models.begin(), map_models.end(), place_model_name);
	if (it != map_models.end()) {
		int index = std::distance(map_models.begin(), it);
		goal_pose = map_models.pose[index];
	}

	robot_obj.setPose(goal_pose);
	bool successful = robot_obj.moveRobotToGoal();
	if (successful) {
		std::sstream delete_cmd;
		delete_cmd << "gz model -d -m " << pick_model_name;
		std::system(delete_cmd.str());

		std::sstream spawn_cmd;
		spawn_cmd << "gz model -m " << pick_model_name << " -f data/models/red_box/model.sdf -x " << goal_pose.position.x << " -y " << goal_pose.position.y << " -z " << goal_pose.position.z << " -R 0 -P 0 -Y 0";
		std::system(spawn_cmd.str());
		std::terminate();
	}
}
