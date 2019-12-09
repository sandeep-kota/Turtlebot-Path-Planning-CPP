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
 * @file    MapClass.cpp
 * @author  Sandeep Kota and Satyarth Praveen
 * @version 1.0
 * @brief Publisher node
 * @section DESCRIPTION
 * C++ Program regarding the properties and functions of the MapClass.
 */


#include <MapClass.h>


MapClass::MapClass() {

	ros::NodeHandler nh;

	/// Subscribers
	map_subscriber = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 5, &MapClass::saveMapCallback, this);
	model_subscriber = nh.subscribe<gazebo::ModelStates>("/gazebo/model_states", 10, &MapClass::getModelsCallback, this);
}

MapClass::~MapClass() {}


void MapClass::saveMapCallback(nav_msgs::OccupancyGrid &map_msg) {
	saved_map.header = map_msg->header;
	saved_map.info = map_msg->info;
	saved_map.data = map_msg->data;
}


void MapClass::getModelsCallback(gazebo::ModelStatesConstPtr &models_msg) {
	map_models.name = models_msg->name;
	map_models.pose = models_msg->pose;
	map_models.twist = models_msg->twist;
}