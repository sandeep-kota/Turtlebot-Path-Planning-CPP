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
 * @section DESCRIPTION
 * C++ Program regarding the properties and functions of the MapClass.
 */


#ifndef MAPCLASS_H
#define MAPCLASS_H

#include <iostream>
#include <ros/ros.h>

/**
 * @brief      This class describes a map class.
 */
class MapClass {
private:
  gazebo::ModelStates map_models;

  ros::Subscriber map_subscriber;
  ros::Subscriber model_subscriber;

  nav_msgs::OccupancyGrid saved_map;

public:
  /**
   * @brief      Constructs a new instance.
   * 
   * @param      none
   * @return     none
   */
  MapClass();
  
  /**
   * @brief      Destroys the object.
   * 
   * @param      none
   * @return     none
   */
  ~MapClass();

  /**
   * @brief      Callback function to save a map.
   *
   * @param      map_msg  The map message
   * @return     none
   */
  void saveMapCallback(nav_msgs::OccupancyGrid &map_msg);

  /**
   * @brief      Gets the models callback.
   *
   * @param      models_msg  The models message
   * @return     none
   */
  void getModelsCallback(gazebo::ModelStatesConstPtr &models_msg);
};

#endif
