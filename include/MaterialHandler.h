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
 * @section DESCRIPTION
 * C++ Program regarding the properties and functions of the Material Handling system.
 */

#ifndef MATERIALHANDLER_H
#define MATERIALHANDLER_H

#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <MapClass.h>
#include <Robot.h>
#include <geometry_msgs/Pose.h>


/**
 * @brief      MaterialHandler Class consists of MaterialHandler properties and member functions..
 */
class MaterialHandler : public MapClass{
private: 
  ros::NodeHandle nh;
  std::string pick_model_name;
  std::string place_model_name;

  geometry_msgs::Pose goal_pose;

  Robot robot_obj;

public:

  /**
   * @brief      Constructs a new instance.
   * 
   * @param      none
   * @return     none
   */
  MaterialHandler();
  ~MaterialHandler();

  /**
   * @brief      Gets the model to be deleted.
   *
   * @param      none
   * @return     The model name.
   */
  std::string getDeleteModel();

  /**
   * @brief      Performs the task of pick and place.
   * 
   * @param      none
   * @return     The model name.
   */
  void performTask();

  /**
   * @brief      Picks the object 
   * 
   * @param      none
   * @return     none
   */
  void pick();

  /**
   * @brief      Places the object at the destination
   * 
   * @param      none
   * @return     none
   */
  void place();

};

#endif
