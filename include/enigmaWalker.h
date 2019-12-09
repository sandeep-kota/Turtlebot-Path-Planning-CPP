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
 * @file    enigmaWalker.cpp
 * @author  Sandeep Kota and Satyarth Praveen
 * @version 1.0
 * @brief   This file implements the enigmaWalker.
 */

#ifndef ENIGMAWALKER_H
#define ENIGMAWALKER_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

/**
 * @brief      This class describes a enigmaWalker.
 */
class enigmaWalker {
private:
  static const int LEFT = 0;
  static const int RIGHT = 1;
  static const int STRAIGHT = 2;
  static constexpr float CLOSEST_DISTANCE = 0.6; // distance in meters.

  ros::NodeHandle node_handle;
  ros::Subscriber laser_subscriber;
  ros::Publisher nav_publisher;

public:
  geometry_msgs::Twist out_msg;

  /**
   * @brief      Constructs a new instance.
   * 
   * @param none
   * @return none
   */
  enigmaWalker();
  /**
   * @brief      Destroys the object.
   * 
   * @param none
   * @return none
   */
  ~enigmaWalker();

  /**
   * @brief      rotates the robot in-place.
   *
   * @param      direction  The direction in which the robot is to be rotated.
   * @return     none
   */
  void moveBot(int direction);

  /**
   * @brief      processes the scene to decide where the to keep moving forward or take a turn.
   *
   * @param      msg   The subscribed message
   * @return     none
   */
  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg);
};

#endif
