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


#include <enigmaWalker.h>

enigmaWalker::enigmaWalker() {
  ros::Rate loop_rate(30);

  laser_subscriber = node_handle.subscribe<sensor_msgs::LaserScan>(
    "/scan", 2, &enigmaWalker::scanCallback, this);

  nav_publisher = node_handle.advertise<geometry_msgs::Twist>(
    "/cmd_vel_mux/input/navi", 2);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

enigmaWalker::~enigmaWalker() {}

void enigmaWalker::moveBot(int direction) {
  out_msg.linear.y = 0.0;
  out_msg.linear.z = 0.0;
  out_msg.angular.x = 0.0;
  out_msg.angular.y = 0.0;
  if (direction == STRAIGHT) {
    out_msg.linear.x = 0.6;
    out_msg.angular.z = 0.0;
  } else if (direction == LEFT) {
    out_msg.linear.x = 0.0;
    out_msg.angular.z = 0.5;
  } else if (direction == RIGHT) {
    out_msg.linear.x = 0.0;
    out_msg.angular.z = -0.5;
  } else {
    out_msg.linear.x = 0.0;
    out_msg.angular.z = 0.0;
  }

  nav_publisher.publish(out_msg);
}

void enigmaWalker::scanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
  float min_dist = 999999.99;
  for (float dist : msg->ranges) {
    if (std::isnan(dist)) {
      continue;
    }
    min_dist = min_dist > dist ? dist : min_dist;
  }

  if (min_dist < CLOSEST_DISTANCE) {
    float left_avg = 0.0;
    for (int i = 0; i < (msg->ranges.size()/2); ++i) {
      if (std::isnan(msg->ranges[i])) {
        left_avg += CLOSEST_DISTANCE;
      }
      left_avg += msg->ranges[i];
    }
    left_avg = left_avg / (msg->ranges.size()/2);

    float right_avg = 0.0;
    for (int i = (1 + msg->ranges.size()/2); i < msg->ranges.size(); ++i) {
      if (std::isnan(msg->ranges[i])) {
        right_avg += CLOSEST_DISTANCE;
      }
      right_avg += msg->ranges[i];
    }
    right_avg = right_avg / (msg->ranges.size()/2);

    if (left_avg > right_avg) {
      moveBot(LEFT);
    } else {
      moveBot(RIGHT);
    }
  } else {
    moveBot(STRAIGHT);
  }
}
