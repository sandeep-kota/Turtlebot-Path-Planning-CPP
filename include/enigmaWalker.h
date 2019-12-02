/**
 * @copyright  Copyright (c) Satyarth Praveen
 * @copyright  3-Clause BSD License
 * 
 * @file       enigmaWalker.h
 *
 * @brief      Header file for the enigmaWalker.cpp file.
 *
 * @author     Satyarth Praveen
 * @date       2019
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
