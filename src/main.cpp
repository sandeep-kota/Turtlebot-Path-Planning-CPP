/**
 * @copyright  Copyright (c) Satyarth Praveen
 * @copyright  3-Clause BSD License
 * 
 * @file       main.cpp
 *
 * @brief      This file implements the main function.
 *
 * @author     Satyarth Praveen
 * @date       2019
 */

#include <ros/ros.h>
#include <enigmaWalker.h>


/**
 * @brief      creates a enigmaWalker object.
 *
 * @param[in]  argc  The count of arguments
 * @param      argv  The arguments array
 *
 * @return     0
 */
int main(int argc, char **argv) {
  /// Initializing a ros node.
  ros::init(argc, argv, "walker");

  enigmaWalker walker_obj;

  return 0;
}
