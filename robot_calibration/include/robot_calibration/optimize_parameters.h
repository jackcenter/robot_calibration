#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>

namespace optimize_parameters
{

/**
* @brief      Creates a string for the absoulte path for a file in the home directory.
*
* @param[in]  local_dir     name of the local directory.
*
* @return     string representing the absolute path to the file.
*/
std::string get_absolute_directory(const std::string& local_dir);

/**
* @brief      Verifies the required parameters have been set and, if not, set default values.
*
* @param[in]  node  node handle reference
*
* @return     bool indicating whether the parameters are all set.
*/
bool check_parameters(const ros::NodeHandle& node);

/**
* @brief      Opens the rosbag at the given location.
*
* @param[in]  bag_filename      absolute path to the rosbag file.
*
* @return     opened rosbag if opening succeded
*/
std::optional<rosbag::Bag> open_rosbag(const std::string& bag_filename);

/**
* @brief      Loads the robot description from the rosbag file.
*
* @param[in]  bag               rosbag object.
*
* @return     robot_description message if successful
*/
std::optional<std_msgs::String> load_robot_description(const rosbag::Bag& bag);


/**
* @brief      Loads the calibration data from the rosbag file.
*
* @param[in]  bag               rosbag object.
*
* @return     vector of calibration data if successful
*/
std::optional<std::vector<robot_calibration_msgs::CalibrationData>>
load_calibration_data(const rosbag::Bag& bag);

/**
* @brief      Organizes loading data from the rosbag file.
*
* @param[in]  bag_filename      absolute path to the rosbag file.
*
* @return     pair of robot description and calibration data if successful
*/
std::optional<std::pair<std_msgs::String, std::vector<robot_calibration_msgs::CalibrationData>>>
load_data(const std::string& bag_filename);

}  // namespace optimize_parameters
