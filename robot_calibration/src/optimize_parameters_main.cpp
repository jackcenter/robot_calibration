#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/optimize_parameters.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv,"robot_calibration");
  ros::NodeHandle nh("~");

  // Parameters
  std::string bag_filename;                                   // Absolute path to location to save the bagfile
  bool verbose;                                               // Level of data to output to the screen

  if(!optimize_parameters::check_parameters(nh))
    return -1;

  nh.getParam("bag_filename", bag_filename);
  nh.getParam("verbose", verbose);

  // Load data
  ROS_INFO("Loading optimization data");

  const auto load_data_result = optimize_parameters::load_data(bag_filename);
  if (!load_data_result) {
    return -1;
  }
  const auto [robot_description, calibration_data] = *load_data_result;

  // Run optimization
  ROS_INFO("Running optimization");
  robot_calibration::Optimizer opt(robot_description.data);

  robot_calibration::OptimizationParams params;
  params.LoadFromROS(nh);
  opt.optimize(params, calibration_data, verbose);

  // Output optimization to screen
  if (verbose)
  {
    std::cout << "Parameter Offsets: \n";
    std::cout << opt.getOffsets()->getOffsetYAML() << "\n";
  }

  ROS_INFO("Done calibrating");

  return 0;
}
