#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <boost/foreach.hpp>  // for rosbag iterator

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/optimize_parameters.h>


// TODO: make topic names not hardcoded 

namespace optimize_parameters
{

using std::nullopt;
using std::optional;
using std::pair;
using std::string;
using std::vector;

string get_absolute_directory(const string& local_dir)
{
  ROS_DEBUG_STREAM("getting the absolute path to " << local_dir);

  const auto home_dir = getenv("HOME");
  const auto absolute_path = home_dir + local_dir;

  return absolute_path;
}

bool check_parameters(const ros::NodeHandle& nh)
{
  bool success = true;
  string verbose_mode_name = "verbose";
  string bag_filename = "bag_filename";

  // Checks that parameter exists and is a bool
  if (!nh.hasParam(verbose_mode_name))
  {
    ROS_ERROR_STREAM("verbose output not set. Defaulting to standard output. The value can be set in the launch file.");
    nh.setParam(verbose_mode_name, false);
  }

  if (!nh.hasParam(bag_filename))
  {
    ROS_ERROR_STREAM("bag filename parameter not set. Setting default location");
// TODO: fix this to be a better default
    string bag_filename_default = get_absolute_directory("/rosbags/default.bag");
    nh.setParam(bag_filename, bag_filename_default);
  }

  if (success)
  {
    ROS_DEBUG_STREAM("parameters successfully set");
  }

  else{
    ROS_FATAL("unable to set the parameters");
  }

  return success;
}


optional<rosbag::Bag> open_rosbag(const string& bag_filename)
{
  ROS_DEBUG_STREAM("openning rosbag: " << bag_filename);

  rosbag::Bag bag;
  try
  {
    bag.open(bag_filename, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException& exception)
  {
    ROS_FATAL_STREAM(
      "Exception thrown when opening rosbag (" << bag_filename << "): " << exception.what());
    return nullopt;
  }

  return bag;
}


optional<std_msgs::String> load_robot_description(const rosbag::Bag& bag)
{
  ROS_DEBUG("loading robot description.");

  rosbag::View bag_view(bag, rosbag::TopicQuery("/robot_description"));

  if (bag_view.size() < 1)
  {
    ROS_FATAL("'/robot_description' topic not found in rosbag file.");
    return nullopt;
  }

  // TODO: what if there are more than 1 found?
  return *(bag_view.begin()->instantiate<std_msgs::String>());
}


optional<vector<robot_calibration_msgs::CalibrationData>>
load_calibration_data(const rosbag::Bag& bag)
{
  ROS_DEBUG("loading calibration data.");

  rosbag::View bag_view(bag, rosbag::TopicQuery("/calibration_data"));

  if (bag_view.size() < 1)
  {
    ROS_FATAL("'/calibration_data' topic not found in rosbag file.");
    return nullopt;
  }

  vector<robot_calibration_msgs::CalibrationData> calibration_data;
  BOOST_FOREACH (rosbag::MessageInstance const m, bag_view)
  {
    calibration_data.push_back(*(m.instantiate<robot_calibration_msgs::CalibrationData>()));
  }

  return calibration_data;
}


optional<pair<std_msgs::String, vector<robot_calibration_msgs::CalibrationData>>>
load_data(const string& bag_filename)
{
  ROS_DEBUG("starting to load data.");

  auto bag = open_rosbag(bag_filename);
  if (!bag) {
    return nullopt;
  }

  const auto robot_description = load_robot_description(*bag);
  if (!robot_description)
  {
    bag->close();
    return nullopt;
  }

  const auto calibration_data = load_calibration_data(*bag);
  if (!calibration_data)
  {
    bag->close();
    return nullopt;
  }

  bag->close();
  return pair{*robot_description, *calibration_data};
}

}  // namespace optimize_parameters
