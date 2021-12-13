// Author: Jack Center

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/String.h>

#include <robot_calibration/capture_features.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv,"capture_features");
  ros::NodeHandle nh("~");

  if (!capture_features::check_parameters(nh))
  {
    ROS_FATAL("parameter checking failed.");
    return -1;
  }

  std::string bag_filename;                                 // Absolute path to location to save the bagfile
  nh.getParam("bag_filename", bag_filename);  
  rosbag::Bag bag(bag_filename, rosbag::bagmode::Write);    // rosbag to store calibration data

  std::string feature;
  nh.getParam("feature_finder", feature);                   // The feature to search for during caputre

  // Set up the calibration
  std_msgs::String description_msg = capture_features::republish_robot_description(nh);
  bag.write("/robot_description", ros::Time::now(), description_msg);

  bool capture_successful = true;

  // Auto capture mode
  if (nh.param<bool>("auto_calibration_mode", false))
  {
    ROS_INFO("Using automatic calibration mode...");
    capture_successful = capture_features::run_automatic_capture(nh, feature, &bag);
  }

  // Manual capture mode
  else
  {
    ROS_INFO("Using manual calibration mode...");
    capture_successful = capture_features::run_manual_capture(nh, feature, &bag);
  }

  bag.close();
  ROS_INFO("Done capturing samples");

  return capture_successful ? 0 : 1;
}

bool check_parameters(ros::NodeHandle& nh)
{
  bool success = true;
  std::string auto_capture_mode_name = "auto_capture_mode";

  std::string bag_filename = "bag_filename";
  std::string feature_finder_name = "feature_finder";

  // Checks that parameter exists and is a bool
  if (!nh.hasParam(auto_capture_mode_name))
  {
    ROS_ERROR_STREAM("calibration capture mode not set. Defaulting to manual mode. The value can be set in the launch file.");
    nh.setParam(auto_capture_mode_name, false);
  }

  if (!nh.hasParam(bag_filename))
  {
    ROS_ERROR_STREAM("bag filename parameter not set. Setting default location");
// TODO: fix this to be a better default
    std::string bag_filename_default = get_absolute_directory("/rosbags/default.bag");
    nh.setParam(bag_filename, bag_filename_default);
  }

  if (!nh.hasParam(feature_finder_name)){
    ROS_ERROR_STREAM("feature parameter not set. Defaulting to checkerboard finder. The value can be set in the launch file.");
    nh.setParam(feature_finder_name, "checkerboard_finder");
  }

  if (!nh.hasParam("/robot_description"))
  {
    ROS_FATAL("robot_description not set. Exiting the program");
    success = false;
  }

  if (success)
  {
    ROS_INFO_STREAM("parameters successfully set");
  }

  else{
    ROS_FATAL("unable to set the parameters");
  }

  return success;
}


std::string get_absolute_directory(std::string local_dir)
{
  std::string home_dir = getenv("HOME");
  std::string absolute_path = home_dir + local_dir;
  return absolute_path;
}


bool get_feature_finders(ros::NodeHandle& nh, robot_calibration::FeatureFinderMap& finders)
{
  bool success = true;
  robot_calibration::FeatureFinderLoader feature_finder_loader;     // Helper to load the feature finders

// TODO: why won't this unload?
  if (!feature_finder_loader.load(nh, finders))
  {
    ROS_FATAL("Unable to load feature finders");
    success = false;
  }

  return success;
}


bool run_automatic_capture(ros::NodeHandle& nh, 
                        robot_calibration::ChainManager* chain_manager, 
                        robot_calibration::FeatureFinderMap& finders,
                        std::string& feature,
                        rosbag::Bag& bag)
{
  // TODO
  robot_calibration_msgs::CalibrationData msg;
  // bool capture_complete = false;

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);

  std::vector<robot_calibration_msgs::CaptureConfig> poses;
// TODO: get a filename
  load_calibration_poses("test", poses);
  
  // Loop - for each loaded pose
  for (auto pose : poses) {
    /* TODO:
        1. Move to the pose
        2. Wait to settle
        3. Capture the calibration data
    */
    capture_calibration_data(chain_manager, finders, feature, msg);
  }

  return true;
}

bool run_manual_capture(ros::NodeHandle& nh, 
                        robot_calibration::ChainManager* chain_manager, 
                        robot_calibration::FeatureFinderMap& finders,
                        std::string& feature,
                        rosbag::Bag& bag)
{
  robot_calibration_msgs::CalibrationData msg;                      // Data message place holder
  int captured_poses;

  bool capture_complete = false;

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);
  

  // Loop - while poses are still being captured
  while (!capture_complete && ros::ok())
  {
    ROS_ERROR("failed to capture all samples");
    return -2;
  }

  else
  {
    return 0;
  }
}
