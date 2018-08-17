#include <lane_publisher/publisher_template.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <path_planning_msgs/Environment.h>
#include <geometry_msgs/Pose.h>

#include <iostream>

// Time periods should correspond to MATLAB script
// All data should be published at the same frequency 
// because all the tracker simply caches messages
// and published based on a timer
const double FUNDAMENTAL_PERIOD = 0.033;
const double ROAD_LINE_PERIOD = 10 * FUNDAMENTAL_PERIOD;
const double IMU_PERIOD = FUNDAMENTAL_PERIOD;
const double ODOMETRY_PERIOD = FUNDAMENTAL_PERIOD;
const double STOP_LINE_PERIOD = 10 * FUNDAMENTAL_PERIOD;
const double LOCAL_MAPPING_PERIOD = 10 * FUNDAMENTAL_PERIOD;

const std::string ROAD_LINE_TOPIC = "/lane_detection_output";
const std::string IMU_TOPIC = "/imu/data";
const std::string ODOMETRY_TOPIC = "/navsat/odom";
const std::string STOP_LINE_TOPIC = "/stopline_detection_output";
const std::string LOCAL_MAPPING_TOPIC = "/local_mapping_data";

const std::string ROAD_LINE_INPUT_PARAM = "/road_line_file";
const std::string IMU_INPUT_PARAM = "/imu_file";
const std::string ODOMETRY_INPUT_PARAM = "/odometry_file";
const std::string STOP_LINE_INPUT_PARAM = "/stop_line_file";
const std::string LOCAL_MAPPING_INPUT_PARAM = "/local_mapping_file";

int main(int argc, char* argv[])
{
  std::string file_path;

  ros::init(argc, argv, "lane_publisher");
  ros::NodeHandle node;

  PublisherTemplate<perception_msgs::LaneDetectionOutput> road_line_publisher(node, ROAD_LINE_TOPIC, FUNDAMENTAL_PERIOD, ROAD_LINE_PERIOD);
  if (!node.getParam(ROAD_LINE_INPUT_PARAM, file_path))
  {
    std::cerr << ROAD_LINE_INPUT_PARAM << " parameter not specified!" << std::endl;
    return -1;
  }
  if (road_line_publisher.readInputFile(file_path) != 0) return -1;
  
  PublisherTemplate<sensor_msgs::Imu> imu_publisher(node, IMU_TOPIC, FUNDAMENTAL_PERIOD, IMU_PERIOD);
  if (!node.getParam(IMU_INPUT_PARAM, file_path))
  {
    std::cerr << IMU_INPUT_PARAM << " parameter not specified!" << std::endl;
    return -1;
  }
  if (imu_publisher.readInputFile(file_path) != 0) return -1;
  
  PublisherTemplate<nav_msgs::Odometry> odometry_publisher(node, ODOMETRY_TOPIC, FUNDAMENTAL_PERIOD, ODOMETRY_PERIOD);
  if (!node.getParam(ODOMETRY_INPUT_PARAM, file_path))
  {
    std::cerr << ODOMETRY_INPUT_PARAM << " parameter not specified!" << std::endl;
    return -1;
  }
  if (odometry_publisher.readInputFile(file_path) != 0) return -1;
  
  PublisherTemplate<common_msgs::StopLine> stop_line_publisher(node, STOP_LINE_TOPIC, FUNDAMENTAL_PERIOD, STOP_LINE_PERIOD);
  if (!node.getParam(STOP_LINE_INPUT_PARAM, file_path))
  {
    std::cerr << STOP_LINE_INPUT_PARAM << " parameter not specified!" << std::endl;
    return -1;
  }
  if (stop_line_publisher.readInputFile(file_path) != 0) return -1;

  PublisherTemplate<jsk_recognition_msgs::BoundingBoxArray> local_mapping_publisher(node, LOCAL_MAPPING_TOPIC, FUNDAMENTAL_PERIOD, LOCAL_MAPPING_PERIOD);
  if (!node.getParam(LOCAL_MAPPING_INPUT_PARAM, file_path))
  {
    std::cerr << LOCAL_MAPPING_INPUT_PARAM << " parameter not specified!" << std::endl;
    return -1;
  }
  if (local_mapping_publisher.readInputFile(file_path) != 0) return -1;

  road_line_publisher.startPublishing();
  imu_publisher.startPublishing();
  odometry_publisher.startPublishing();
  stop_line_publisher.startPublishing();
  local_mapping_publisher.startPublishing();

  ROS_INFO("Done publishing");

  ros::spin();

  return 0;
}
