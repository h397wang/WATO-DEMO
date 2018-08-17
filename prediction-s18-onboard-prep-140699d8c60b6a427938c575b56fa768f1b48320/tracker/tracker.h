#pragma once

#include <ros/ros.h>

// standard ROS messages
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

// WATO messages
#include <path_planning_msgs/Environment.h>
#include <perception_msgs/LaneDetectionOutput.h>
#include <common_msgs/RoadLine.h>
#include <common_msgs/StopLine.h>

// standard libraries
#include <vector>
#include <queue>
#include <mutex>

class Tracker{

public:
    Tracker(ros::NodeHandle &node, bool passthrough = false);

private:
    // Forward messages from Perception without modification.
    bool passthrough_;

    // handles, pubs, subs
    ros::NodeHandle node_;
    ros::Publisher environment_pub_;
    ros::Subscriber lane_detections_sub_;
    ros::Subscriber stop_line_detections_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber bounding_box_sub_;
    
    std::mutex callback_lock_;

    // Message handlers
    void stopLineCallback(const common_msgs::StopLine &);
    void laneDetectionCallback(const perception_msgs::LaneDetectionOutput &);
    void odomCallback(const nav_msgs::Odometry &);
    void objectDetectionCallback(const jsk_recognition_msgs::BoundingBoxArray &);
        
    void receiveRoadLines(const std::vector<common_msgs::RoadLine> & road_lines);
    void receiveStopLines(const common_msgs::StopLine & stop_line);
    void receiveRoadObjects(const std::vector<jsk_recognition_msgs::BoundingBox> & bounding_boxes);
    void receiveGlobalPose(const geometry_msgs::Pose & new_pose);
    
    path_planning_msgs::Environment getRoadDataToPublish();

    const int pose_receive_to_publish_ratio = 2; // 25 Hz
    int publish_count_mod = 0;

    // Messages to store
    std::vector<common_msgs::RoadLine> road_lines_nov_;
    std::vector<common_msgs::StopLine> stop_lines_nov_;

    std::vector<common_msgs::RoadLine> road_lines_pov_;
    std::vector<common_msgs::StopLine> stop_lines_pov_;
    std::vector<jsk_recognition_msgs::BoundingBox> bounding_boxes_nov_;
    geometry_msgs::Pose pose_;
    bool valid_pose_ = false;
    path_planning_msgs::Environment road_data_;

    // Passthrough messages to store
    std::vector<common_msgs::RoadLine> road_lines_raw_;
    std::vector<common_msgs::StopLine> stop_lines_raw_;

    void setAllTrackedLanesNotVisible();

    geometry_msgs::Point updatePointPOV(
        const geometry_msgs::Point & point_pov_prev,
        const geometry_msgs::Pose & pose_nova_prev,
        const geometry_msgs::Pose & pose_nova_new);
    void updateRoadLinesPOV(const geometry_msgs::Pose &);
    common_msgs::StopLine updateStopLinePOV(
        const common_msgs::StopLine &,
        const geometry_msgs::Pose &);
    void updateStopLinesPOV(const geometry_msgs::Pose &);
    void updateBoundingBoxesPOV(const geometry_msgs::Pose &);

    void filterStopLinesByTrackingBounds(const geometry_msgs::Pose &);
    bool isStopLineTracked(const common_msgs::StopLine &);
};
