#include <ros/ros.h>

#include <tracker/tracker.h>
#include <utils/utils.h>
#include <utils/phys_params.h>

#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include <iostream>
#include <algorithm>
#include <cassert>
#include <cmath>

#include <opencv2/opencv.hpp>


Tracker::Tracker(ros::NodeHandle & node, bool passthrough):
    passthrough_(passthrough),
    node_(node),
    environment_pub_(node.advertise<path_planning_msgs::Environment>("/environment_prediction", 1)),
    lane_detections_sub_(node.subscribe("/lane_detection_output", 1, &Tracker::laneDetectionCallback, this)),
    stop_line_detections_sub_(node.subscribe("/stopline_detection_output", 1, &Tracker::stopLineCallback, this)),
    odom_sub_(node.subscribe("/navsat/odom", 1, &Tracker::odomCallback, this)),
    bounding_box_sub_(node.subscribe("/bounding_boxes", 1, &Tracker::objectDetectionCallback, this))
{
}

void Tracker::laneDetectionCallback(const perception_msgs::LaneDetectionOutput & lane_detection_output)
{
    std::lock_guard<std::mutex> guard(callback_lock_);
    if (passthrough_) {
        road_lines_raw_ = lane_detection_output.lanes;
    }
    // TODO: Sanity checks
    receiveRoadLines(lane_detection_output.lanes);
} // mutex released on scope exit

void Tracker::stopLineCallback(const common_msgs::StopLine & stop_line_pov)
{
    std::lock_guard<std::mutex> guard(callback_lock_);

    if (passthrough_) {
        stop_lines_raw_.clear();
        stop_lines_raw_.push_back(stop_line_pov);
    }

    // TODO: We still need this just because lane_publisher publishes 0,0,0,0 for empties
    const bool emptyDetection = (stop_line_pov.region[0].x == 0)
        && (stop_line_pov.region[0].y == 0)
        && (stop_line_pov.region[1].x == 0)
        && (stop_line_pov.region[1].y == 0);
    if (emptyDetection){
        return;
    }

    if (isStopLineInvisible(stop_line_pov)){
        return;
    }

    const float lane_width = 5;
    const bool within_lane = (abs(stop_line_pov.region[0].x) < lane_width)
        && (abs(stop_line_pov.region[1].x) < lane_width);
    if (!within_lane) {
        return;
    }

    receiveStopLines(stop_line_pov);
} // mutex released on scope exit

void Tracker::odomCallback(const nav_msgs::Odometry & odom)
{
    std::lock_guard<std::mutex> guard(callback_lock_);

    receiveGlobalPose(odom.pose.pose);

    if (publish_count_mod == pose_receive_to_publish_ratio) {
        path_planning_msgs::Environment road_data = getRoadDataToPublish();
        environment_pub_.publish(road_data);
        publish_count_mod = 0;
    } else {
        publish_count_mod++;
    }

} // mutex released on scope exit

void Tracker::objectDetectionCallback(const jsk_recognition_msgs::BoundingBoxArray & bounding_box_list)
{
    std::lock_guard<std::mutex> guard(callback_lock_);
    receiveRoadObjects(bounding_box_list.boxes);
} // mutex released on scope exit

// called when no detections received for some time
void Tracker::setAllTrackedLanesNotVisible() {
    for (common_msgs::RoadLine & line : road_lines_pov_) {
        line.visible = 0;
    }
}

/*
Brief:
    Match incoming road line message with currently tracked road lines before storing.
Inputs:
    road_line : message recieved from perception
*/
void Tracker::receiveRoadLines(const std::vector<common_msgs::RoadLine> & road_lines_pov)
{
    if (road_lines_nov_.size() == 0) {
        road_lines_nov_ = convertRoadLinesPOVToNovatel(road_lines_pov, pose_);
    } else {
        std::vector<common_msgs::RoadLine> road_lines_pov_tracked = convertRoadLinesNovatelToPOV(road_lines_nov_, pose_);
        // Matching is currenty done in POV
        std::vector<common_msgs::RoadLine> road_lines_pov_merged = matchRoadLines(road_lines_pov, road_lines_pov_tracked);
        road_lines_nov_ = convertRoadLinesPOVToNovatel(road_lines_pov_merged, pose_);
    }
}

/*
Brief:
    Store incoming stop line message.
Inputs:
    stop_line : message recieved from perception
*/
void Tracker::receiveStopLines(const common_msgs::StopLine & stop_line_pov)
{   
    const common_msgs::StopLine stop_line_nov = convertStopLinePOVToNovatel(stop_line_pov, pose_);
    if (stop_lines_nov_.size() == 0) {
        stop_lines_nov_.push_back(stop_line_nov);
    } else {
        bool matched = false;
        for (auto & stop_line_nov_tracked : stop_lines_nov_) {
            if (areStopLinesMatch(stop_line_nov_tracked, stop_line_nov)) {
                // TODO: needs non-naive merge method
                stop_line_nov_tracked = stop_line_nov;
                break;
            }
        }
        if (!matched) {
            stop_lines_nov_.push_back(stop_line_nov);
        }
    }
}

/*
Brief:
    Convert bounding box messages from lidar to novatel coordinate system for tracking.
Inputs:
    bounding_boxes : message recieved from local mapping
*/
void Tracker::receiveRoadObjects(const std::vector<jsk_recognition_msgs::BoundingBox> & bounding_boxes)
{
    std::vector<jsk_recognition_msgs::BoundingBox> transformed_boxes;
    for (const auto box : bounding_boxes) {
        transformed_boxes.push_back(convertBoundingBoxLidarPOVToNovatel(box, pose_));
    }
    bounding_boxes_nov_ = matchBoundingBoxes(transformed_boxes, bounding_boxes_nov_, pose_);
}

/*
Brief:
    Update internal pose
Inputs:
    new_pose : message describing vehicle pose from sensor fusion to be cached
*/
void Tracker::receiveGlobalPose(const geometry_msgs::Pose & new_pose)
{
    geometry_msgs::Pose p = new_pose;
    // TODO: remove for updated rosbags
    p.position = rotatePointCCW90Deg(new_pose.position);
    pose_ = p;
}

/*
Brief:
    Update tracked road lines (pov) based on vehicle displacement.
Inputs:
    new_car_pose : current vehicle pose (standard Novatel coordinates)
*/
void Tracker::updateRoadLinesPOV(const geometry_msgs::Pose & new_car_pose)
{
    std::vector<common_msgs::RoadLine> road_lines_updated;
    for (const auto road_line_pov : road_lines_pov_) {
        std::vector<geometry_msgs::Point> trackable_points;
        for (const auto point : road_line_pov.line) {
            const geometry_msgs::Point new_point = updatePointPOV(point, pose_, new_car_pose);
            if (isPointInTrackingRange(new_point)) {
                trackable_points.push_back(new_point);
            }
        }
        if (trackable_points.size() > 1) {
            common_msgs::RoadLine road_line_updated;
            road_line_updated.line = trackable_points;
            road_line_updated.visible = road_line_pov.visible;
            road_line_updated.type = road_line_pov.type;
            road_lines_updated.push_back(road_line_updated);
        }
    }
    road_lines_pov_ = road_lines_updated;
}

/*
Brief:
    Return updated/transformed stop line (pov) based on change in vehicle pose.
Inputs:
    stop_line : current stop line (pov) to be transformed
Outputs:
    stop line with transformed coordiates based on current vehicle pose
*/
common_msgs::StopLine Tracker::updateStopLinePOV(
    const common_msgs::StopLine & stop_line_pov,
    const geometry_msgs::Pose & new_car_pose)
{
    auto left = stop_line_pov.region[0];
    auto right = stop_line_pov.region[1];
    left = updatePointPOV(left, pose_, new_car_pose);
    right = updatePointPOV(right, pose_, new_car_pose);

    common_msgs::StopLine stop_line_updated;
    stop_line_updated.region.push_back(left);
    stop_line_updated.region.push_back(right);
    return stop_line_updated;
}

/*
Brief:
    Update tracked stop lines (pov) based on vehicle displacement
Inputs:
    new_car_pose : current vehicle pose (standard Novatel coordinates)
*/
void Tracker::updateStopLinesPOV(const geometry_msgs::Pose & new_car_pose)
{
    std::vector<common_msgs::StopLine> stop_lines_updated;
    for (const auto stop_line_pov : stop_lines_pov_) {
        const auto stop_line_updated = updateStopLinePOV(stop_line_pov, new_car_pose);
        if (isStopLineInTrackingRange(stop_line_updated)) {
            stop_lines_updated.push_back(stop_line_updated);
        }
    }
    stop_lines_pov_ = stop_lines_updated;
}

/*
Brief:
    Update  point in the POV frame of reference based on vehicle displacement.
Inputs:
    point_pov_prev : original point (pov)
    pose_nova_prev : previous vehicle pose (standard Novatel coordinates)
    pose_nova_new  : current vehicle pose (standard Novatel coordinates)
Outputs:
    point_pov_new : updated point (pov)
*/
geometry_msgs::Point Tracker::updatePointPOV(
    const geometry_msgs::Point & point_pov_prev,
    const geometry_msgs::Pose & pose_nova_prev,
    const geometry_msgs::Pose & pose_nova_new)
{
    const double yaw_prev = tf::getYaw(pose_nova_prev.orientation);
    const double yaw_new = tf::getYaw(pose_nova_new.orientation);
    const double delta_yaw = yaw_new - yaw_prev;

    tf::Point tf_point_pov_prev;
    tf::pointMsgToTF(point_pov_prev, tf_point_pov_prev);

    tf::Point tf_car_pos_nova_prev;
    tf::pointMsgToTF(pose_nova_prev.position, tf_car_pos_nova_prev);

    tf::Point tf_car_pos_nova_new;
    tf::pointMsgToTF(pose_nova_new.position, tf_car_pos_nova_new);

    const tf::Vector3 z_axis_nova = {0, 0, 1};
    tf::Point tf_car_displacement_nova = tf_car_pos_nova_new - tf_car_pos_nova_prev;
    tf::Point tf_car_displacement_pov = tf_car_displacement_nova.rotate(z_axis_nova, -yaw_prev);

    tf::Point tf_point_pov_new = tf_point_pov_prev.rotate(z_axis_nova, -delta_yaw);
    tf_point_pov_new -= tf_car_displacement_pov;
   
    geometry_msgs::Point point_pov_new;
    tf::pointTFToMsg(tf_point_pov_new, point_pov_new);

#ifdef DEBUG_UPDATE_POV
    ROS_INFO("yaw_prev: %.2f", yaw_prev);
    ROS_INFO("yaw_new: %.2f", yaw_new);
    ROS_INFO("delta_yaw: %.2f", delta_yaw);
    ROS_INFO("point_pov_prev.x: %.2f, point_pov_prev.y: %.2f",
        point_pov_prev.x,
        point_pov_prev.y);
    ROS_INFO("pose_nova_prev.position.x: %.2f, pose_nova_prev.position.y: %.2f",
        pose_nova_prev.position.x,
        pose_nova_prev.position.y);
    ROS_INFO("pose_nova_new.position.x: %.2f, pose_nova_new.position.y: %.2f",
        pose_nova_new.position.x,
        pose_nova_new.position.y);
    ROS_INFO("tf_car_displacement_nova.x: %.2f, tf_car_displacement_nova.y: %.2f",
        tf_car_displacement_nova.getX(),
        tf_car_displacement_nova.getY());
    ROS_INFO("tf_car_displacement_pov.x: %.2f, tf_car_displacement_pov.y: %.2f",
        tf_car_displacement_pov.getX(),
        tf_car_displacement_pov.getY());
    ROS_INFO("tf_point_pov_new.x: %.2f, tf_point_pov_new.y: %.2f",
        tf_point_pov_new.getX(),
        tf_point_pov_new.getY());
#endif // DEBUG_UPDATE_POV
    return point_pov_new;
}

/*
Brief:
    Create and publish environment message.
Inputs:
    none
Outputs:
    env : environment message
*/
path_planning_msgs::Environment Tracker::getRoadDataToPublish()
{
    path_planning_msgs::Environment env;
    if (passthrough_) {
        env.lanes = convertRoadLinesPOVToNovatel(road_lines_raw_, pose_);
        env.stop_lines = convertStopLinesPOVToNovatel(stop_lines_raw_, pose_);
    } else {
        env.lanes = convertRoadLinesPOVToNovatel(road_lines_pov_, pose_);
        env.stop_lines = stop_lines_nov_;
    }
    env.obstacles = bounding_boxes_nov_;
    env.global_pose = pose_;
    return env;
}

void Tracker::filterStopLinesByTrackingBounds(const geometry_msgs::Pose & new_car_pose)
{
    std::vector<common_msgs::StopLine> stop_lines_nov_to_track;
    for (const auto stop_line_nov : stop_lines_nov_) {
        common_msgs::StopLine stop_line_pov = convertStopLineNovatelToPOV(stop_line_nov, new_car_pose);
        if (isStopLineInTrackingRange(stop_line_pov)) {
            stop_lines_nov_to_track.push_back(stop_line_nov);
        }
    }
    stop_lines_nov_ = stop_lines_nov_to_track;
}
