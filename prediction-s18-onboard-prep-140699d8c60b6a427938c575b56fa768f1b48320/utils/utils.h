#pragma once

#include <opencv2/opencv.hpp>

#include <tf/transform_datatypes.h>

#include <path_planning_msgs/Environment.h>
#include <common_msgs/RoadLine.h>
#include <common_msgs/StopLine.h>

bool isPointInvisible(const geometry_msgs::Point &);
bool isPointInTrackingRange(const geometry_msgs::Point &);

bool isStopLineInTrackingRange(const common_msgs::StopLine &);
bool isStopLineInvisible(const common_msgs::StopLine &);
bool areStopLinesMatch(const common_msgs::StopLine &, const common_msgs::StopLine &);

void debugPrintRoadLine(const common_msgs::RoadLine &);
void debugPrintRoadLines(const std::vector<common_msgs::RoadLine> &);

common_msgs::RoadLine getRoadLineSegmentByYBounds(
    const common_msgs::RoadLine & road_line,
    const double lower,
    const double upper);
common_msgs::RoadLine mergeRoadLines(
    const common_msgs::RoadLine &,
    const common_msgs::RoadLine &);
common_msgs::RoadLine interpolateRoadLinePoints(const common_msgs::RoadLine &, const double);
int getRoadLinePointsOverlapCount(
    const common_msgs::RoadLine &,
    const common_msgs::RoadLine &);
double getRoadLinePointsOverlapRate(
    const common_msgs::RoadLine &,
    const common_msgs::RoadLine &);
std::vector<common_msgs::RoadLine> matchRoadLines(
    const std::vector<common_msgs::RoadLine> &,
    const std::vector<common_msgs::RoadLine> &);

std::vector<jsk_recognition_msgs::BoundingBox> matchBoundingBoxes(
    const std::vector<jsk_recognition_msgs::BoundingBox> &,
    const std::vector<jsk_recognition_msgs::BoundingBox> &,
    const geometry_msgs::Pose &);

double distanceBetween(const geometry_msgs::Point &, const geometry_msgs::Point &);

geometry_msgs::Point rotatePointCCW90Deg(const geometry_msgs::Point &);

geometry_msgs::Point convertPOVToNovatel(
    const geometry_msgs::Point &,
    const geometry_msgs::Pose &);
geometry_msgs::Point convertNovatelToPOV(
    const geometry_msgs::Point &,
    const geometry_msgs::Pose &);

common_msgs::StopLine convertStopLinePOVToNovatel(
    const common_msgs::StopLine &,
    const geometry_msgs::Pose &);
std::vector<common_msgs::StopLine> convertStopLinesPOVToNovatel(
    const std::vector<common_msgs::StopLine> &,
    const geometry_msgs::Pose &);
common_msgs::StopLine convertStopLineNovatelToPOV(
    const common_msgs::StopLine &,
    const geometry_msgs::Pose &);
std::vector<common_msgs::StopLine> convertStopLinesNovatelToPOV(
    const std::vector<common_msgs::StopLine> &,
    const geometry_msgs::Pose &);

common_msgs::RoadLine convertRoadLinePOVToNovatel(
    const common_msgs::RoadLine &,
    const geometry_msgs::Pose &);
std::vector<common_msgs::RoadLine> convertRoadLinesPOVToNovatel(
    const std::vector<common_msgs::RoadLine> &,
    const geometry_msgs::Pose &);
common_msgs::RoadLine convertRoadLineNovatelToPOV(
    const common_msgs::RoadLine &,
    const geometry_msgs::Pose &);
std::vector<common_msgs::RoadLine> convertRoadLinesNovatelToPOV(
    const std::vector<common_msgs::RoadLine> &,
    const geometry_msgs::Pose &);

geometry_msgs::Quaternion convertBoundingBoxOrientationPOVToNovatel(
    const geometry_msgs::Quaternion &,
    const geometry_msgs::Pose &);
jsk_recognition_msgs::BoundingBox convertBoundingBoxLidarPOVToNovatel(
    const jsk_recognition_msgs::BoundingBox &,
    const geometry_msgs::Pose &);
cv::RotatedRect convertBoundingBoxToRotatedRect(
    const jsk_recognition_msgs::BoundingBox &,
    const geometry_msgs::Pose &);
