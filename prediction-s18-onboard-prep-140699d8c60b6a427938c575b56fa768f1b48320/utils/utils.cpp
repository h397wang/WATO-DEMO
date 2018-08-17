#include <utils/utils.h>
#include <utils/phys_params.h>
#include <utils/spline.h>

bool isPointInvisible(const geometry_msgs::Point & point)
{
    return (point.y >= lower_y_bound_to_track && point.y <= lower_y_bound_visible);
}

bool isPointInTrackingRange(const geometry_msgs::Point & point)
{
    return (point.y > lower_y_bound_to_track);
}

/*
Brief: TODO: revamped on pivot
    Stop line is classified as in tracking range if both end points of the stop line are above tracking bounds.
Inputs:
    stop_line : stop line under test
Outputs:
    true if this stop line is in tracking range, otherwise false
*/
bool isStopLineInTrackingRange(const common_msgs::StopLine & stop_line_pov)
{
    const auto left = stop_line_pov.region[0];
    const auto right = stop_line_pov.region[1];
    bool is_stop_line_in_tracking_range = isPointInTrackingRange(left) && isPointInTrackingRange(right);
    return is_stop_line_in_tracking_range;
}

/*
Brief:
    Stop line is classified as invisibile if at least 1 point of the stop line is invisible.
Inputs:
    stop_line : stop line under test
Outputs:
    true if this stop line is invisible, otherwise false
*/
bool isStopLineInvisible(const common_msgs::StopLine & stop_line_pov)
{
    const auto point_left = stop_line_pov.region[0];
    const auto point_right = stop_line_pov.region[1];
    bool is_stop_line_invisible = isPointInvisible(point_left) || isPointInvisible(point_right);
    return is_stop_line_invisible;
}

bool areStopLinesMatch(const common_msgs::StopLine & sl_1, const common_msgs::StopLine & st_2) {
    return true;
}

void debugPrintRoadLine(const common_msgs::RoadLine & line)
{
    std::cerr << "\n    [";
    for (auto &point : line.line) {
        std::cerr << "(" << point.x << "," << point.y << "),";
    }
    std::cerr << "]\n";
}

void debugPrintRoadLines(const std::vector<common_msgs::RoadLine> & road_lines)
{
    for (const auto road_line : road_lines) {
        debugPrintRoadLine(road_line);
    }
}

common_msgs::RoadLine getRoadLineSegmentByYBounds(
    const common_msgs::RoadLine & road_line,
    const double lower,
    const double upper)
{
    common_msgs::RoadLine segment;
    for (const auto point : road_line.line) {
        if (lower < point.y) {
            if (point.y < upper) {
                segment.line.push_back(point);
            } else {
                // points are in order
                break;
            }
        }
    }
    return segment;
}

common_msgs::RoadLine mergeRoadLines(
    const common_msgs::RoadLine & detected_line,
    const common_msgs::RoadLine & tracked_line)
{
    const double min_y_detected = detected_line.line[0].y;

    std::vector<geometry_msgs::Point> points_to_track;
    for (const auto point : tracked_line.line) {
        if (point.y < min_y_detected) {
            points_to_track.push_back(point);
        }
    }
    // TODO: current implementation is naive as it assumes all incoming data is valid and does no filtering
    for (const auto point : detected_line.line) {
        points_to_track.push_back(point);
    }

    common_msgs::RoadLine merged_road_line;
    merged_road_line.line = points_to_track;
    merged_road_line.type = detected_line.type;

#ifdef DEBUG_MERGE_ROAD_LINES
    debugPrintRoadLine(detected_line);
    debugPrintRoadLine(tracked_line);
    debugPrintRoadLine(merged_road_line);
#endif  // DEBUG_MERGE_ROAD_LINES

    return merged_road_line;
}


/*
Brief:
    Increase the point per meter density of the road line
Inputs:
    road_line : road line original
    epsilon   : meter per point density
Outputs:
    road line with desired point density
*/
common_msgs::RoadLine interpolateRoadLinePoints(const common_msgs::RoadLine & road_line, const double epsilon) {
    assert(epsilon > 0);
    assert(road_line.line.size() >= 2); // TODO: this is getting triggered alot for some reason

    common_msgs::RoadLine road_line_interp;
    for (size_t i = 1; i < road_line.line.size(); ++i) {
        // Add a bunch of points between a and b, at intervals of distance = epsilon
        const geometry_msgs::Point & a = road_line.line[i - 1];
        const geometry_msgs::Point & b = road_line.line[i];
        const double delta_x = b.x - a.x;
        const double delta_y = b.y - a.y;
        const double dist_a_b = distanceBetween(a, b);
        for (double dist = 0; dist < dist_a_b; dist += epsilon) {
            // Insert the point at some fraction t of the way from a to b.
            const double t = dist / dist_a_b;
            geometry_msgs::Point p;
            p.x = a.x + delta_x * t;
            p.y = a.y + delta_y * t;
            road_line_interp.line.push_back(p);
        }
    }
    // push on the very last point
    road_line_interp.line.push_back(road_line.line.back());

    return road_line_interp;
}

int getRoadLinePointsOverlapCount(
    const common_msgs::RoadLine & a_subdivided,
    const common_msgs::RoadLine & b_subdivided)
{
    int a_near_b = 0;
    for (const geometry_msgs::Point & p_a : a_subdivided.line) {
        for (const geometry_msgs::Point & p_b : b_subdivided.line) {
            if (distanceBetween(p_a, p_b) < max_delta) {
                ++a_near_b;
                break;
            }
        }
    }
    return a_near_b;
}

double getRoadLinePointsOverlapRate(
    const common_msgs::RoadLine & detected_road_line,
    const common_msgs::RoadLine & tracked_road_line)
{
    const common_msgs::RoadLine a_subdivided = interpolateRoadLinePoints(detected_road_line, meters_per_point);
    const common_msgs::RoadLine b_subdivided = interpolateRoadLinePoints(tracked_road_line, meters_per_point);
    const int num_points_overlap = getRoadLinePointsOverlapCount(a_subdivided, b_subdivided);
    const int num_points_inspected = std::min(a_subdivided.line.size(), b_subdivided.line.size());
    const double overlap_rate = (double)num_points_overlap / (double)num_points_inspected;
    return overlap_rate;
}

/*
Brief:
    Compare tracked road lines with detected, merge matching road lines.
Inputs:
    road_lines_detected :
    road_lines_tracked :
Outputs:
    new set of road lines obtained from merging tracked and detected
*/
std::vector<common_msgs::RoadLine> matchRoadLines(
    const std::vector<common_msgs::RoadLine> & road_lines_detected,
    const std::vector<common_msgs::RoadLine> & road_lines_tracked)
{
    assert(road_lines_tracked.size() > 0);

    std::vector<common_msgs::RoadLine> ret;
    std::vector<common_msgs::RoadLine> tracked_lines_not_matched = road_lines_tracked;

    for (auto detected_road_line : road_lines_detected) {
        std::vector<double> scores;
    
        for (const auto tracked_road_line : tracked_lines_not_matched) {
            const double score = getRoadLinePointsOverlapRate(detected_road_line, tracked_road_line);
            scores.push_back(score);
        }

        std::vector<double>::iterator max_it = std::max_element(scores.begin(), scores.end());
        const int max_score_index = std::distance(scores.begin(), max_it);
        const double max_score = *max_it;

        if (max_score > match_threshold) {
            common_msgs::RoadLine merged_road_line = mergeRoadLines(detected_road_line, tracked_lines_not_matched[max_score_index]);
            merged_road_line.visible = 1; // set line as partially visible
            // avoid shrinking vectors, and null vectors
            tracked_lines_not_matched[max_score_index] = merged_road_line;
        } else {
            detected_road_line.visible = 1; // set line as visible
            ret.push_back(detected_road_line);
        }
    }

    // Ordering of lines does not matter, so insert freely
    for (size_t i = 0; i < tracked_lines_not_matched.size(); i++) {
        ret.push_back(tracked_lines_not_matched[i]);
    }

#ifdef DEBUG_MATCH_ROAD_LINES
    ROS_INFO("tracked_lines_not_matched.size(): %lu", tracked_lines_not_matched.size());
    ROS_INFO("road_lines_detected.size(): %lu", road_lines_detected.size());

    debugPrintRoadLines(road_lines_detected);
    debugPrintRoadLines(road_lines_tracked);
    debugPrintRoadLines(ret);
#endif  // DEBUG_MATCH_ROAD_LINES

    return ret;
}

/*
Brief:
    match tracked bounding boxes with detecions, matching based on some positional? thresholding
Inputs:
    bounding_boxes_detected :
    bounding_boxes_tracked :
    car_pose :
Outputs:
    fused vector of detected and tracked bounding boxes after matching
*/
std::vector<jsk_recognition_msgs::BoundingBox> matchBoundingBoxes(
    const std::vector<jsk_recognition_msgs::BoundingBox> & bounding_boxes_detected,
    const std::vector<jsk_recognition_msgs::BoundingBox> & bounding_boxes_tracked,
    const geometry_msgs::Pose & car_pose)
{
#if DEBUG_MATCH_BOUNDING_BOXES
    ROS_INFO("bounding_boxes_detected.size(): %lu", bounding_boxes_detected.size());
    ROS_INFO("bounding_boxes_tracked.size(): %lu", bounding_boxes_tracked.size());
#endif // DEBUG_MATCH_BOUNDING_BOXES

    std::vector<jsk_recognition_msgs::BoundingBox> ret;
    // check all tracked bounding boxes and push ones without a match
    // loop through each tracked bounding box
    for (auto tracked_box : bounding_boxes_tracked) {
        bool matched = false;
        // loop through each detected bounding box
        for (auto detected_box : bounding_boxes_detected) {
            double center_displacement = distanceBetween(detected_box.pose.position, tracked_box.pose.position);
            if (center_displacement <= bounding_box_match_threshold) {
                matched = true;
                break;
            }
        }
        if (!matched) {
            // TODO: not really efficient to convert it back into POV
            const geometry_msgs::Point center_pov = convertNovatelToPOV(tracked_box.pose.position, car_pose);
            if (isPointInTrackingRange(center_pov)) {
                ret.push_back(tracked_box);
            } else {
#if DEBUG_MATCH_BOUNDING_BOXES
                ROS_INFO("Bounding box not in tracking range");
#endif // DEBUG_MATCH_BOUNDING_BOXES
            }
        }
    }
    // TODO: naively appending all detected boxes
    for (auto detected_box : bounding_boxes_detected) {
        ret.push_back(detected_box);
    }

    return ret;
}

double distanceBetween(const geometry_msgs::Point & a, const geometry_msgs::Point & b)
{
    return hypot(a.x - b.x, a.y - b.y);
}

// Originally used to acomodate for novatel hack that makes +y point W and +x point N
geometry_msgs::Point rotatePointCCW90Deg(const geometry_msgs::Point & point)
{
    geometry_msgs::Point point_rotated;
    point_rotated.x = -point.y;
    point_rotated.y = point.x;
    return point_rotated;
}

geometry_msgs::Point convertPOVToNovatel(
    const geometry_msgs::Point & point_pov,
    const geometry_msgs::Pose & car_pose)
{
    const double yaw = tf::getYaw(car_pose.orientation);

    tf::Point tf_car_pos_nova;
    tf::pointMsgToTF(car_pose.position, tf_car_pos_nova);

    tf::Point tf_point_pov;
    tf::pointMsgToTF(point_pov, tf_point_pov);

    const tf::Vector3 z_axis_nova = {0, 0, 1};
    tf::Point tf_point_nova = tf_point_pov.rotate(z_axis_nova, yaw) + tf_car_pos_nova;

    geometry_msgs::Point point_nova;
    tf::pointTFToMsg(tf_point_nova, point_nova);

    return point_nova;
}

geometry_msgs::Point convertNovatelToPOV(
    const geometry_msgs::Point & point_nov,
    const geometry_msgs::Pose & car_pose) {

    const double yaw = tf::getYaw(car_pose.orientation);

    tf::Point tf_car_pos_nov;
    tf::pointMsgToTF(car_pose.position, tf_car_pos_nov);

    tf::Point tf_point_nov;
    tf::pointMsgToTF(point_nov, tf_point_nov);

    const tf::Vector3 z_axis_nov = {0, 0, 1};
    tf::Point tf_point_pov = (tf_point_nov - tf_car_pos_nov)\
        .rotate(z_axis_nov, -yaw);

    geometry_msgs::Point point_pov;
    tf::pointTFToMsg(tf_point_pov, point_pov);

    return point_pov;
}

common_msgs::StopLine convertStopLinePOVToNovatel(
    const common_msgs::StopLine & stop_line,
    const geometry_msgs::Pose & car_pose)
{
    geometry_msgs::Point left_nova = convertPOVToNovatel(stop_line.region[0], car_pose);
    geometry_msgs::Point right_nova = convertPOVToNovatel(stop_line.region[1], car_pose);
    common_msgs::StopLine stop_line_nova;
    stop_line_nova.region.push_back(left_nova);
    stop_line_nova.region.push_back(right_nova);
    
    return stop_line_nova;
}

std::vector<common_msgs::StopLine> convertStopLinesPOVToNovatel(
    const std::vector<common_msgs::StopLine> & stop_lines_pov,
    const geometry_msgs::Pose & car_pose)
{
    std::vector<common_msgs::StopLine> stop_lines_nova;
    for (const auto stop_line : stop_lines_pov) {
        stop_lines_nova.push_back(convertStopLinePOVToNovatel(stop_line, car_pose));
    }
    return stop_lines_nova;
}

common_msgs::StopLine convertStopLineNovatelToPOV(
    const common_msgs::StopLine & stop_line_nov,
    const geometry_msgs::Pose & car_pose) {
    geometry_msgs::Point left_nov = convertNovatelToPOV(stop_line_nov.region[0], car_pose);
    geometry_msgs::Point right_nov = convertNovatelToPOV(stop_line_nov.region[1], car_pose);
    common_msgs::StopLine stop_line_pov;
    stop_line_pov.region.push_back(left_nov);
    stop_line_pov.region.push_back(right_nov);
    return stop_line_pov;
}

std::vector<common_msgs::StopLine> convertStopLinesNovatelToPOV(
    const std::vector<common_msgs::StopLine> & stop_lines_nov,
    const geometry_msgs::Pose & car_pose) {
    std::vector<common_msgs::StopLine> stop_lines_pov;
    for (const auto stop_line : stop_lines_nov) {
        stop_lines_pov.push_back(convertStopLineNovatelToPOV(stop_line, car_pose));
    }
    return stop_lines_pov;
}

common_msgs::RoadLine convertRoadLinePOVToNovatel(
    const common_msgs::RoadLine & road_line_pov,
    const geometry_msgs::Pose & car_pose)
{
    common_msgs::RoadLine road_line_nova;
    for (const auto point : road_line_pov.line) {
        road_line_nova.line.push_back(convertPOVToNovatel(point, car_pose));
    }
    road_line_nova.visible = road_line_pov.visible;
    road_line_nova.type = road_line_pov.type;
    return road_line_nova;
}

std::vector<common_msgs::RoadLine> convertRoadLinesPOVToNovatel(
    const std::vector<common_msgs::RoadLine> & road_lines_pov,
    const geometry_msgs::Pose & car_pose)
{
    std::vector<common_msgs::RoadLine> road_lines_nova;
    for (const auto road_line :road_lines_pov) {
        road_lines_nova.push_back(convertRoadLinePOVToNovatel(road_line, car_pose));
    }
    return road_lines_nova;
}

common_msgs::RoadLine convertRoadLineNovatelToPOV(
    const common_msgs::RoadLine & road_line_nova,
    const geometry_msgs::Pose & car_pose)
{
    common_msgs::RoadLine road_line_pov;
    for (const auto point : road_line_nova.line) {
        road_line_pov.line.push_back(convertNovatelToPOV(point, car_pose));
    }
    return road_line_pov;
}

std::vector<common_msgs::RoadLine> convertRoadLinesNovatelToPOV(
    const std::vector<common_msgs::RoadLine> & road_lines_nova,
    const geometry_msgs::Pose & car_pose)
{
    std::vector<common_msgs::RoadLine> road_lines_pov;
    for (const auto road_line :road_lines_nova) {
        road_lines_pov.push_back(convertRoadLineNovatelToPOV(road_line, car_pose));
    }
    return road_lines_pov;
}

geometry_msgs::Quaternion convertBoundingBoxOrientationPOVToNovatel(
    const geometry_msgs::Quaternion & orientation,
    const geometry_msgs::Pose & car_pose)
{
    const double vehicle_azimuth = tf::getYaw(car_pose.orientation);
    // Lidar coordinate system has positive x facing forwards, add 90 degrees
    const double bounding_box_pov_euler = tf::getYaw(orientation) + (M_PI/2);
    const double bounding_box_nova_euler = bounding_box_pov_euler - (M_PI/2) + vehicle_azimuth;
    
    tf::Quaternion quat;
    quat.setEuler(bounding_box_nova_euler, 0, 0);

    geometry_msgs::Quaternion ret;
    tf::quaternionTFToMsg(quat, ret);

    return ret;
}

jsk_recognition_msgs::BoundingBox convertBoundingBoxLidarPOVToNovatel(
    const jsk_recognition_msgs::BoundingBox & bounding_box,
    const geometry_msgs::Pose & car_pose)
{
    jsk_recognition_msgs::BoundingBox bounding_box_nova = bounding_box;
    bounding_box_nova.pose.position.x -= lidar_to_bumper_distance;
    const geometry_msgs::Point center = bounding_box_nova.pose.position;
    // Lidar coordinate system has positive x pointing forwards
    bounding_box_nova.pose.position = convertPOVToNovatel(rotatePointCCW90Deg(center), car_pose);
    // Convert orientation given in lidar pov coordinate system to coventional azimuth
    const geometry_msgs::Quaternion orientation = bounding_box_nova.pose.orientation;
    bounding_box_nova.pose.orientation = convertBoundingBoxOrientationPOVToNovatel(orientation, car_pose);
    return bounding_box_nova;
}

cv::RotatedRect convertBoundingBoxToRotatedRect(
    const jsk_recognition_msgs::BoundingBox & bounding_box,
    const geometry_msgs::Pose & vehicle_pose)
{
    const double bounding_box_azimuth = tf::getYaw(bounding_box.pose.orientation);
    const double vehicle_azimuth = tf::getYaw(vehicle_pose.orientation);
    const double box_angle_euler_pov = (M_PI/2) - (vehicle_azimuth - bounding_box_azimuth);
    const geometry_msgs::Point box_center_real_pov = convertNovatelToPOV(bounding_box.pose.position, vehicle_pose);

    cv::RotatedRect rotated_rect(
        cv::Point(box_center_real_pov.x, box_center_real_pov.y),
        cv::Size(bounding_box.dimensions.x, bounding_box.dimensions.y),
        box_angle_euler_pov
        );

    return rotated_rect;
}
