#include <visualizer/visualizer.h>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>

Visualizer::Visualizer() :
    rng_(100)
{
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT);

    background_ = cv::Mat(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

    // draw horizontal axis
    cv::line(background_
        , cv::Point(0, real_to_image_offset_y)
        , cv::Point(WINDOW_WIDTH, real_to_image_offset_y)
        , cv::Scalar(255, 255, 255)
        , 1);

    // draw vertical axis
    cv::line(background_
        , cv::Point(real_to_image_offset_x, 0)
        , cv::Point(real_to_image_offset_x, WINDOW_HEIGHT)
        , cv::Scalar(255, 255, 255)
        , 1
        );

    // draw horizontal ticks on positive y (real space) axis
    for (int y = real_to_image_offset_y - pixels_per_meter_int; y > 0; y -= pixels_per_meter_int)
    {
        cv::line(background_
            , cv::Point(tick_x_0, y)
            , cv::Point(tick_x_1, y)
            , cv::Scalar(255, 255, 255)
            , 1
            );
    }

    // draw horizontal ticks on negative y (real space) axis
    for (int y = real_to_image_offset_y + pixels_per_meter_int; y < WINDOW_HEIGHT; y += pixels_per_meter_int)
    {
        cv::line(background_
            , cv::Point(tick_x_0, y)
            , cv::Point(tick_x_1, y)
            , cv::Scalar(255, 255, 255)
            , 1
            );
    }

    // draw vertical ticks on positive x (real space) axis
    for (int x = real_to_image_offset_x + pixels_per_meter_int; x < WINDOW_WIDTH; x += pixels_per_meter_int)
    {
        cv::line(background_
            , cv::Point(x, tick_y_0)
            , cv::Point(x, tick_y_1)
            , cv::Scalar(255, 255, 255)
            , 1
            );
    }

    // draw vertical ticks on negative x (real space) axis
    for (int x = real_to_image_offset_x - pixels_per_meter_int; x > 0; x -= pixels_per_meter_int)
    {
        cv::line(background_
            , cv::Point(x, tick_y_0)
            , cv::Point(x, tick_y_1)
            , cv::Scalar(255, 255, 255)
            , 1
            );
    }

    road_line_colors_.push_back(cv::Scalar(100, 100, 255)); // salmon
    road_line_colors_.push_back(cv::Scalar(100, 175, 255)); // orange
    road_line_colors_.push_back(cv::Scalar(100, 255, 255)); // yellow
    road_line_colors_.push_back(cv::Scalar(100, 255, 170)); // green
    road_line_colors_.push_back(cv::Scalar(255, 255, 100)); // teal
    road_line_colors_.push_back(cv::Scalar(255, 100, 255)); // violet
    road_line_colors_.push_back(cv::Scalar(155, 155, 155)); // grey
}

void Visualizer::updateWindow(const path_planning_msgs::Environment & env)
{   
    cv::Mat new_frame = background_.clone();

    std::vector<common_msgs::RoadLine> road_lines_pov = convertRoadLinesNovatelToPOV(env.lanes, env.global_pose);
    for (size_t i = 0; i < road_lines_pov.size(); i++) {
        cv::Scalar color;
        if (i >= road_line_colors_.size()) {
            color = cv::Scalar(rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255));
            road_line_colors_.push_back(color);
        } else {
            color = road_line_colors_[i];
        }

        std::vector<cv::Point2i> road_line_points = convertRoadLineToPoints(road_lines_pov[i]);
        for (const auto point: road_line_points) {
            circle(new_frame
                , point
                , radius
                , color
                , thickness);
        }
    }

    std::vector<common_msgs::StopLine> stop_lines_pov = convertStopLinesNovatelToPOV(env.stop_lines, env.global_pose);
    for (const auto stop_line : stop_lines_pov) {
        const std::vector<cv::Point2i> stop_line_points = convertStopLineToPoints(stop_line);
        cv::line(new_frame
            , stop_line_points[0]
            , stop_line_points[1]
            , stop_line_color_
            , thickness
            );
    }

    for (const auto bounding_box : env.obstacles) {
        const cv::RotatedRect rotated_rect = convertBoundingBoxToRotatedRect(bounding_box, env.global_pose);
        cv::Point2f vertices2f[4];
        rotated_rect.points(vertices2f);

        cv::Point2i vertices2i[4];
        for (int i = 0; i < 4; i++){
            vertices2i[i] = convertPointRealToImage(vertices2f[i]);
        }

        for (int i = 0; i < 4; i++ )
            line(new_frame
            , vertices2i[i]
            , vertices2i[(i + 1) % 4]
            , bounding_box_color_
            , thickness
            );
    }

    cv::imshow(WINDOW_NAME, new_frame);
    cv::waitKey(1);
}

cv::Point2i Visualizer::convertPointRealToImage(const cv::Point2f & point_real_space)
{
    const int x_image_space = (pixels_per_meter_float * point_real_space.x) + real_to_image_offset_x;
    const int y_image_space = -(pixels_per_meter_float * point_real_space.y) + real_to_image_offset_y;
    const cv::Point point_image_space = cv::Point(x_image_space, y_image_space);
    return point_image_space;
}

cv::Point2f Visualizer::convertPointImageToReal(const cv::Point2i & point_image_space)
{   
    const float x_real_space = (point_image_space.x - real_to_image_offset_x) / pixels_per_meter_float;
    const float y_real_space = (-point_image_space.y + real_to_image_offset_x) / pixels_per_meter_float;
    const cv::Point2f point_real_space = cv::Point2f(x_real_space, y_real_space);
    return point_real_space;
}

std::vector<cv::Point2i> Visualizer::convertRoadLineToPoints(const common_msgs::RoadLine & road_line)
{
    std::vector<cv::Point2i> ret;
    std::vector<geometry_msgs::Point> points = road_line.line;

    for (const auto point : points) {
        const cv::Point2f point_real_space(point.x, point.y);
        const cv::Point2i point_image_space = convertPointRealToImage(point_real_space);
        ret.push_back(point_image_space);
    }

    return ret;
}

std::vector<cv::Point2i> Visualizer::convertStopLineToPoints(const common_msgs::StopLine & stop_line)
{
    std::vector<cv::Point2i> stop_line_end_points;

    const geometry_msgs::Point point_left = stop_line.region[0];
    const geometry_msgs::Point point_right = stop_line.region[1];

    const cv::Point2f point_left_real_space(point_left.x, point_left.y);
    const cv::Point2i point_left_image_space = convertPointRealToImage(point_left_real_space);
    stop_line_end_points.push_back(point_left_image_space);
    
    
    const cv::Point2f point_right_real_space(point_right.x, point_right.y);
    const cv::Point2i point_right_image_space = convertPointRealToImage(point_right_real_space);
    stop_line_end_points.push_back(point_right_image_space);
    
    return stop_line_end_points;
}