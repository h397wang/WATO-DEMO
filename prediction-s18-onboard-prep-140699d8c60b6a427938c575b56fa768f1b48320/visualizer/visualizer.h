#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <opencv2/opencv.hpp>
#include <path_planning_msgs/Environment.h>
#include <common_msgs/RoadLine.h>
#include <common_msgs/StopLine.h>

#include <utils/utils.h>

class Visualizer
{
public:

    Visualizer();
    void updateWindow(const path_planning_msgs::Environment &);

private:

    const std::string WINDOW_NAME = "Visualizer";
    
    cv::Mat background_;

    const int WINDOW_WIDTH = 540;
    const int WINDOW_HEIGHT = 640;

    const float pixels_per_meter_float = 15;
    const int pixels_per_meter_int = pixels_per_meter_float;

    // real space origin mapped to image space coordinates
    const int real_to_image_offset_x = 270;
    const int real_to_image_offset_y = 420;

    // drawing constants
    const int radius = 4;
    const int thickness = 3;
    const int tick_pixel_width = 5;
    const int tick_x_0 = real_to_image_offset_x - tick_pixel_width;
    const int tick_x_1 = real_to_image_offset_x + tick_pixel_width;
    const int tick_y_0 = real_to_image_offset_y - tick_pixel_width;
    const int tick_y_1 = real_to_image_offset_y + tick_pixel_width;
    
    const cv::Scalar bounding_box_color_ = cv::Scalar(0, 255, 255); // yellow
    const cv::Scalar stop_line_color_ = cv::Scalar(0, 0, 255); // red
    std::vector<cv::Scalar> road_line_colors_;

    cv::RNG rng_;

    cv::Point2i convertPointRealToImage(const cv::Point2f &);
    cv::Point2f convertPointImageToReal(const cv::Point2i &);
    std::vector<cv::Point2i> convertRoadLineToPoints(const common_msgs::RoadLine &);
    std::vector<cv::Point2i> convertStopLineToPoints(const common_msgs::StopLine &);
};

#endif // VISUALIZER_H



