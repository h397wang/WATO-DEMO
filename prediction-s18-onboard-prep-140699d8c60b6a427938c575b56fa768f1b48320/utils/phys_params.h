#ifndef PHYS_PARAMS_H
#define PHYS_PARAMS_H

// horizontal distance between lidar mount and front bumper
static const double lidar_to_bumper_distance = 2.0; //meters

// camera has no vision of points whose y value < lower_y_bound_visible
static const double lower_y_bound_visible = 3.0; // meters

// stop tracking points whose y value < lower_y_bound_to_track
static const double lower_y_bound_to_track = -10.0; // meters

// match rate of points that are within bounds, between two road line objects, to qualify as matched road lines
static const double match_threshold = 0.5; // out of 1

// max distance between points to qualify as being from the same road line
static const double max_delta = 1.5; // meters

// when matching road lines, only examine points above the invisible threshold, and within a certain bound
static const double upper_y_bound_to_match_check = 10; // meters

// used to increase the point density during road line match making process
static const double meters_per_point = 0.1; // meters

// match distance of bounding box properties between 2 bounding boxes
static const double bounding_box_match_threshold = 1.5;

// min distance between segments of points in road lines to be classified as dash line
static const double min_dashed_gap_distance = 1.8; // meters

#endif // PHYS_PARAMS_H