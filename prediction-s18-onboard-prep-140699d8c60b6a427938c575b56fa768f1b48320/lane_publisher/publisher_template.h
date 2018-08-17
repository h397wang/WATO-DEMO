#ifndef PUBLISHER_TEMPLATE_H
#define PUBLISHER_TEMPLATE_H

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <path_planning_msgs/Environment.h>
#include <perception_msgs/FrameDetections.h>
#include <geometry_msgs/Pose.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <perception_msgs/LaneDetectionOutput.h>

#include <cstring>
#include <string>
#include <fstream>
#include <exception>
#include <queue>
#include <sstream>
#include <math.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp> 
#include <boost/algorithm/string.hpp>

template<typename MsgType>
class PublisherTemplate
{
public:

    PublisherTemplate(ros::NodeHandle& node
        , const std::string topic_name
        , const double fundamental_period = 1.0
        , const double publishing_period = 1.0);

    int readInputFile(const std::string& filename);
    
    void startPublishing();

private:
    ros::NodeHandle node_;
    ros::Publisher publisher_;
    ros::Timer timer_;
    std::queue<MsgType> msg_q_;
    bool has_input_ = false;
    double fundamental_period_;
    double publishing_period_;
    unsigned publish_mod_;
    unsigned publish_count_ = 0;

    void publisherCallback(const ros::TimerEvent& timer);

    // To be explicitly specialized
    MsgType convertLineToMessage(const std::string line);
};

template<typename MsgType>
PublisherTemplate<MsgType>::PublisherTemplate(
    ros::NodeHandle& node,
    const std::string topic_name,
    const double fundamental_period,
    const double publishing_period)
    :
    node_(node),
    publisher_(node.advertise<MsgType>(topic_name, 1)),
    fundamental_period_(fundamental_period),
    publishing_period_(publishing_period)
{
    publish_mod_ = static_cast<unsigned>(ceil(publishing_period / fundamental_period));
    publish_count_ = 0;
}

template<typename MsgType>
int PublisherTemplate<MsgType>::readInputFile(const std::string& filename)
{
    ROS_INFO("Reading input file: %s", filename.c_str());

    std::ifstream file(filename.c_str(), std::ios::in);
    if (file.fail())
    {
        std::cerr << "Could not read input file " << filename << std::endl;
        return -1;
    }
    std::string line;
    while (getline(file, line))
    {
        if (line != "") {
            MsgType msg = convertLineToMessage(line);
            msg_q_.push(msg);
        }
    }

    ROS_INFO("Done reading input file %s", filename.c_str());
    has_input_ = true;
    return 0;
}


template<typename MsgType>
void PublisherTemplate<MsgType>::startPublishing()
{
    if (has_input_)
    {
        timer_ = node_.createTimer(ros::Duration(fundamental_period_), &PublisherTemplate<MsgType>::publisherCallback, this);
    }
    else 
    {
        ROS_INFO("No data to publish, needs to first readInputFile. ");
    }
}

template<typename MsgType>
void PublisherTemplate<MsgType>::publisherCallback(const ros::TimerEvent& timer)
{
    if (!msg_q_.empty())
    {   
        if (publish_count_ % publish_mod_ == 0) {
            MsgType msg = msg_q_.front();
            publisher_.publish(msg);
        }
        msg_q_.pop();
        publish_count_++;
    }
}

// Helper parsing functions
void consumeToken(std::vector<std::string> & tokens, const std::string token)
{
    if (tokens.front().compare(token) == 0) {
        tokens.erase(tokens.begin());
    } else {
        ROS_INFO("Invalid grammar, expected token %s, got %s", token.c_str(), tokens.front().c_str());
        throw std::exception();
    }
}

geometry_msgs::Point parsePoint(std::vector<std::string> & tokens)
{
    geometry_msgs::Point point;
    consumeToken(tokens, "(");

    point.x = std::stod(tokens.front());
    tokens.erase(tokens.begin());
    
    point.y = std::stod(tokens.front());
    tokens.erase(tokens.begin());
    
    point.z = 0;
    
    consumeToken(tokens, ")");
    return point;
}

common_msgs::RoadLine parseRoadLine(std::vector<std::string> & tokens)
{
    common_msgs::RoadLine road_line;
    consumeToken(tokens, "[");

    while(tokens.front().compare("]") != 0)
    {
        geometry_msgs::Point point = parsePoint(tokens);
        road_line.line.push_back(point);
    }
    road_line.type = 0;

    consumeToken(tokens, "]");

    return road_line;
}

jsk_recognition_msgs::BoundingBox parseBoundingBox(std::vector<std::string> & tokens)
{
    jsk_recognition_msgs::BoundingBox bounding_box;        
    try {
        consumeToken(tokens, "(");

        double left = std::stod(tokens.front());
        tokens.erase(tokens.begin());

        double top = std::stod(tokens.front());
        tokens.erase(tokens.begin());

        double right = std::stod(tokens.front());
        tokens.erase(tokens.begin());

        double bottom = std::stod(tokens.front());
        tokens.erase(tokens.begin());

        consumeToken(tokens, ")");

        geometry_msgs::Point pos;
        pos.x = (right - left)/2;
        pos.y = (top - bottom)/2;
        pos.z = 0;

        bounding_box.pose.position = pos;
    } catch (std::exception e) {
        throw e;
    }
    return bounding_box;
}

// Explicit template specializations for message types
template<>
sensor_msgs::Imu PublisherTemplate<sensor_msgs::Imu>::convertLineToMessage(const std::string line)
{
    const int num_vals = 6;
    sensor_msgs::Imu imu;
    if (line != "") {
        std::vector<std::string> tokens;
        std::string delim = ",";
        boost::split(tokens, line, boost::is_any_of(delim));
        
        if (tokens.size() != num_vals) {
            ROS_INFO("Invalid grammar, expected %d values per line, delim = %s", num_vals, delim.c_str());
            throw std::exception();
        }

        imu.linear_acceleration.x = std::stod(tokens[0]);
        imu.linear_acceleration.y = std::stod(tokens[1]);
        imu.linear_acceleration.z = std::stod(tokens[2]);
        imu.angular_velocity.x = std::stod(tokens[3]);
        imu.angular_velocity.y = std::stod(tokens[4]);
        imu.angular_velocity.z = std::stod(tokens[5]);
    }
    return imu;
}

template<>
perception_msgs::LaneDetectionOutput PublisherTemplate<perception_msgs::LaneDetectionOutput>::convertLineToMessage(const std::string line)
{
  perception_msgs::LaneDetectionOutput lane_detection_output;
  std::vector<common_msgs::RoadLine> road_lines;

  if (line != "") {
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" "));

    while (!tokens.empty()) {
      common_msgs::RoadLine road_line = parseRoadLine(tokens);
      road_lines.push_back(road_line);
    }

    lane_detection_output.lanes = road_lines;
  }

  return lane_detection_output;
}

template<>
nav_msgs::Odometry PublisherTemplate<nav_msgs::Odometry>::convertLineToMessage(const std::string line)
{   
    const int num_tokens = 7;
    nav_msgs::Odometry odometry;

    if (line != "") {
        std::vector<std::string> tokens;
        std::string delim = ",";
        boost::split(tokens, line, boost::is_any_of(delim));
        
        if (tokens.size() != num_tokens) {
            ROS_INFO("Invalid grammar, expected %d tokens per line, delim = %s", num_tokens, delim.c_str());
            throw std::exception();
        }

        geometry_msgs::Vector3 linear_vel;
        linear_vel.x = std::stod(tokens[0]);
        linear_vel.y = std::stod(tokens[1]);
        linear_vel.z = std::stod(tokens[2]);
        odometry.twist.twist.linear = linear_vel;

        geometry_msgs::Quaternion orientation;
        orientation.x = std::stod(tokens[3]);
        orientation.y = std::stod(tokens[4]);
        orientation.z = std::stod(tokens[5]);
        orientation.w = std::stod(tokens[6]);
        odometry.pose.pose.orientation = orientation;
    }

    return odometry;
}

template<>
common_msgs::StopLine PublisherTemplate<common_msgs::StopLine>::convertLineToMessage(const std::string line)
{
    common_msgs::StopLine stop_line;
    geometry_msgs::Point left_point;
    geometry_msgs::Point right_point;
    left_point.x = 0;
    left_point.y = 0;
    left_point.z = 0;

    right_point.x = 0;
    right_point.y = 0;
    right_point.z = 0;

    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" "));

    if (tokens.size() == 4) {
        left_point.x = std::stod(tokens.front());
        tokens.erase(tokens.begin());
        left_point.y = std::stod(tokens.front());
        tokens.erase(tokens.begin());
        left_point.z = 0;

        right_point.x = std::stod(tokens.front());
        tokens.erase(tokens.begin());
        right_point.y = std::stod(tokens.front());
        tokens.erase(tokens.begin());
        right_point.z = 0;

    } else {
        ROS_INFO("Invalid grammar, expected 4 tokens, got %lu", tokens.size());
        for (int i = 0; i < tokens.size(); i++) {
            ROS_INFO("%s, ", tokens[i].c_str());
        }
        throw std::exception();
    }

    stop_line.region.push_back(left_point);
    stop_line.region.push_back(right_point);       
    return stop_line;
}

template<>
geometry_msgs::Pose PublisherTemplate<geometry_msgs::Pose>::convertLineToMessage(const std::string line)
{
    geometry_msgs::Pose pose;
    return pose;
}

template<>
jsk_recognition_msgs::BoundingBoxArray PublisherTemplate<jsk_recognition_msgs::BoundingBoxArray>::convertLineToMessage(const std::string line)
{
    jsk_recognition_msgs::BoundingBoxArray bounding_box_list;
    std::vector<jsk_recognition_msgs::BoundingBox> bounding_boxes;

    if (line != "") {
        std::vector<std::string> tokens;
        boost::split(tokens, line, boost::is_any_of(" "));
        
        while (!tokens.empty())
        {
            jsk_recognition_msgs::BoundingBox bounding_box = parseBoundingBox(tokens);
            bounding_boxes.push_back(bounding_box);
        }
        bounding_box_list.boxes = bounding_boxes;
    }
    return bounding_box_list;
}
#endif    // #ifndef PUBLISHER_TEMPLATE_H