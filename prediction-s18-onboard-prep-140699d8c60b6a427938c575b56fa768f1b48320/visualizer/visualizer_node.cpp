#include <signal.h>
#include <ros/ros.h>
#include <visualizer/visualizer.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle node;
   	Visualizer visualizer;

	ros::Subscriber sub = node.subscribe("/environment_prediction", 1, &Visualizer::updateWindow, &visualizer);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
