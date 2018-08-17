#include <signal.h>
#include <ros/ros.h>
#include <tracker/tracker.h>
#include <std_msgs/String.h>

sig_atomic_t volatile requestedShutdown = 0;
void sigIntHandler(int sig);

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tracker", ros::init_options::NoSigintHandler);
    signal(SIGINT, sigIntHandler);
    ros::NodeHandle node;

    bool passthrough;
    if (!node.getParam("prediction/passthrough", passthrough)) {
        passthrough = false;
    }

    Tracker tracker(node, passthrough);

    while (!requestedShutdown && ros::ok()){
        ros::spinOnce();
    }

    return 0;
}

void sigIntHandler(int sig) {
    requestedShutdown = 1;
}