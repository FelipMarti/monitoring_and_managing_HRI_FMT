#include "ros/ros.h"
#include "data_parser/Parse.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "interaction_monitor");
    if (argc != 2) {
        ROS_ERROR("[interaction_monitor]: usage: interaction_monitor PATH_FILE");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<data_parser::Parse>("parse_data");
    data_parser::Parse srv;
    srv.request.path = argv[1];

    if (client.call(srv)) {
        ROS_INFO("[interaction_monitor]: Calling parse_data_srv");
    }
    else {
        ROS_ERROR("[interaction_monitor]: Failed to call service parse_data_srv");
        return 1;
    }

    return 0;
}

