#ifndef TRADRDB_H
#define TRADRDB_H

#include <ros/ros.h>

#include <bridge_path_interface_msgs/UavIdent.h>

#include <string>

class TradrDB
{

private:
    ros::NodeHandle* nodeHandle;    //shared

    std::string registrationTopicName;

    ros::ServiceClient registrationServiceClient;

public:
    TradrDB(ros::NodeHandle* nodeHandle);



    //int registerUav(bridge_path_interface_msgs::Uav msg);
};

#endif // TRADRDB_H
