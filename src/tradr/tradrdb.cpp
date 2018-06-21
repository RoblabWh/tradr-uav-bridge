#include "tradrdb.h"

TradrDB::TradrDB(ros::NodeHandle* nodeHandle)
{
    this->nodeHandle = nodeHandle;
}

/*
int TradrDB::registerUav(bridge_path_interface_msgs::Uav msg)
{
    bridge_path_interface_msgs::putUavRegistration putMsg;
    putMsg.request.data = msg;
    if (ros::service::call(registrationTopicName + "/put", putMsg))
    {
        int id = atoi(putMsg.response.object_id.c_str());

        return id;
    }
    else
    {
        return 0;
    }
}

*/
