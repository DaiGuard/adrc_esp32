#ifndef __CUSTUM_ROS_MANAGER_H__
#define __CUSTUM_ROS_MANAGER_H__

#include "RosManagerBase.h"

#include <rcl/publisher.h>
#include <std_msgs/msg/int32.h>


class CustomRosManager: public RosManagerBase
{
    public:
        CustomRosManager(): RosManagerBase(){}

        bool init_node(const char* node_name, const char* name_space);
        bool fini_node();

        bool publish_state();

    private:
        rcl_publisher_t status_pub;        
        std_msgs__msg__Int32 status_msg;
};


#endif