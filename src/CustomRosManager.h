#ifndef __CUSTUM_ROS_MANAGER_H__
#define __CUSTUM_ROS_MANAGER_H__

#include "RosManagerBase.h"

#include <rcl/publisher.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <std_srvs/srv/trigger.h>

class CustomRosManager: public RosManagerBase
{
    public:
        CustomRosManager(): RosManagerBase(){}

        bool init_node(const char* node_name, const char* name_space, int domain_id=0);
        bool fini_node();

        bool publish_all();

        std_msgs__msg__Int32 status_msg;
        geometry_msgs__msg__PoseStamped pose_msg;
        geometry_msgs__msg__Twist cmd_vel_msg;
        geometry_msgs__msg__Twist cur_vel_msg;
        // std_msgs__msg__Int32 reset_msg;
        std_srvs__srv__Trigger_Response reset_response_msg;
        std_srvs__srv__Trigger_Request reset_request_msg;        

    private:
        rcl_publisher_t status_pub;
        rcl_publisher_t pose_pub;
        rcl_publisher_t cur_vel_pub;
        rcl_subscription_t cmd_vel_sub;
        // rcl_subscription_t reset_sub;
        rcl_service_t reset_srv;

        static void cb_cmd_vel(const void* msgin);
        // static void cb_reset(const void* msgin);
        static void cb_reset(const void* request, void* response);
};


extern CustomRosManager Ros;

#endif