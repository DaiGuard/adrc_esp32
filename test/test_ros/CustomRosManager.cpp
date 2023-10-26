#include "CustomRosManager.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>


bool CustomRosManager::init_node(const char* node_name, const char* name_space)
{
    if(!RosManagerBase::init_node(node_name, name_space))
    {
        return false;
    }

    // ユーザーの処理を書く
    rcl_ret_t ret;
    ret = rclc_publisher_init_default(
        &status_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "status_esp");
    //

    return true;
}

bool CustomRosManager::fini_node()
{
    // ユーザーの処理を書く
    rcl_ret_t ret;
    ret = rcl_publisher_fini(&status_pub, &node);
    //    

    RosManagerBase::fini_node();

    return true;
}


bool CustomRosManager::publish_state()
{
    rcl_ret_t ret;

    status_msg.data++;
    ret = rcl_publish(&status_pub, &status_msg, NULL);
    if(ret != RCL_RET_OK){
        return false;
    }

    return true;
}