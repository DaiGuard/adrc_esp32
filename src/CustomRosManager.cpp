#include "CustomRosManager.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

#include <std_msgs/msg/int32.h>


bool CustomRosManager::init_node(const char* node_name, const char* name_space, int domain_id)
{
    if(!RosManagerBase::init_node(node_name, name_space, domain_id))
    {
        return false;
    }

    // ユーザーの処理を書く
    rcl_ret_t ret;
    ret = rclc_publisher_init_default(
        &status_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "status_esp32");
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rclc_publisher_init_default(
        &pose_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "pose_esp32");
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rclc_publisher_init_default(
        &cur_vel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cur_vel");
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rclc_service_init_default(
        &reset_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "reset_esp32"
    );
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        CustomRosManager::cb_cmd_vel,
        ON_NEW_DATA);
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rclc_executor_add_service(
        &executor,
        &reset_srv,
        &reset_request_msg,
        &reset_response_msg,
        CustomRosManager::cb_reset
    );
    if(ret != RCL_RET_OK){
        return false;
    }

    rosidl_runtime_c__String__assign(&Ros.pose_msg.header.frame_id, "map");
    //

    return true;
}

bool CustomRosManager::fini_node()
{
    // ユーザーの処理を書く
    rcl_ret_t ret;
    ret = rclc_executor_remove_subscription(&executor, &cmd_vel_sub);
    ret = rcl_subscription_fini(&cmd_vel_sub, &node);
    ret = rclc_executor_remove_service(&executor, &reset_srv);
    ret = rcl_service_fini(&reset_srv, &node);
    ret = rcl_publisher_fini(&status_pub, &node);
    ret = rcl_publisher_fini(&pose_pub, &node);
    ret = rcl_publisher_fini(&cur_vel_pub, &node);    
    //

    RosManagerBase::fini_node();

    return true;
}


bool CustomRosManager::publish_all()
{
    rcl_ret_t ret;
    
    ret = rcl_publish(&status_pub, &status_msg, NULL);
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rcl_publish(&cur_vel_pub, &cur_vel_msg, NULL);
    if(ret != RCL_RET_OK){
        return false;
    }

    ret = rcl_publish(&pose_pub, &pose_msg, NULL);
    if(ret != RCL_RET_OK){
        return false;
    }

    return true;
}


// ROSマネージャクラス
CustomRosManager Ros;


void CustomRosManager::cb_cmd_vel(const void* msgin)
{
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist *)msgin;

    Ros.cmd_vel_msg.linear.x = msg->linear.x;
    Ros.cmd_vel_msg.linear.y = msg->linear.y;
    Ros.cmd_vel_msg.linear.z = msg->linear.z;
    Ros.cmd_vel_msg.angular.x = msg->angular.x;
    Ros.cmd_vel_msg.angular.y = msg->angular.y;
    Ros.cmd_vel_msg.angular.z = msg->angular.z;

    Serial.println("CMD_VEL");
}


void CustomRosManager::cb_reset(const void* request, void* response)
{
    const std_srvs__srv__Trigger_Request* req = (const std_srvs__srv__Trigger_Request*)request;
    std_srvs__srv__Trigger_Response* res = (std_srvs__srv__Trigger_Response*)response;

    Serial.println("RESET");

    res->success = true;
}