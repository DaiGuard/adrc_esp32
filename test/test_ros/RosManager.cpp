#include "RosManager.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif


RosManagerBase::RosManagerBase()
{
    init_comp = false;
}


void RosManagerBase::begin(HardwareSerial& serial)
{
    set_microros_serial_transports(serial);
    delay(2000);
}


bool RosManagerBase::init_node(const char* node_name, const char* name_space)
{
    rcl_ret_t ret;

    allocator = rcl_get_default_allocator();
    
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if(ret != RCL_RET_OK) {
        return false;
    }        

    ret = rclc_node_init_default(&node, node_name, name_space, &support);
    if(ret != RCL_RET_OK) {
        return false;
    }

    ret = rclc_executor_init(&executor, &support.context, 1, & allocator);
    if(ret != RCL_RET_OK) {
        return false;
    }

    init_comp = true;

    return true;
}


bool RosManagerBase::fini_node()
{
    rcl_ret_t ret;
    ret = rcl_node_fini(&node);
    ret = rclc_executor_fini(&executor);
    ret = rclc_support_fini(&support);

    init_comp = false;

    return true;
}


bool RosManagerBase::spin_once(uint32_t msec)
{   
    rcl_ret_t ret;
    ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(msec));
    if(ret != RCL_RET_OK){
        return false;
    }

    return true;
}


bool RosManager::init_node(const char* node_name, const char* name_space)
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

bool RosManager::fini_node()
{
    // ユーザーの処理を書く
    rcl_ret_t ret;
    ret = rcl_publisher_fini(&status_pub, &node);
    //    

    RosManagerBase::fini_node();

    return true;
}


bool RosManager::publish_state()
{
    rcl_ret_t ret;

    status_msg.data++;
    ret = rcl_publish(&status_pub, &status_msg, NULL);
    if(ret != RCL_RET_OK){
        return false;
    }

    return true;
}