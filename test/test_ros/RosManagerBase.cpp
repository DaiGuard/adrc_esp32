#include "RosManagerBase.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


RosManagerBase::RosManagerBase()
{
    init_comp = false;
}


#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
void RosManagerBase::begin(HardwareSerial& serial)
{
    set_microros_serial_transports(serial);    
    delay(2000);
}
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
void RosManagerBase::begin(const char* ssid, const char* psk, IPAddress& ip, size_t port)
{
    set_microros_wifi_transports((char*)ssid, (char*)psk, ip, port);
}
#endif

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
