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
bool RosManagerBase::begin(HardwareSerial& serial)
{
    set_microros_serial_transports(serial);    
    delay(2000);

    return true;
}
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
bool RosManagerBase::begin(const char* ssid, const char* psk, IPAddress& ip, size_t port)
{
    set_microros_wifi_transports((char*)ssid, (char*)psk, ip, port);

    return true;
}
#endif

bool RosManagerBase::init_node(const char* node_name, const char* name_space, int handnum, int domain_id)
{
    rcl_ret_t ret;

    allocator = rcl_get_default_allocator();

    options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&options, allocator);
    if(ret != RCL_RET_OK) {
        return false;
    }
    ret = rcl_init_options_set_domain_id(&options, domain_id);
    if(ret != RCL_RET_OK) {
        return false;
    }
    
    // ret = rclc_support_init(&support, 0, NULL, &allocator);
    // if(ret != RCL_RET_OK) {
    //     return false;
    // }        
    ret = rclc_support_init_with_options(&support, 0, NULL, &options, &allocator);
    if(ret != RCL_RET_OK) {
        return false;
    }        

    ret = rclc_node_init_default(&node, node_name, name_space, &support);
    if(ret != RCL_RET_OK) {
        return false;
    }

    ret = rclc_executor_init(&executor, &support.context, handnum, & allocator);
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
    ret = rcl_init_options_fini(&options);

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
