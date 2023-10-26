#ifndef __ROS_MANAGER_H__
#define __ROS_MANAGER_H__

#include <rcl/allocator.h>
#include <rcl/node.h>
#include <rclc/types.h>
#include <rclc/timer.h>
#include <rclc/executor.h>

#include <HardwareSerial.h>
#include <IPAddress.h>


class RosManagerBase
{
    public:
        RosManagerBase();

        #if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
        void begin(HardwareSerial& serial);
        #elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
        void begin(const char* ssid, const char* psk, IPAddress& ip, size_t port);
        #endif

        bool init_node(const char* node_name, const char* name_space);
        bool fini_node();

        bool is_initialized(){ return init_comp; }

        bool spin_once(uint32_t msec);

    protected:
        // ROS関連変数
        rclc_executor_t executor;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        rcl_timer_t timer;

        // 状態変数
        bool init_comp;
};


#endif