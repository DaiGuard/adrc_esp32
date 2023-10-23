#ifndef __ROS_MANAGER_H__
#define __ROS_MANAGER_H__

#include <rcl/allocator.h>
#include <rcl/node.h>
#include <rclc/types.h>
#include <rclc/timer.h>
#include <rclc/executor.h>

#include <HardwareSerial.h>


class RosManagerBase
{
    public:
        RosManagerBase();

        void begin(HardwareSerial& serial);

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


#include <rcl/publisher.h>
#include <std_msgs/msg/int32.h>

class RosManager: public RosManagerBase
{
    public:
        RosManager(): RosManagerBase(){}

        void begin(HardwareSerial& serial){ RosManagerBase::begin(serial); }

        bool init_node(const char* node_name, const char* name_space);
        bool fini_node();

        bool publish_state();

    private:
        rcl_publisher_t status_pub;        
        std_msgs__msg__Int32 status_msg;
};

#endif