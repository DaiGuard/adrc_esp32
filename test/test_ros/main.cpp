#include <Arduino.h>

#include "CustomRosManager.h"


CustomRosManager ros_manager;

// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/int32.h>
// #include <geometry_msgs/msg/twist.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// void error_one(const char* msg)
// {
//     Serial.println(msg);    
// }
// void error_loop(const char* msg) {
//   while(1) {
//     Serial.println(msg);
//     delay(100);
//   }
// }

// #define RCL_CHECK(fn, s) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(s);}}
// #define RCL_SOFT_CHECK(fn, s) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_one(s);}}

// // ROS関連変数
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;
// rcl_publisher_t status_pub;
// rcl_subscription_t cmd_vel_sub;
// std_msgs__msg__Int32 status_msg;
// geometry_msgs__msg__Twist cmd_vel_msg;


/**
 * @brief 指示速度取得(ROSコールバック)
 * 
 */
// void cmd_vel_callback(const void* msgin)
// {
//     const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist *)msgin;
// }


// namespace ros {

//     bool init_complate = false;
//     bool first_init = false;

//     bool init_node(const char* node_name, const char* name_space) {
//         rcl_ret_t ret;

//         allocator = rcl_get_default_allocator();
        
//         ret = rclc_support_init(&support, 0, NULL, &allocator);
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to initialize support");
//             return false;
//         }        

//         ret = rclc_node_init_default(&node, node_name, name_space, &support);
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to initialize node");
//             return false;
//         }

//         ret = rclc_publisher_init_default(
//             &status_pub,
//             &node,
//             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//             "status_esp");
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to initialize state publisher");
//             return false;
//         }
        
//         ret = rclc_executor_init(&executor, &support.context, 1, & allocator);
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to initialize executor");
//             return false;
//         }

//         init_complate = true;
//         first_init = true;

//         return true;
//     }

//     bool fini_node(){
//         rcl_ret_t ret;

//         ret = rcl_publisher_fini(&status_pub, &node);
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to finish publisher");
//             // return false;
//         }

//         ret = rcl_node_fini(&node);
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to finish node");
//             // return false;
//         }

//         ret = rclc_executor_fini(&executor);
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to finish executor");
//             // return false;
//         }

//         ret = rclc_support_fini(&support);
//         if(ret != RCL_RET_OK) {
//             Serial.println("unnable to finish support");
//             // return false;
//         }

//         init_complate = false;

//         return true;
//     }
// }

void setup()
{    
    // シリアルデバック用設定
    Serial.begin(115200);

    // ROS初期化
    Serial2.begin(115200);
    ros_manager.begin(Serial2);
    // ros_manager.begin(ssid, psk, ip, port);

    // set_microros_serial_transports(Serial2);
    // delay(2000);

//     // allocator = rcl_get_default_allocator();    
//     // RCL_CHECK(rclc_support_init(&support, 0, NULL, &allocator),
//     //     "rcl support initialize error");
//     // RCL_CHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support),
//     //     "rcl node initialize error");
//     // // rcl_node_options_t node_ops = rcl_node_get_default_options();
//     // // node_ops.domain_id = 1;
//     // // rclc_node_init_with_options(&node, "jetbot_pico", "", &support, &node_ops);

//     // RCL_CHECK(rclc_subscription_init_default(
//     //     &cmd_vel_sub,
//     //     &node,
//     //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
//     //     "/cmd_vel"), 
//     //     "cmd_vel subscribe initialize error");

//     // RCL_CHECK(rclc_executor_init(&executor, &support.context, 1, & allocator),
//     //     "rcl executor initialize error");

//     // RCL_CHECK(rclc_executor_add_subscription(
//     //     &executor,
//     //     &cmd_vel_sub,
//     //     &cmd_vel_msg,
//     //     &cmd_vel_callback,
//     //     ON_NEW_DATA),
//     //     "regist subscribe error");
}


void loop()
{    
    if(!ros_manager.is_initialized()){
        if(ros_manager.init_node("adrc_esp32", "")){
            Serial.println("initialize comp");
        }
        else{
            Serial.println("initialize failed");
        }
    }
    else{
        if(!ros_manager.publish_state()){
            Serial.println("failed to publish state");
            ros_manager.fini_node();
            return;
        }
        if(!ros_manager.spin_once(10)){
            Serial.println("failed to execute spin");
            ros_manager.fini_node();
            return;
        }
    }

    // if(!ros::init_complate) {
    //     if(ros::init_node("adrc_eps32", "")){
    //         Serial.println("initialize comp");            
    //     }        
    // }
    // else{
    //     rcl_ret_t ret;

    //     status_msg.data++;
    //     ret = rcl_publish(&status_pub, &status_msg, NULL);
    //     if(ret != RCL_RET_OK){
    //         Serial.println("failed to publish state");
    //         ros::fini_node();            
    //         return;
    //     }

    //     ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    //     if(ret != RCL_RET_OK){
    //         Serial.println("failed to execute spin");
    //         ros::fini_node();            
    //         return;
    //     }
    // }
    // status_msg.data = 1;
    // RCL_SOFT_CHECK(rcl_publish(&status_pub, &status_msg, NULL),
    //     "status pub warn");

    // RCL_SOFT_CHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)),
    //     "rcl spin loop warn")    
    delay(100);
}
