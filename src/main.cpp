#include <Arduino.h>

#include <Wire.h>
#include <PS4Controller.h>
#include <RobotLocalization/EKF.h>

#include "Logging.h"
#include "Utils.h"

#include "CustomRosManager.h"
#include "Encoder_AS5600.h"
#include "Imu_BMX055.h"
#include "Driver_DRV8835.h"

// タスクハンドラ
TaskHandle_t thp[2];

// x, y, th, x', y', th'
float x[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// v, arx, ary, w
float y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float y_offset[4] = {0.0f, 0.749f, -0.806f, 0.029f};
float P[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
float Q[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
float R[3] = {0.01f, 0.01f, 0.8f};
float x_new[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
RobotLocalization::EKF ekf(
  x, // [x, y, th, x', y', th']
  P, // P
  Q, // Q
  R // R
);

// 状態遷移用の状態リスト
enum MachineState {
    MAINMASK = 0xf0,
    SUBMASK  = 0x0f,
    
    /**
     * @brief 各機器の接続完了を確認中     
     *  - PS4コントローラの接続
     *  - IMU, Encoder, ToF, モータドライバの初期化
     */
    IDLE = 0x00,

    /**
     * @brief 手動動作モード
     *  PS4コントローラの操作で動作する
     */
    MANUAL = 0x10,

    /**
     * @brief 自動動作モード
     *  自動運転を実施、開始時にパラメータを初期化する
     */
    AUTO        = 0x20,
    _AUTO_INIT  = 0x01,
    _AUTO_RUN   = 0x02,
    AUTO_INIT   = AUTO | _AUTO_INIT,
    AUTO_RUN    = AUTO | _AUTO_RUN,
};


/**
 * @brief グローバル変数
 * 
 */
// 現状状態
MachineState g_currentState = MachineState::IDLE;
// Bluetooth MACアドレス
uint8_t gEspMAC[6];

// 目標速度
float target_vel[2] = {0.0f, 0.0f};
float current_vel[2] = {0.0f, 0.0f};

// /**
//  * @brief 指示速度取得(ROSコールバック)
//  * 
//  */
// void cmd_vel_callback(const void* msgin)
// {
//     const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist *)msgin;

//     target_vel[0] = msg->linear.x;
//     target_vel[1] = msg->angular.z;
// }


/**
 * @brief センサー用タスク（センサ値取得を担当する）
 * 
 * @param args 
 */
void sensor_loop(void *args)
{    
    // I2Cの初期化
    Wire.begin();
    Wire.setClock(400000);
    
    // エンコーダ初期化
    RUNTIME_CHECK(Encoder.begin(&Wire, 0.0638*M_PI/45056, 0x36),
        "encoder initialize error");

    // IMU初期化
    RUNTIME_CHECK(Imu.begin(&Wire, 0x19, 0x69, 0x13),
        "mu initialize error");

    // モータドライバ初期化
    Driver.setAchRange(500, 2400, 1450);
    Driver.setBchRange(700, 2700, 1650);
    RUNTIME_CHECK(Driver.begin(33, 25, 26, 27),
        "driver initialize error");

    // 変数の宣言
    int64_t pulse_interval;
    float vel;
    float dt;
    float imu_acc[3], imu_gyro[3], imu_mag[3];
    uint64_t start_time, stop_time;

    start_time = millis();
    stop_time = start_time;

    while(1){
        // エンコーダパルス取得
        pulse_interval = Encoder.readInterval();          

        // IMU値取得
        Imu.readAcc(imu_acc);
        Imu.readGyro(imu_gyro);
        Imu.readMag(imu_mag);

        // 計測時間間隔
        stop_time = millis();
        dt = (stop_time - start_time)/1000.0f;
        start_time = stop_time;

        // 速度計算
        vel = Encoder.pulse2vel(pulse_interval, dt);

        // 観測情報の登録
        y[0] = vel - y_offset[0];
        y[1] = - imu_acc[0] - y_offset[1];
        y[2] = - imu_acc[1] - y_offset[2];
        y[3] = (imu_gyro[2] - y_offset[3])  / 180.0f * M_PI;

        // 現在状態の確保
        memcpy(x, x_new, sizeof(float)*6);
        ekf.update(x, y, dt, x_new);

        current_vel[0] = vel;
        current_vel[1] = x_new[5];

        // モータ制御        
        Driver.setAvalue(target_vel[0]);
        Driver.setBvalue(target_vel[1]);

        delay(1);
    }
}


void setup()
{    
    // シリアルデバック用設定
    Serial.begin(115200);

    // センサ用ループのタスク登録    
    xTaskCreatePinnedToCore(sensor_loop, "sensor_loop", 8192, NULL, 3, &thp[0], APP_CPU_NUM);

    // ROS初期化
    Serial2.begin(115200);
    RUNTIME_CHECK(Ros.begin(Serial2),
        "ros initialize error");

    // PS4コントローラの初期化
    esp_read_mac(gEspMAC, ESP_MAC_BT);
    char bt_str[32];
    sprintf(bt_str, "%02x:%02x:%02x:%02x:%02x:%02x",
        gEspMAC[0], gEspMAC[1], gEspMAC[2], gEspMAC[3], gEspMAC[4], gEspMAC[5]);    
    PS4.begin(bt_str);
}


void loop()
{
    // 外部コマンド受診時に出力する情報
    if(Serial.available() > 0)
    {        
        String cmd = Serial.readStringUntil('\n');
        cmd = cmd.substring(0, 3);

        // Bluetooth MACアドレスの表示
        if(cmd.compareTo("mac") == 0)
        {            
            log_info("BtMacAddress=%02X:%02X:%02X:%02X:%02X:%02X", 
                gEspMAC[0], gEspMAC[1], gEspMAC[2], gEspMAC[3], gEspMAC[4], gEspMAC[5]);
        }
    }
    // デフォルトの情報出力
    else
    {
        log_debug("CurrentState=%02x", g_currentState);
    }

    // 接続機器の確認
    bool is_all_connected = true;
    bool switch_mode = false;
    float vel[2];    
    if(PS4.isConnected()){
        is_all_connected &= true;

        vel[0] = PS4.LStickY() / 127.0 * 0.08;
        vel[1] = PS4.RStickX() / 127.0 * 0.5;
        

        switch_mode = PS4.Triangle();
    }
    else{
        is_all_connected &= false;
    }

    Ros.status_msg.data = g_currentState;
    Ros.cur_vel_msg.linear.x = target_vel[0];
    Ros.cur_vel_msg.angular.z = target_vel[1];         
    Ros.pose_msg.pose.position.x = x[0];
    Ros.pose_msg.pose.position.y = x[1];
    Ros.pose_msg.pose.position.z = 0.0;
    Ros.pose_msg.pose.orientation.x = 0.0;
    Ros.pose_msg.pose.orientation.y = 0.0;
    Ros.pose_msg.pose.orientation.z = sin(x[2]/2.0);
    Ros.pose_msg.pose.orientation.w = cos(x[2]/2.0);

    if(!Ros.is_initialized()){
        if(Ros.init_node("adrc_esp32", "", 1)){
            is_all_connected &= true;
        }
        else{
            is_all_connected &= false;
        }
    }
    else{
        if(!Ros.publish_all()){
            Ros.fini_node();
            is_all_connected &= false;
        }
        else{
            if(!Ros.spin_once(30)){
                Ros.fini_node();
                is_all_connected &= false;
            }
            else{
                is_all_connected &= true;
            }
        }
    }

    // 状態毎の処理
    switch(g_currentState & MachineState::MAINMASK)
    {
        case MachineState::IDLE:
            target_vel[0] = 0.0f;
            target_vel[1] = 0.0f;
        break;
        case MachineState::MANUAL:
            target_vel[0] =  vel[0];
            target_vel[1] = -vel[1];
        break;
        case MachineState::AUTO:
            // target_vel[0] =  vel[0];
            // target_vel[1] = -vel[1];
        break;
        default:
            log_erro("unknown current state (%x)", g_currentState);
        break;
    }

    // 状態の更新
    switch(g_currentState & MachineState::MAINMASK)
    {
        case MachineState::IDLE:
            if(is_all_connected)
            {
                g_currentState = MachineState::MANUAL;
                PS4.setLed(0, 0, 255);
                PS4.sendToController();
            }
        break;
        case MachineState::MANUAL:
            if(!is_all_connected)
            {
                g_currentState = MachineState::IDLE;
                PS4.setLed(255, 0, 0);
                PS4.sendToController();
            }
            else if(switch_mode)
            {
                g_currentState = MachineState::AUTO;
                PS4.setLed(0, 255, 0);
                PS4.sendToController();
            }
        break;
        case MachineState::AUTO:
            if(!is_all_connected)
            {
                g_currentState = MachineState::IDLE;
                PS4.setLed(255, 0, 0);
                PS4.sendToController();
            }
            else if(!switch_mode)
            {
                g_currentState = MachineState::MANUAL;
                PS4.setLed(0, 0, 255);
                PS4.sendToController();
            }
        break;
    }
    
    // delay(30);
}
