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
#include <Adafruit_VL53L1X.h>


// Right ToF Sensor
const uint8_t rtof_address = 0x28;
const int rtof_shut_pin = 5;
Adafruit_VL53L1X right_tof = Adafruit_VL53L1X(rtof_shut_pin);
// Center ToF Sensor
const uint8_t ctof_address = 0x27;
const int ctof_shut_pin = 18;
Adafruit_VL53L1X center_tof = Adafruit_VL53L1X(ctof_shut_pin);
// Left ToF Sensor
const uint8_t ltof_address = 0x26;
const int ltof_shut_pin = 19;
Adafruit_VL53L1X left_tof = Adafruit_VL53L1X(ltof_shut_pin);


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

    /**
     * @brief 緊急停止モード
     * 　自動運転実施時に接近物が合った場合
     */
    STOP        = 0x40,
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
int16_t current_range[3] = {0, 0, 0};
uint16_t estop_start = 0u;
uint16_t estop_end = 0u;

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

    // ToF初期化
    right_tof.VL53L1X_Off();
    center_tof.VL53L1X_Off();
    left_tof.VL53L1X_Off();
    RUNTIME_CHECK(right_tof.begin(rtof_address, &Wire),
        "right tof initialize error");
    RUNTIME_CHECK(center_tof.begin(ctof_address, &Wire),
        "center tof initialize error");
    RUNTIME_CHECK(left_tof.begin(ltof_address, &Wire),
        "left tof initialize error");
    RUNTIME_CHECK(right_tof.startRanging(),    
        "right tof start error");
    RUNTIME_CHECK(right_tof.setTimingBudget(100),
        "right tof set budget error");
    RUNTIME_CHECK(center_tof.startRanging(),
        "center tof start error");
    RUNTIME_CHECK(center_tof.setTimingBudget(100),
        "center tof set budget error");
    RUNTIME_CHECK(left_tof.startRanging(),
        "left tof start error");
    RUNTIME_CHECK(left_tof.setTimingBudget(100),
        "left tof set budget error");
    
    // エンコーダ初期化
    RUNTIME_CHECK(Encoder.begin(&Wire, 0.0638*M_PI/45056, 0x36),
        "encoder initialize error");

    // IMU初期化
    RUNTIME_CHECK(Imu.begin(&Wire, 0x19, 0x69, 0x13),
        "mu initialize error");

    // モータドライバ初期化
    Driver.setAchRange(500, 2400, 1400);
    Driver.setBchRange(1100, 2200, 1650);
    RUNTIME_CHECK(Driver.begin(33, 25, 26, 27),
        "driver initialize error");

    // 変数の宣言
    int64_t pulse_interval;
    float vel;
    float dt;
    float imu_acc[3], imu_gyro[3], imu_mag[3];
    uint64_t start_time, stop_time;
    int16_t range[3];

    start_time = millis();
    // start_time = micros();
    stop_time = start_time;

    while(1){
        // ToF距離測定
        if(right_tof.dataReady() 
            && center_tof.dataReady()
            && left_tof.dataReady()
            ){
            range[0] = right_tof.distance();
            range[1] = center_tof.distance();
            range[2] = left_tof.distance();

            memcpy(current_range, range, 3 * sizeof(int16_t));
        }

        // エンコーダパルス取得
        pulse_interval = Encoder.readInterval();          

        // IMU値取得
        Imu.readAcc(imu_acc);
        Imu.readGyro(imu_gyro);
        Imu.readMag(imu_mag);

        // 計測時間間隔
        stop_time = millis();
        // stop_time = micros();
        dt = (stop_time - start_time)/1000.0f;
        // dt = (stop_time - start_time)/1000000.0f;
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

        log_debug("[ekf] pose=(%f, %f, %f)", 
            x_new[0], x_new[1], x_new[2]);

        // モータ制御        
        Driver.setAvalue(target_vel[0]);
        Driver.setBvalue(target_vel[1]);

        delay(1);
        // delayMicroseconds(100);
    }
}


void setup()
{    
    // シリアルデバック用設定
    Serial.begin(115200);

    // センサ用ループのタスク登録    
    xTaskCreatePinnedToCore(sensor_loop, "sensor_loop", 8192, NULL, 3, &thp[0], APP_CPU_NUM);
    // xTaskCreatePinnedToCore(sensor_loop, "sensor_loop", 8192, NULL, 3, &thp[0], 0);

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
    bool emergency_stop = false;
    bool switch_mode = false;
    float vel[2], auto_vel[2];
    if(PS4.isConnected()){
        is_all_connected &= true;

        vel[0] = PS4.LStickY() / 127.0 * 0.12;
        vel[1] = PS4.RStickX() / 127.0 * 0.5;

        switch_mode = PS4.Triangle();
    }
    else{
        is_all_connected &= false;
    }

    // Serial.print(current_range[0]); Serial.print(", ");
    // Serial.print(current_range[1]); Serial.print(", ");
    // Serial.print(current_range[2]); Serial.print(", ");
    // Serial.println();
    uint16_t min_dist = 120;
    if((0 <= current_range[0] && current_range[0] < min_dist)
        || (0 <= current_range[1] && current_range[1] < min_dist)
        || (0 <= current_range[2] && current_range[2] < min_dist))
    {
        emergency_stop = true;
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
    auto_vel[0] = Ros.cmd_vel_msg.linear.x;
    auto_vel[1] = Ros.cmd_vel_msg.angular.z;
    if(Ros.reset_response_msg.success){        
        memset(x_new, 0, 6 * sizeof(float));
        Ros.reset_response_msg.success = false;
    }
    // if(Ros.reset_msg.data > 0){
    //     memset(x_new, 0, 6 * sizeof(float));
    // }


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
            target_vel[0] =  auto_vel[0];
            target_vel[1] =  auto_vel[1];
        break;
        case MachineState::STOP:
            target_vel[0] = -0.07f;
            target_vel[1] = 0.0f;
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
                log_info("[main] state switch MANUAL");

                g_currentState = MachineState::MANUAL;
                PS4.setLed(0, 0, 255);
                PS4.sendToController();
            }
        break;
        case MachineState::MANUAL:
            if(!is_all_connected)
            {
                log_info("[main] state switch IDLE");

                g_currentState = MachineState::IDLE;
                PS4.setLed(255, 0, 0);
                PS4.sendToController();
            }
            else if(switch_mode)
            {
                log_info("[main] state switch AUTO");

                g_currentState = MachineState::AUTO;
                PS4.setLed(0, 255, 0);
                PS4.sendToController();
            }
        break;
        case MachineState::AUTO:
            if(!is_all_connected)
            {
                log_info("[main] state switch IDLE");

                g_currentState = MachineState::IDLE;
                PS4.setLed(255, 0, 0);
                PS4.sendToController();
            }
            else if(emergency_stop)
            {
                log_info("[main] state switch STOP");

                g_currentState = MachineState::STOP;
                PS4.setLed(255, 165, 0);
                PS4.sendToController();

                estop_start = millis();
            }
            else if(!switch_mode)
            {
                log_info("[main] state switch MANUAL");

                g_currentState = MachineState::MANUAL;
                PS4.setLed(0, 0, 255);
                PS4.sendToController();
            }            
        break;
        case MachineState::STOP:
            estop_end = millis();
            if((estop_end - estop_start) > 800){
                log_info("[main] state switch AUTO");

                g_currentState = MachineState::AUTO;
                PS4.setLed(0, 255, 0);
                PS4.sendToController();
            }
            else if(!switch_mode)
            {
                log_info("[main] state switch MANUAL");

                g_currentState = MachineState::MANUAL;
                PS4.setLed(0, 0, 255);
                PS4.sendToController();
            }
        break;
    }

    log_debug("[main] current state = %d", g_currentState);
    
    // delay(30);
}
