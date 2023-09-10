#include <Arduino.h>
#include <Wire.h>
#include <PS4Controller.h>

#include "Logging.h"
#include "Encoder_AS5600.h"
#include "Imu_BMX055.h"
#include "Driver_DRV8835.h"

// タスクハンドラ
TaskHandle_t thp[2];


// 状態遷移用の状態リスト
enum MachineState {
    MAINMASK = 0xf0,
    SUBMASK  = 0x0f,

    /**
     * @brief 各機器の接続完了を確認中
     *  - ROSエージェントの接続
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


struct SensorDataPack
{
    float vel;
};


/**
 * @brief グローバル変数
 * 
 */
// 現状状態
MachineState g_currentState = MachineState::IDLE;
// Bluetooth MACアドレス
uint8_t gEspMAC[6];
// エンコーダクラス
Encoder_AS5600 g_encoder(0.0638*M_PI/45056);
// IMUクラス
Imu_BMX055 g_imu;
// モータドライバクラス
Driver_DRV8835 drive(33, 25, 26, 27);
// タイマ割り込みハンドラ
// hw_timer_t * g_timer = NULL;
// タイマ内排他制御ハンドラ
// volatile SemaphoreHandle_t g_timerSemaphore;
// portMUX_TYPE g_timerMux = portMUX_INITIALIZER_UNLOCKED;
// スレッド間キュー
QueueHandle_t g_sensorQueue;


/**
 * @brief センサー取得タイマー関数(1kHz)
 * 
 */
// void IRAM_ATTR get_sensors()
// {
//     portENTER_CRITICAL_ISR(&g_timerMux);

//     portEXIT_CRITICAL_ISR(&g_timerMux);

//     xSemaphoreGiveFromISR(g_timerSemaphore, NULL);
// }


/**
 * @brief Core0用タスク（センサ値取得を担当する）
 * 
 * @param args 
 */
void main_core0(void *args)
{    
    // I2Cの初期化
    Wire.begin();
    Wire.setClock(400000);
    
    // エンコーダ初期化
    if(!g_encoder.begin(&Wire, 0x36))
    {
        while(1)
        {
            log_erro("encoder initialize error");
            delay(500);
        }
    }    

    // IMU初期化
    if(!g_imu.begin(&Wire, 0x19, 0x69, 0x13)){
        while(1)
        {
            log_erro("imu initialize error");
            delay(500);
        }
    }

    // // センサ取得割り込み登録
    // g_timerSemaphore = xSemaphoreCreateBinary();
    // g_timer = timerBegin(0, 80, true);
    // timerAttachInterrupt(g_timer, &get_sensors, true);
    // timerAlarmWrite(g_timer, 1000, true);
    // timerAlarmEnable(g_timer);

    // uint16_t encoder_angle;
    int64_t pulse_interval;
    float vel;
    float dt;
    float imu_acc[3], imu_gyro[3], imu_mag[3];
    uint64_t start_time, stop_time;

    start_time = millis();
    stop_time = start_time;

    while(1){
        // エンコーダパルス取得
        pulse_interval = g_encoder.readInterval();          

        // IMU値取得
        g_imu.readAcc(imu_acc);
        g_imu.readGyro(imu_gyro);
        g_imu.readMag(imu_mag);

        // 計測時間間隔
        stop_time = millis();
        dt = (stop_time - start_time)/1000.0f;
        start_time = stop_time;

        // 速度計算
        vel = g_encoder.pulse2vel(pulse_interval, dt);
    }
}


void setup()
{    
    // シリアルデバック用設定
    Serial.begin(115200);

    // // センサ取得キューの登録
    // g_sensorQueue = xQueueCreate(3, sizeof(SensorDataPack));

    // Core0のタスク登録    
    xTaskCreatePinnedToCore(main_core0, "main_core0", 4096, NULL, 3, &thp[0], 0);

    // PS4コントローラの初期化
    esp_read_mac(gEspMAC, ESP_MAC_BT);
    char bt_str[18];
    sprintf(bt_str, "%02x:%02x:%02x:%02x:%02x:%02x",
        gEspMAC[0], gEspMAC[1], gEspMAC[2], gEspMAC[3], gEspMAC[4], gEspMAC[5]);
    PS4.begin(bt_str);

    // モータドライバ初期化
    drive.setAchRange(500, 2400, 1450);
    drive.setBchRange(500, 2400, 1450);
    drive.begin();
}


void loop()
{
    // 外部コマンド受診時に出力する情報
    if(Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        
        // Bluetooth MACアドレスの表示
        if(cmd.compareTo("mac\n"))
        {            
            log_info("BtMacAddress=%02X:%02X:%02X:%02X:%02X:%02X", 
                gEspMAC[0], gEspMAC[1], gEspMAC[2], gEspMAC[3], gEspMAC[4], gEspMAC[5]);
        }
    }
    // デフォルトの情報出力
    else
    {
        log_info("CurrentState=%02x", g_currentState);
    }

    // SensorDataPack sensor_data;
    // xQueueReceive(g_sensorQueue, &sensor_data, portMAX_DELAY);
    // Serial.print(sensor_data.vel);

    // 接続機器の確認
    bool is_all_connected = true;
    float vel[2];
    if(PS4.isConnected()){
        is_all_connected &= true;

        vel[0] = PS4.LStickY() / 127.0;
        vel[1] = PS4.LStickX() / 127.0;
    }
    else{
        is_all_connected &= false;
    }

    // 状態毎の処理
    switch(g_currentState & MachineState::MAINMASK)
    {
        case MachineState::IDLE:
        break;
        case MachineState::MANUAL:
        break;
        case MachineState::AUTO:
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
            }
        break;
        case MachineState::MANUAL:
            drive.setAvalue(vel[0]);
            drive.setBvalue(-vel[1]);
        break;
        case MachineState::AUTO:
        break;
    }

    delay(10);
}
