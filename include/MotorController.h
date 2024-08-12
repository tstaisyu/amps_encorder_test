/* Copyright 2024 Taisyu Shibata
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

class MotorController {
private:
    HardwareSerial& rightMotorSerial;
    HardwareSerial& leftMotorSerial;

public:
    MotorController(HardwareSerial& rightSerial, HardwareSerial& leftSerial)
    : rightMotorSerial(rightSerial), leftMotorSerial(leftSerial) {}

    void sendCommand(byte motorID, uint16_t address, byte command, uint32_t data);
};

struct VelocityCommand {
  float linear_x;
  float angular_z;
};

// グローバル変数で現在の位置と姿勢を保持
double x_position = 0.0;
double y_position = 0.0;
double theta = 0.0; // ロボットの向き（ラジアン）

constexpr byte MOTOR_RIGHT_ID = 0x02;
constexpr byte MOTOR_LEFT_ID = 0x01;

// UARTピン設定
constexpr int RX_PIN_1 = 21; // UART1のRXピン
constexpr int TX_PIN_1 = 22; // UART1のTXピン
constexpr int RX_PIN_2 = 16; // UART2のRXピン
constexpr int TX_PIN_2 = 17; // UART2のTXピン

// オブジェクトアドレス
constexpr uint16_t OPERATION_MODE_ADDRESS = 0x7017;
constexpr uint16_t EMERGENCY_STOP_ADDRESS = 0x701F;
constexpr uint16_t CONTROL_WORD_ADDRESS = 0x7019;
constexpr uint16_t TARGET_VELOCITY_DEC_ADDRESS = 0x70B2;
constexpr uint16_t ACTUAL_SPEED_DEC_ADDRESS = 0x7077;

// コマンド定義
constexpr byte MOTOR_SETUP_COMMAND = 0x51;
constexpr byte MOTOR_ENABLE_COMMAND = 0x52;
constexpr byte VEL_SEND_COMMAND = 0x54;
constexpr byte READ_COMMAND = 0x52;
constexpr byte READ_DEC_COMMAND = 0xA0;
constexpr byte READ_DEC_SUCCESS = 0xA4;

// デフォルト値
constexpr uint32_t OPERATION_MODE_SPEED_CONTROL = 0x00000003;
constexpr uint32_t DISABLE_EMERGENCY_STOP = 0x00000000;
constexpr uint32_t ENABLE_MOTOR = 0x0000000F;
constexpr uint32_t NO_DATA = 0x00000000;

// 通信設定
constexpr int BAUD_RATE = 115200;
constexpr byte ERROR_BYTE = 0x00; // エラーバイトは必要に応じて調整

// ディレイ設定
constexpr uint16_t COMMAND_DELAY = 100; // コマンド間のディレイ
constexpr uint32_t SEND_INTERVAL = 1000; // 速度コマンドの送信間隔 (ミリ秒)

// モーター仕様
constexpr float WHEEL_RADIUS = 0.055; // 車輪の半径 (メートル)
constexpr float WHEEL_DISTANCE = 0.202; // ホイール間の距離を設定 (メートル)

bool initial_data_received = false; // データ受信の有無を追跡
unsigned long last_receive_time = 0; // 最後にデータを受信した時刻
#define RECEIVE_TIMEOUT 5000 // タイムアウト値を5000ミリ秒に設定

BluetoothSerial SerialBT;

HardwareSerial rightMotorSerial(1); // UART1, 右ホイール用
HardwareSerial leftMotorSerial(2); // UART2, 左ホイール用

MotorController motorController(rightMotorSerial, leftMotorSerial);

VelocityCommand currentCommand;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
geometry_msgs__msg__Twist msg_pub;
rcl_publisher_t vel_publisher;
nav_msgs__msg__Odometry odom_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//rcl_init_options_t init_options; // Humble
//size_t domain_id = 117;
unsigned long lastReadTime = 0;
const unsigned int readInterval = 40; 

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {Serial.println("Error in " #fn); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
