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

#include "BluetoothSerial.h"
#include <micro_ros_arduino.h>
#include <HardwareSerial.h>

#include <M5Stack.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include "CytronMotorDriver.h"

#include "esp32-hal-timer.h"

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

byte MOTOR_RIGHT_ID = 0x01;
byte MOTOR_LEFT_ID = 0x02;

// UARTピン設定
const int RX_PIN_1 = 25; // UART1のRXピン
const int TX_PIN_1 = 26; // UART1のTXピン
const int RX_PIN_2 = 16; // UART2のRXピン
const int TX_PIN_2 = 17; // UART2のTXピン

// オブジェクトアドレス
constexpr uint16_t OPERATION_MODE_ADDRESS = 0x7017;
constexpr uint16_t EMERGENCY_STOP_ADDRESS = 0x701F;
constexpr uint16_t CONTROL_WORD_ADDRESS = 0x7019;
constexpr uint16_t TARGET_VELOCITY_DEC_ADDRESS = 0x70B2;
constexpr uint16_t ACTUAL_SPEED_DEC_ADDRESS = 0x7077;

// コマンド定義
constexpr byte WRITE_COMMAND = 0x51;
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
constexpr float WHEEL_DIAMETER = 0.11; // 車輪の直径 (メートル)
constexpr float WHEEL_DISTANCE = 0.30; // ホイール間の距離を設定 (メートル)

bool initial_data_received = false; // データ受信の有無を追跡
unsigned long last_receive_time = 0; // 最後にデータを受信した時刻
const unsigned long RECEIVE_TIMEOUT = 5000; // タイムアウト値を5000ミリ秒に設定

BluetoothSerial SerialBT;

HardwareSerial rightMotorSerial(1); // UART1, 右ホイール用
HardwareSerial leftMotorSerial(2); // UART2, 左ホイール用

MotorController motorController(rightMotorSerial, leftMotorSerial);

VelocityCommand currentCommand;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//rcl_init_options_t init_options; // Humble
//size_t domain_id = 117;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {Serial.println("Error in " #fn); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void subscription_callback(const void * msgin) {

  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  if (!initial_data_received) {
    initial_data_received = true;
  }
  
  updateDisplay(msg);
  logReceivedData(msg);
  sendMotorCommands(msg->linear.x, msg->angular.z);

  float rightWheelSpeed = readSpeedData(rightMotorSerial, MOTOR_RIGHT_ID);
  float leftWheelSpeed = readSpeedData(leftMotorSerial, MOTOR_LEFT_ID);

  float linearVelocity, angularVelocity;
  calculateOverallVelocity(rightWheelSpeed, leftWheelSpeed, linearVelocity, angularVelocity);

  Serial.print("Linear Velocity: ");
  Serial.print(linearVelocity);
  Serial.print(", Angular Velocity: ");
  Serial.println(angularVelocity);

  delay(30); // 読み取り間隔を30ミリ秒に設定
}

void setup() {
  Serial.begin(BAUD_RATE);
  while(!Serial);  // シリアルポートが開くのを待つ
  // UART1の初期化
  rightMotorSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
  // UART2の初期化
  leftMotorSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN_2, TX_PIN_2);
  Serial.println("Setup complete. Ready to read high resolution speed data.");
  initMotor(rightMotorSerial, MOTOR_RIGHT_ID);  
  initMotor(leftMotorSerial, MOTOR_LEFT_ID);  

  M5.begin();
  delay(500);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);  // ステータスメッセージの位置を設定
  M5.Lcd.print("micro ROS2 M5Stack START\n");  

  setupMicroROS();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  if (!initial_data_received && (millis() - last_receive_time > RECEIVE_TIMEOUT)) {
    Serial.printf("No data received for %d seconds, restarting...\n", RECEIVE_TIMEOUT / 1000);
    ESP.restart();
  }
  delay(100);
  if (rcl_error_is_set()) {
    RCL_SET_ERROR_MSG("rclc_executor_spin_some failed");
    printf("Error in rclc_executor_spin_some: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
  }
}

void setupMicroROS() {
	set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	//init_options = rcl_get_zero_initialized_init_options();
	//RCCHECK(rcl_init_options_init(&init_options, allocator));
	//RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));		// ドメインIDの設定
	//RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)); // 前のrclc_support_initは削除する
  RCCHECK(rclc_node_init_default(&node, "subscriber_node", "", &support));
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));
	int callback_size = 1;	// コールバックを行う数
//	executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void logReceivedData(const geometry_msgs__msg__Twist *msg) {
  Serial.print("Received linear.x: ");
  Serial.println(msg->linear.x);
  Serial.print("Received angular.z: ");
  Serial.println(msg->angular.z);
}

void updateDisplay(const geometry_msgs__msg__Twist *msg) {
  M5.Lcd.clear();  // LCD画面をクリア
  M5.Lcd.setCursor(0, 20);  // テキスト表示位置を設定
  M5.Lcd.print("Callback triggered");

  M5.Lcd.setCursor(0, 40);
  M5.Lcd.print("Linear.x: ");
  M5.Lcd.println(msg->linear.x);

  M5.Lcd.setCursor(0, 60);
  M5.Lcd.print("Angular.z: ");
  M5.Lcd.println(msg->angular.z);
}

void initMotor(HardwareSerial& serial, byte motorID) {
    motorController.sendCommand(motorID, OPERATION_MODE_ADDRESS, WRITE_COMMAND, OPERATION_MODE_SPEED_CONTROL);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, EMERGENCY_STOP_ADDRESS, WRITE_COMMAND, DISABLE_EMERGENCY_STOP);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, CONTROL_WORD_ADDRESS, WRITE_COMMAND, ENABLE_MOTOR);
    delay(COMMAND_DELAY);
}

void calculateOverallVelocity(float rightWheelSpeed, float leftWheelSpeed, float& linearVelocity, float& angularVelocity) {
    linearVelocity = (rightWheelSpeed + leftWheelSpeed) / 2;
    angularVelocity = (rightWheelSpeed - leftWheelSpeed) / WHEEL_DISTANCE;
}

float readSpeedData(HardwareSerial& serial, byte motorID) {
    motorController.sendCommand(motorID, ACTUAL_SPEED_DEC_ADDRESS, READ_COMMAND, 0);
    while (serial.available() >= 10) {
        uint8_t response[10];
        serial.readBytes(response, 10);
        uint16_t responseAddress = ((uint16_t)response[2] << 8) | response[3];
        if (response[0] == motorID && response[1] == READ_DEC_SUCCESS && responseAddress == ACTUAL_SPEED_DEC_ADDRESS) {
            int32_t receivedDec;
            memcpy(&receivedDec, &response[5], sizeof(receivedDec));
            receivedDec = reverseBytes(receivedDec);
            float velocityMPS = calculateVelocityMPS(receivedDec);
            Serial.print("Motor ");
            Serial.print(motorID);
            Serial.print("Received DEC: ");
            Serial.print(receivedDec);
            Serial.print(" - Velocity in mps: ");
            Serial.println(velocityMPS);
            return velocityMPS;
        }
    }
    return 0.0;
}

uint32_t reverseBytes(uint32_t value) {
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8) |
           ((value & 0x00FF0000) >> 8) |
           ((value & 0xFF000000) >> 24);
}

float calculateVelocityMPS(int32_t dec) {
    float wheelCircumference = WHEEL_DIAMETER * PI;
    float rpm = (dec * 1875.0) / (512.0 * 4096);
    return (rpm * wheelCircumference) / 60.0;
}

void sendMotorCommands(float linearVelocity, float angularVelocity) {
  // ここで左右のホイールの速度を計算
  float rightWheelSpeed = linearVelocity + (WHEEL_DISTANCE * angularVelocity / 2);
  float leftWheelSpeed = linearVelocity - (WHEEL_DISTANCE * angularVelocity / 2);

  int rightWheelDec = velocityToDEC(rightWheelSpeed);
  int leftWheelDec = velocityToDEC(leftWheelSpeed);

    // 右輪と左輪に速度指令を送信
  sendVelocityDEC(rightMotorSerial, rightWheelDec, MOTOR_RIGHT_ID);
  sendVelocityDEC(leftMotorSerial, leftWheelDec, MOTOR_LEFT_ID);
}

void sendVelocityDEC(HardwareSerial& serial, int velocityDec, byte motorID) {
  motorController.sendCommand(motorID, TARGET_VELOCITY_DEC_ADDRESS, WRITE_COMMAND, velocityDec);
}

uint32_t velocityToDEC(float velocityMPS) {
    float wheelCircumference = WHEEL_DIAMETER * PI;
    float rpm = (velocityMPS * 60.0) / wheelCircumference;
    return static_cast<uint32_t>((rpm * 512.0 * 4096.0) / 1875.0);
}

void MotorController::sendCommand(byte motorID, uint16_t address, byte command, uint32_t data) {
    byte packet[] = {motorID, command, highByte(address), lowByte(address), ERROR_BYTE, (byte)(data >> 24), (byte)(data >> 16), (byte)(data >> 8), (byte)data};
    byte checksum = 0;
    for (int i = 0; i < sizeof(packet); i++) {
        checksum += packet[i];
    }
    if (motorID == MOTOR_RIGHT_ID) {
        rightMotorSerial.write(packet, sizeof(packet));
        rightMotorSerial.write(checksum);
    } else {
        leftMotorSerial.write(packet, sizeof(packet));
        leftMotorSerial.write(checksum);
    }
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF;
}