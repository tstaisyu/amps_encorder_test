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

BluetoothSerial SerialBT;

// ハードウェアシリアル設定
HardwareSerial mySerial(2); // RX=16, TX=17を使用

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

// UARTピン設定
const int RX_PIN = 16; // RXピン
const int TX_PIN = 17; // TXピン

// オブジェクトアドレス
const uint16_t OPERATION_MODE_ADDRESS = 0x7017;
const uint16_t EMERGENCY_STOP_ADDRESS = 0x701F;
const uint16_t CONTROL_WORD_ADDRESS = 0x7019;
const uint16_t TARGET_VELOCITY_DEC_ADDRESS = 0x70B2;
const uint16_t ACTUAL_SPEED_DEC_ADDRESS = 0x7077;

// コマンド定義
const byte WRITE_COMMAND = 0x51;
const byte READ_COMMAND = 0x52;

// デフォルト値
const uint32_t OPERATION_MODE_SPEED_CONTROL = 0x00000003;
const uint32_t DISABLE_EMERGENCY_STOP = 0x00000000;
const uint32_t ENABLE_MOTOR = 0x0000000F;
const uint32_t NO_DATA = 0x00000000;

// 通信設定
const int BAUD_RATE = 115200;
const byte MOTOR_ID = 0x01;
const byte ERROR_BYTE = 0x00; // エラーバイトは必要に応じて調整

double speed_ang = 0.0;
double speed_lin = 0.0;

void subscription_callback(const void * msgin) {
  M5.Speaker.begin();
  M5.Lcd.setCursor(0, 20);  
  M5.Lcd.println("Callback triggered");
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // 受信したメッセージの内容をシリアルポート経由で出力
  Serial.print("Received linear.x: ");
  Serial.println(msg->linear.x);
  Serial.print("Received angular.z: ");
  Serial.println(msg->angular.z);

  M5.Lcd.clear();  // LCD画面をクリア
  M5.Lcd.setCursor(0, 20);  // テキスト表示位置を設定
  M5.Lcd.print("Callback triggered");

  M5.Lcd.setCursor(0, 40);
  M5.Lcd.print("Linear.x: ");
  M5.Lcd.println(msg->linear.x);

  M5.Lcd.setCursor(0, 60);
  M5.Lcd.print("Angular.z: ");
  M5.Lcd.println(msg->angular.z);

  // 速度と角速度を取得
  speed_lin = msg->linear.x;
  speed_ang = msg->angular.z;

  static unsigned long lastSendTime = 0;
  const int sendInterval = 20; // ミリ秒単位でコマンド送信間隔
  static int last_velocity_dec = 0; // 最後に送信した速度を保持する変数

  static unsigned long lastUpdateTime = 0;
  const long updateInterval = 20; // 更新間隔を100ミリ秒に設定

  // 新しいデータがあるか確認
  if (Serial.available() > 0) {
    // 入力を読み取り、新しい速度があれば更新
    float velocity_mps = Serial.parseFloat(); // 速度を読み取る
    if (velocity_mps != 0) { // 0以外の値を受け取った場合のみ更新
      int velocity_dec = velocityToDEC(velocity_mps); // m/sをDECに変換
      sendVelocityDEC(velocity_dec); // 速度を送信
      last_velocity_dec = velocity_dec; // 送信した速度を記録
      lastSendTime = millis(); // 最後にコマンドを送信した時間を更新
    }
  }

  // 前回の送信から一定時間が経過したら、最後に設定した速度を再送
  if (millis() - lastSendTime > sendInterval) {
    sendVelocityDEC(last_velocity_dec); // 最後に受け取った速度を再送
    lastSendTime = millis();
  }
  
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime > updateInterval) {
    readActualSpeed();  // 実速度を読み取る
    lastUpdateTime = currentTime;
  }

  // 受信データの処理
  if (mySerial.available()) {
    uint32_t receivedDec = 0;
    int bytesToRead = sizeof(receivedDec);
    if (mySerial.readBytes((char*)&receivedDec, bytesToRead) == bytesToRead) {
      float velocity_mps = calculateVelocityMPS(receivedDec);
      Serial.print("Velocity in mps: ");
      Serial.println(velocity_mps, 3);  // 3桁の精度で表示
    }
  }

  delay(30); // 読み取り間隔を30ミリ秒に設定

  // レスポンスを受信して表示
  if (mySerial.available()) {
    Serial.print("Received: ");
    while (mySerial.available()) {
      int receivedByte = mySerial.read();
      if (receivedByte < 16) {
        Serial.print("0"); // 1桁の場合、先頭に0を追加
      }
      Serial.print(receivedByte, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

}

void setup() {
  Serial.begin(BAUD_RATE);
  while(!Serial);  // シリアルポートが開くのを待つ
  mySerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("Starting motor setup...");

  // オペレーションモードを速度制御モードに設定
  sendCommand(OPERATION_MODE_ADDRESS, WRITE_COMMAND, OPERATION_MODE_SPEED_CONTROL);
  delay(100);

  // エマージェンシーストップを解除
  sendCommand(EMERGENCY_STOP_ADDRESS, WRITE_COMMAND, DISABLE_EMERGENCY_STOP);
  delay(100);

  // モーターを有効化
  sendCommand(CONTROL_WORD_ADDRESS, WRITE_COMMAND, ENABLE_MOTOR);
  delay(100);

  M5.begin();
  delay(500);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);  // ステータスメッセージの位置を設定
  M5.Lcd.print("micro ROS2 M5Stack START\n");  

	// USB経由の場合
	set_microros_transports();

//  delay(2000);

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

void sendVelocityDEC(uint32_t velocity_dec) {
  sendCommand(TARGET_VELOCITY_DEC_ADDRESS, WRITE_COMMAND, velocity_dec); // DECを設定
}

void sendCommand(uint16_t address, byte cmd, uint32_t data) {
  byte addrH = highByte(address);
  byte addrL = lowByte(address);
  byte dataMSB = (data >> 24) & 0xFF;  // 最上位バイト
  byte dataN1 = (data >> 16) & 0xFF;   // 2番目のバイト
  byte dataN2 = (data >> 8) & 0xFF;    // 3番目のバイト
  byte dataLSB = data & 0xFF;          // 最下位バイト

  byte packet[] = {MOTOR_ID, cmd, addrH, addrL, ERROR_BYTE, dataMSB, dataN1, dataN2, dataLSB};
  byte checksum = calculateChecksum(packet, sizeof(packet));

  mySerial.write(packet, sizeof(packet)); // コマンドを送信
  mySerial.write(checksum); // チェックサムを送信
  Serial.print("Sent DEC: ");
  Serial.println(data);
  printPacket(packet, sizeof(packet), checksum);
}

void printPacket(byte *packet, int length, byte checksum) {
  Serial.print("Sent: ");
  for (int i = 0; i < length; i++) {
    if (packet[i] < 16) {
      Serial.print("0");
    }
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  if (checksum < 16) {
    Serial.print("0");
  }
  Serial.println(checksum, HEX);
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF; // チェックサム計算
}

uint32_t velocityToDEC(float velocity_mps) {
    float wheel_circumference = 0.11f * 3.14159f; // 車輪の円周 = 直径 * π
    float rpm = (velocity_mps * 60.0f) / wheel_circumference; // 1分間の回転数
    uint32_t dec = static_cast<uint32_t>((rpm * 512.0f * 4096.0f) / 1875.0f); // DEC値の計算
    return dec;
    }

void readActualSpeed() {
  sendCommand(ACTUAL_SPEED_DEC_ADDRESS, READ_COMMAND, NO_DATA); // 実速度を読み取るコマンドを送信
}

float calculateVelocityMPS(uint32_t dec) {
    // ここに実際の変換ロジックを記述
    return (dec * WHEEL_CIRCUMFERENCE_METERS) / (GEAR_RATIO * ENCODER_RESOLUTION);
}