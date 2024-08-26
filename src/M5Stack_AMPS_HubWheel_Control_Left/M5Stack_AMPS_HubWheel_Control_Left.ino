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
#include <string.h>
#include <cmath>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rcutils/time.h"
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include "MotorController.h"
#include "RosCommunications.h"

void subscription_callback(const void * msgin) {

  const geometry_msgs__msg__Twist * msg_sub = (const geometry_msgs__msg__Twist *)msgin;
  last_receive_time = millis();
  if (!initial_data_received) {
    initial_data_received = true;
  }
  
  updateDisplay(msg_sub);
//  logReceivedData(msg_sub);
  sendMotorCommands(msg_sub->linear.x, msg_sub->angular.z);

//  updateOdometry(rightWheelSpeed, leftWheelSpeed); // オドメトリの更新

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
  last_receive_time = millis();  
}

void loop() {

  unsigned long currentMillis = millis();

  // データのパブリッシュ処理
  if (currentMillis - lastReadTime >= readInterval) {
    publishSpeedData();
    lastReadTime = currentMillis;
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  if (!initial_data_received && (millis() - last_receive_time > RECEIVE_TIMEOUT)) {
    Serial.printf("No data received for %d seconds, restarting...\n", RECEIVE_TIMEOUT / 1000);
    ESP.restart();
  }
  if (rcl_error_is_set()) {
    RCL_SET_ERROR_MSG("rclc_executor_spin_some failed");
    printf("Error in rclc_executor_spin_some: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
  }
}

void publishSpeedData() {

  float leftWheelSpeed = readSpeedData(leftMotorSerial, MOTOR_LEFT_ID);

  msg_pub.linear.x = (-1) * leftWheelSpeed;

  // データをパブリッシュ
  rcl_publish(&vel_publisher, &msg_pub, NULL);

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

  RCCHECK(rclc_publisher_init_best_effort(
      &vel_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/left_vel"
  ));

	int callback_size = 1;	// コールバックを行う数
//	executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

}
