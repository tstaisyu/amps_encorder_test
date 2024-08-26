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
    motorController.sendCommand(motorID, OPERATION_MODE_ADDRESS, MOTOR_SETUP_COMMAND, OPERATION_MODE_SPEED_CONTROL);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, EMERGENCY_STOP_ADDRESS, MOTOR_SETUP_COMMAND, DISABLE_EMERGENCY_STOP);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, CONTROL_WORD_ADDRESS, MOTOR_ENABLE_COMMAND, ENABLE_MOTOR);
    delay(COMMAND_DELAY);
}

void updateOdometry(float rightWheelSpeed, float leftWheelSpeed) {
    static unsigned long last_encoder_receive_time = 0;
    unsigned long current_time = millis();
    double dt = (current_time - last_encoder_receive_time) / 1000.0;
    last_encoder_receive_time = current_time;

    // 左右の車輪の速度を計算
    double v_right = rightWheelSpeed * WHEEL_RADIUS;
    double v_left = leftWheelSpeed * WHEEL_RADIUS;

    // 中心線上の速度と角速度を計算
    double v = (v_right + v_left) / 2.0;
    double omega = (v_right - v_left) / WHEEL_DISTANCE;

    // 新しい位置と姿勢を計算
    x_position += v * cos(theta) * dt;
    y_position += v * sin(theta) * dt;
    theta += omega * dt;

    prepareAndPublishOdometry(x_position, y_position, theta, v, omega);
}

void prepareAndPublishOdometry(double x, double y, double theta, double linear_velocity, double angular_velocity) {
/*    nav_msgs__msg__Odometry odom_msg;

    // 現在のROS 2タイムスタンプを取得
    rcutils_time_point_value_t now;
    rcutils_system_time_now(&now);

    odom_msg.header.stamp.sec = now / 1000000000LL; // 秒単位に変換
    odom_msg.header.stamp.nanosec = now % 1000000000LL; // 余りがナノ秒

    // 文字列フィールドに値を直接設定
    const char* frame_id = "odom";
    const char* child_frame_id = "base_link";
*/
/*    strncpy(odom_msg.header.frame_id.data, frame_id, sizeof(odom_msg.header.frame_id.data));
    odom_msg.header.frame_id.size = strlen(frame_id);

    strncpy(odom_msg.child_frame_id.data, child_frame_id, sizeof(odom_msg.child_frame_id.data));
    odom_msg.child_frame_id.size = strlen(child_frame_id);
*/
/*    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    // Z軸は0として、2Dナビゲーションを想定
    odom_msg.pose.pose.position.z = 0.0;

    setQuaternionFromYaw(theta, &odom_msg.pose.pose.orientation);

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.angular.z = angular_velocity;

    // オドメトリのメッセージをパブリッシュ
    RCCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
*/
}

void setQuaternionFromYaw(double yaw, geometry_msgs__msg__Quaternion *orientation) {
    orientation->x = 0.0;
    orientation->y = 0.0;
    orientation->z = sin(yaw / 2);
    orientation->w = cos(yaw / 2);
}
