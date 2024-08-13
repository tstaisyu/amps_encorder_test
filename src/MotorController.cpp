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

float readSpeedData(HardwareSerial& serial, byte motorID) {
    motorController.sendCommand(motorID, ACTUAL_SPEED_DEC_ADDRESS, READ_DEC_COMMAND, 0);
    if (serial.available() >= 10) {
        uint8_t response[10];
        serial.readBytes(response, 10);
        uint16_t responseAddress = ((uint16_t)response[2] << 8) | response[3];
        if (response[0] == motorID && response[1] == READ_DEC_SUCCESS && responseAddress == ACTUAL_SPEED_DEC_ADDRESS) {
            int32_t receivedDec;
            memcpy(&receivedDec, &response[5], sizeof(receivedDec));
            receivedDec = reverseBytes(receivedDec);
            float velocityMPS = calculateVelocityMPS(receivedDec);
            return velocityMPS;        }
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
    int scaledRPM = (dec * 1875) / (512 * 4096);
    return (scaledRPM * WHEEL_CIRCUMFERENCE) / SCALE_FACTOR;
}

void sendMotorCommands(float linearVelocity, float angularVelocity) {
  // ここで左右のホイールの速度を計算
  float rightWheelSpeed = linearVelocity + (WHEEL_DISTANCE * angularVelocity / 2);
  float leftWheelSpeed = (-1) * (linearVelocity + (WHEEL_DISTANCE * angularVelocity / 2));
  //Rightは  float leftWheelSpeed = linearVelocity - (WHEEL_DISTANCE * angularVelocity / 2);

  int rightWheelDec = velocityToDEC(rightWheelSpeed);
  int leftWheelDec = velocityToDEC(leftWheelSpeed);

    // 右輪と左輪に速度指令を送信
  sendVelocityDEC(rightMotorSerial, rightWheelDec, MOTOR_RIGHT_ID);
  sendVelocityDEC(leftMotorSerial, leftWheelDec, MOTOR_LEFT_ID);
}

void sendVelocityDEC(HardwareSerial& serial, int velocityDec, byte motorID) {
  motorController.sendCommand(motorID, TARGET_VELOCITY_DEC_ADDRESS, VEL_SEND_COMMAND, velocityDec);
}

uint32_t velocityToDEC(float velocityMPS) {
    float wheelCircumference = WHEEL_RADIUS * 2 * PI;
    float rpm = (velocityMPS * 60.0) / wheelCircumference;
    return static_cast<uint32_t>((rpm * 512.0 * 4096.0) / 1875.0);
}

void MotorController::sendCommand(byte motorID, uint16_t address, byte command, uint32_t data) {
    byte packet[] = {motorID, command, highByte(address), lowByte(address), ERROR_BYTE, (byte)(data >> 24), (byte)(data >> 16), (byte)(data >> 8), (byte)data};
    byte checksum = 0;
    for (int i = 0; i < sizeof(packet); i++) {
        checksum += packet[i];
    }
    
    // 条件に基づいて特定のバイトが特定の値の場合にのみパケット全体を表示
    if ((packet[1] == 0xA4 && packet[3] == 0x77)) {
        M5.Lcd.setCursor(0, 80);
        M5.Lcd.print("Packet: ");
        for (int i = 0; i < sizeof(packet); i++) {
            M5.Lcd.printf("%02X ", packet[i]);
        }
        M5.Lcd.println();
        M5.Lcd.setCursor(0, 120);
        M5.Lcd.printf("Checksum: %02X", checksum);
        M5.Lcd.println();
    }

    if ((packet[1] == 0xA4 && packet[3] == 0x77)) {
        M5.Lcd.setCursor(0, 140);
        M5.Lcd.print("Packet: ");
        for (int i = 0; i < sizeof(packet); i++) {
            M5.Lcd.printf("%02X ", packet[i]);
        }
        M5.Lcd.println();
        M5.Lcd.setCursor(0, 180);
        M5.Lcd.printf("Checksum: %02X", checksum);
        M5.Lcd.println();
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


