#include <Arduino.h>
#include <HardwareSerial.h>

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

HardwareSerial rightMotorSerial(1); // UART1, 右ホイール用
HardwareSerial leftMotorSerial(2); // UART2, 左ホイール用

MotorController motorController(rightMotorSerial, leftMotorSerial);

VelocityCommand currentCommand;

void setup() {
  Serial.begin(BAUD_RATE); // デバッグ用シリアルポートを開始
  // UART1の初期化
  rightMotorSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
  // UART2の初期化
  leftMotorSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN_2, TX_PIN_2);
  Serial.println("Setup complete. Ready to read high resolution speed data.");
  initMotor(rightMotorSerial, MOTOR_RIGHT_ID);  
  initMotor(leftMotorSerial, MOTOR_LEFT_ID);  
}

void loop() {
  static unsigned long lastSendTime = 0;

  if (updateVelocity()) {
    sendMotorCommands(currentCommand.linear_x, currentCommand.angular_z);
    lastSendTime = millis();
  }

  if (millis() - lastSendTime > SEND_INTERVAL) {
    sendMotorCommands(currentCommand.linear_x, currentCommand.angular_z); // 最後のコマンドを再送
    lastSendTime = millis();
  }

  float rightWheelSpeed = readSpeedData(rightMotorSerial, MOTOR_RIGHT_ID);
  float leftWheelSpeed = readSpeedData(leftMotorSerial, MOTOR_LEFT_ID);

  float linearVelocity, angularVelocity;
  calculateOverallVelocity(rightWheelSpeed, leftWheelSpeed, linearVelocity, angularVelocity);

  Serial.print("Linear Velocity: ");
  Serial.print(linearVelocity);
  Serial.print(", Angular Velocity: ");
  Serial.println(angularVelocity);
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
        if (response[0] == motorID && response[1] == READ_DEC_SUCCESS && response[2] == 0x70 && response[3] == 0x77) {
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

bool updateVelocity() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int separator = input.indexOf(',');
    if (separator != -1) {
      String linearStr = input.substring(0, separator);
      String angularStr = input.substring(separator + 1);
      currentCommand.linear_x = linearStr.toFloat();
      currentCommand.angular_z = angularStr.toFloat();
      return true; // 成功した場合
    }
  }
  return false; // データが正しくない場合
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
