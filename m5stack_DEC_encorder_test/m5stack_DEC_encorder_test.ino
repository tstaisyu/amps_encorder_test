#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

// UARTピン設定
const int RX_PIN = 16; // RXピン
const int TX_PIN = 17; // TXピン

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

// デフォルト値
constexpr uint32_t OPERATION_MODE_SPEED_CONTROL = 0x00000003;
constexpr uint32_t DISABLE_EMERGENCY_STOP = 0x00000000;
constexpr uint32_t ENABLE_MOTOR = 0x0000000F;
constexpr uint32_t NO_DATA = 0x00000000;

// 通信設定
constexpr int BAUD_RATE = 115200;
constexpr byte MOTOR_ID = 0x01;
constexpr byte ERROR_BYTE = 0x00; // エラーバイトは必要に応じて調整

// ディレイ設定
constexpr uint16_t COMMAND_DELAY = 100; // コマンド間のディレイ
constexpr uint32_t SEND_INTERVAL = 1000; // 速度コマンドの送信間隔 (ミリ秒)

// モーター仕様
constexpr float WHEEL_DIAMETER = 0.11; // 車輪の直径 (メートル)

bool initial_data_received = false; // データ受信の有無を追跡
unsigned long last_receive_time = 0; // 最後にデータを受信した時刻
const unsigned long RECEIVE_TIMEOUT = 5000; // タイムアウト値を5000ミリ秒に設定

void setup() {
  Serial.begin(BAUD_RATE); // デバッグ用シリアルポートを開始
  mySerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN); // モータードライバとのUART通信を開始
  Serial.println("Setup complete. Ready to read high resolution speed data.");
  initMotor();  
}

void loop() {
  static unsigned long lastSendTime = 0;
  static int lastVelocityDec = 0;

  if (Serial.available() > 0) {
      float velocityMPS = Serial.parseFloat();
      if (velocityMPS != 0) {
          int velocityDec = velocityToDEC(velocityMPS);
          sendVelocityDEC(velocityDec);
          lastVelocityDec = velocityDec;
          lastSendTime = millis();
      }
  }

  if (millis() - lastSendTime > SEND_INTERVAL) {
      sendVelocityDEC(lastVelocityDec);
      lastSendTime = millis();
  }

  readSpeedData();
}

void initMotor() {
    sendCommand(OPERATION_MODE_ADDRESS, WRITE_COMMAND, OPERATION_MODE_SPEED_CONTROL);
    delay(COMMAND_DELAY);
    sendCommand(EMERGENCY_STOP_ADDRESS, WRITE_COMMAND, DISABLE_EMERGENCY_STOP);
    delay(COMMAND_DELAY);
    sendCommand(CONTROL_WORD_ADDRESS, WRITE_COMMAND, ENABLE_MOTOR);
    delay(COMMAND_DELAY);
}

void readSpeedData() {
    sendCommand(ACTUAL_SPEED_HIGH_RES_ADDRESS, READ_COMMAND, 0);
    while (mySerial.available() >= 10) {
        uint8_t response[10];
        mySerial.readBytes(response, 10);
        if (response[0] == 0x01 && response[1] == 0xA4 && response[2] == 0x70 && response[3] == 0x77) {
            int32_t receivedDec;
            memcpy(&receivedDec, &response[5], sizeof(receivedDec));
            receivedDec = reverseBytes(receivedDec);
            float velocityMPS = calculateVelocityMPS(receivedDec);
            Serial.print("Received DEC: ");
            Serial.print(receivedDec);
            Serial.print(" - Velocity in mps: ");
            Serial.println(velocityMPS);
        }
    }
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

void sendVelocityDEC(uint32_t velocityDec) {
  sendCommand(TARGET_VELOCITY_DEC_ADDRESS, WRITE_COMMAND, velocityDec);
}

uint32_t velocityToDEC(float velocityMPS) {
    float wheelCircumference = WHEEL_DIAMETER * PI;
    float rpm = (velocityMPS * 60.0) / wheelCircumference;
    return static_cast<uint32_t>((rpm * 512.0 * 4096.0) / 1875.0);
}

void sendCommand(uint16_t address, byte command, uint32_t data) {
    byte packet[] = {MOTOR_ID, command, highByte(address), lowByte(address), ERROR_BYTE, (byte)(data >> 24), (byte)(data >> 16), (byte)(data >> 8), (byte)data};
    byte checksum = calculateChecksum(packet, sizeof(packet));
    mySerial.write(packet, sizeof(packet));
    mySerial.write(checksum);
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF;
}
