#include <HardwareSerial.h>

constexpr uint16_t ACTUAL_SPEED_HIGH_RES_ADDRESS = 0x7077; // 0.001 RPMのデータを持つアドレス
constexpr byte READ_COMMAND = 0xA0; // 読み取りコマンド
constexpr uint16_t COMMAND_DELAY = 100; // コマンド間のディレイ
constexpr uint32_t SEND_INTERVAL = 1000; // 速度コマンドの送信間隔 (ミリ秒)
constexpr float WHEEL_DIAMETER = 0.11; // 車輪の直径 (メートル)

HardwareSerial mySerial(2); // RX=16, TX=17を使用

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始
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
    sendCommand(0x7017, 0x51, 0x00000003);
    delay(COMMAND_DELAY);
    sendCommand(0x701F, 0x51, 0x00000000);
    delay(COMMAND_DELAY);
    sendCommand(0x7019, 0x52, 0x0000000F);
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
  sendCommand(0x70B2, 0x54, velocityDec);
}

uint32_t velocityToDEC(float velocityMPS) {
    float wheelCircumference = WHEEL_DIAMETER * PI;
    float rpm = (velocityMPS * 60.0) / wheelCircumference;
    return static_cast<uint32_t>((rpm * 512.0 * 4096.0) / 1875.0);
}

void sendCommand(uint16_t address, byte command, uint32_t data) {
    byte packet[] = {0x01, command, highByte(address), lowByte(address), 0x00, (byte)(data >> 24), (byte)(data >> 16), (byte)(data >> 8), (byte)data};
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
