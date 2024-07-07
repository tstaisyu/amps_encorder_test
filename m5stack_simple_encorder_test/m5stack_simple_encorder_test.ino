#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

// モータードライバからの実速度を取得するためのオブジェクトアドレス
const uint16_t ACTUAL_SPEED_DEC_ADDRESS = 0x7077;
const byte READ_COMMAND = 0x52; // 読み取りコマンド
const int READ_INTERVAL = 100; // 読み取り間隔を100msに設定

unsigned long lastReadTime = 0; // 最後にデータを読み取った時間

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始
  Serial.println("Setup complete. Ready to read speed data.");
}

void loop() {
  // 定期的に実速度データをリクエスト
  if (millis() - lastReadTime > READ_INTERVAL) {
    requestSpeedData();
    lastReadTime = millis();
  }

  // 受信データを処理
  if (mySerial.available()) {
    uint32_t receivedDec = readSpeedData();
    if (receivedDec != 0) { // 有効なデータのみ処理
      float velocity_mps = calculateVelocityMPS(receivedDec);
      Serial.print("Velocity in mps: ");
      Serial.println(velocity_mps, 3);
    }
  }
}

void requestSpeedData() {
  sendCommand(ACTUAL_SPEED_DEC_ADDRESS, READ_COMMAND, 0); // 実速度をリクエスト
}

uint32_t readSpeedData() {
  uint32_t receivedDec = 0;
  int bytesToRead = sizeof(receivedDec);
  if (mySerial.readBytes((char*)&receivedDec, bytesToRead) == bytesToRead) {
    return receivedDec; // 正しくデータを受信した場合
  }
  return 0; // データ受信失敗
}

void sendCommand(uint16_t address, byte cmd, uint32_t data) {
  byte addrH = highByte(address);
  byte addrL = lowByte(address);
  byte dataMSB = (data >> 24) & 0xFF;
  byte dataN1 = (data >> 16) & 0xFF;
  byte dataN2 = (data >> 8) & 0xFF;
  byte dataLSB = data & 0xFF;
  byte packet[] = {0x01, cmd, addrH, addrL, 0x00, dataMSB, dataN1, dataN2, dataLSB};
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

float calculateVelocityMPS(uint32_t dec) {
  float rpm = (dec * 1875.0) / (512.0 * 4096.0);
  float wheel_circumference = 0.11 * 3.14159; // 車輪の円周
  return (rpm * wheel_circumference) / 60.0; // m/sへ変換
}
