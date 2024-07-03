#include <HardwareSerial.h>

HardwareSerial mySerial(2); // 使用するHardwareSerialポートを指定

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始

  // 5 RPMの速度を設定するコマンドを送信
  int rpm = 5; // RPM値
  sendVelocityCommand(0x70B1, rpm);
}

void loop() {
  // モータードライバからの応答を受信して表示
  if (mySerial.available()) {
    Serial.print("Received: ");
    while (mySerial.available()) {
      Serial.print(mySerial.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  delay(1000); // 次の読み取りまで1秒待機
}

void sendVelocityCommand(int address, int rpm) {
  byte addrH = highByte(address);
  byte addrL = lowByte(address);
  byte rpmH = highByte(rpm);
  byte rpmL = lowByte(rpm);
  byte data[] = {0x01, 0x52, addrH, addrL, 0x00, 0x00, 0x00, rpmL, rpmH};
  byte checksum = calculateChecksum(data, sizeof(data));

  mySerial.write(data, sizeof(data)); // コマンドを送信
  mySerial.write(checksum); // チェックサムを送信
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF; // チェックサム計算
}
