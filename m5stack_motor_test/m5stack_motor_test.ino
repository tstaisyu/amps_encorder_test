#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を利用

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始
}

void loop() {
  int targetSpeedRpm = 100; // 速度指令値（RPM）
  setTargetVelocity(targetSpeedRpm); // 速度を設定する関数を呼び出し

  delay(2000); // 次のコマンド送信までの間隔
}

void setTargetVelocity(int speed) {
  byte cmd = 0x52; // 'Write data for object of 16 bits' のコマンド
  int addr = 0x70B1; // 'Target velocity_rpm' のアドレス
  byte addrH = highByte(addr);
  byte addrL = lowByte(addr);
  byte speedH = highByte(speed);
  byte speedL = lowByte(speed);
  byte data[] = {0x01, cmd, addrH, addrL, 0x00, 0x00, 0x00, speedL, speedH};
  byte checksum = calculateChecksum(data, sizeof(data));

  mySerial.write(data, sizeof(data)); // コマンドを送信
  mySerial.write(checksum); // チェックサムを送信

  // モータードライバからの応答を受信
  if (waitForResponse()) {
    Serial.println("Speed set successfully.");
  } else {
    Serial.println("Failed to set speed.");
  }
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF; // チェックサム計算
}

bool waitForResponse() {
  unsigned long startTime = millis();
  while (millis() - startTime < 500) { // 500ミリ秒待つ
    if (mySerial.available() >= 10) { // 応答の長さが10バイトの場合
      while (mySerial.available()) {
        Serial.print(mySerial.read(), HEX); // 応答をヘックス形式で表示
        Serial.print(" ");
      }
      Serial.println();
      return true;
    }
  }
  return false;
}
