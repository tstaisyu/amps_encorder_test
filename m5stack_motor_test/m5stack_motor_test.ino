#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始

  // オペレーションモードを速度制御モードに設定（モード3）
  sendCommand(0x7017, 0x51, 0x03, 0x00); // 0x51は16ビットデータ書き込みのコマンド

  // モーターを有効化
  sendCommand(0x7019, 0x52, 0x0F, 0x00); // コントロールワードを 'Enable' に設定

  // 速度を100 RPMに設定
  sendCommand(0x70B1, 0x52, 0x64, 0x00); // 100 RPM設定
}

void loop() {
  // 応答を受信して表示
  if (mySerial.available()) {
    Serial.print("Received: ");
    while (mySerial.available()) {
      Serial.print(mySerial.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  delay(1000); // 1秒ごとにループ
}

void sendCommand(uint16_t address, byte cmd, uint16_t data, byte msbData) {
  byte addrH = highByte(address);
  byte addrL = lowByte(address);
  byte dataH = highByte(data);
  byte dataL = lowByte(data);
  byte packet[] = {0x01, cmd, addrH, addrL, 0x00, 0x00, msbData, dataL, dataH};
  byte checksum = calculateChecksum(packet, sizeof(packet));

  mySerial.write(packet, sizeof(packet)); // コマンドを送信
  mySerial.write(checksum); // チェックサムを送信
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF; // チェックサム計算
}
