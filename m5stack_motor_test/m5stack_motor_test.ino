#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始

  // オペレーションモードを速度制御モードに設定（モード3）
  sendCommand(0x7017, 0x51, 0x03, 0x00);
  delay(100); // コマンド処理に時間を与える

  // モーターを有効化
  sendCommand(0x7019, 0x52, 0x0F, 0x00);
  delay(100);

  // 速度を100 RPMに設定
  sendCommand(0x70B1, 0x52, 0x64, 0x00);
  delay(100);

  // エラーコードの読み取り
  sendCommand(0x7011, 0xA0, 0x00, 0x00);

  // エマージェンシーストップの状態確認
  sendCommand(0x701F, 0xA0, 0x00, 0x00);
}

void loop() {
  // 応答を受信して表示
  if (mySerial.available()) {
    Serial.print("Received: ");
    while (mySerial.available()) {
      int receivedByte = mySerial.read();
      Serial.print(receivedByte, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  // 定期的にステータス要求を送信
  delay(1000); // 1秒ごとにループ
  sendCommand(0x7001, 0xA0, 0x00, 0x00); // ドライバステータスを要求  
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
  Serial.print("Sent: ");
  for (int i = 0; i < sizeof(packet); i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println(checksum, HEX);
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF; // チェックサム計算
}
