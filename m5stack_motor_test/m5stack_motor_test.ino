#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始

  Serial.println("Starting motor setup...");

  // オペレーションモードを速度制御モードに設定（モード3）
  sendCommand(0x7017, 0x51, 0x03, 0x00);
  delay(100); // コマンド処理に時間を与える

  // エマージェンシーストップを解除
  sendCommand(0x701F, 0x51, 0x00, 0x00); // 0:Disable
  delay(100);

  // モーターを有効化
  sendCommand(0x7019, 0x52, 0x0F, 0x00);
  delay(100);

  sendCommand(0x70B1, 0x52, 0x04, 0x00);
  delay(100);


}

void loop() {

  // レスポンスを受信して表示
  if (mySerial.available()) {
    Serial.print("Received: ");
    while (mySerial.available()) {
      int receivedByte = mySerial.read();
      if (receivedByte < 16) {
        Serial.print("0"); // 1桁の場合、先頭に0を追加
      }
      Serial.print(receivedByte, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void sendCommand(uint16_t address, byte cmd, uint16_t data, byte msbData) {
  byte addrH = highByte(address);
  byte addrL = lowByte(address);
  byte dataH = highByte(data);
  byte dataL = lowByte(data);
  byte packet[] = {0x01, cmd, addrH, addrL, 0x00, 0x00, 0x00, 0x00, dataL};
  byte checksum = calculateChecksum(packet, sizeof(packet));

  mySerial.write(packet, sizeof(packet)); // コマンドを送信
  mySerial.write(checksum); // チェックサムを送信
  Serial.print("Sent: ");
  for (int i = 0; i < sizeof(packet); i++) {
    if (packet[i] < 16) {
      Serial.print("0");
    }
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  if (checksum < 16) {
    Serial.print("0");
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
