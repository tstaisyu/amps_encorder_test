#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始

  Serial.println("Setup complete. Sending fixed velocity DEC...");

  // オペレーションモードを速度制御モードに設定（モード3）
  sendCommand(0x7017, 0x51, 0x00000003);
  delay(100); // コマンド処理に時間を与える

  // エマージェンシーストップを解除
  sendCommand(0x701F, 0x51, 0x00000000);
  delay(100);

  // モーターを有効化
  sendCommand(0x7019, 0x52, 0x0000000F);
  delay(100);

  // 固定DEC値で速度を設定（例：1000 DEC）
  sendVelocityDEC(0x00100000);
}

void loop() {
  // ループ内で特に操作は行わない
}

void sendVelocityDEC(int velocity_dec) {
  sendCommand(0x70B2, 0x54, velocity_dec); // Target Velocity_DECを設定
}

void sendCommand(uint16_t address, byte cmd, uint32_t data) {
    byte addrH = highByte(address);
    byte addrL = lowByte(address);
    byte dataMSB = (data >> 24) & 0xFF;  // 最上位バイト
    byte dataN1 = (data >> 16) & 0xFF;   // 2番目のバイト
    byte dataN2 = (data >> 8) & 0xFF;    // 3番目のバイト
    byte dataLSB = data & 0xFF;          // 最下位バイト
    byte ErrR = 0x00; // エラー情報、必要に応じて変更

    byte packet[] = {0x01, cmd, addrH, addrL, ErrR, dataMSB, dataN1, dataN2, dataLSB};
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
