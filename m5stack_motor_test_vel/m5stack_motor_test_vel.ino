#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

// モータードライバからの実速度を取得するためのオブジェクトアドレス
const uint16_t ACTUAL_SPEED_DEC_ADDRESS = 0x7077;
const byte READ_COMMAND = 0x52; // 読み取りコマンド

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始

  Serial.println("Starting motor setup...");

  // オペレーションモードを速度制御モードに設定（モード3）
  sendCommand(0x7017, 0x51, 0x00000003);
  delay(100);

  // エマージェンシーストップを解除
  sendCommand(0x701F, 0x51, 0x00000000);
  delay(100);

  // モーターを有効化
  sendCommand(0x7019, 0x52, 0x0000000F);
  delay(100);

}

void loop() {
  static unsigned long lastSendTime = 0;
  const int sendInterval = 100; // ミリ秒単位でコマンド送信間隔
  static int last_velocity_dec = 0; // 最後に送信した速度を保持する変数

  // 新しいデータがあるか確認
  if (Serial.available() > 0) {
    // 入力を読み取り、新しい速度があれば更新
    float velocity_mps = Serial.parseFloat(); // 速度を読み取る
    if (velocity_mps != 0) { // 0以外の値を受け取った場合のみ更新
      int velocity_dec = velocityToDEC(velocity_mps); // m/sをDECに変換
      sendVelocityDEC(velocity_dec); // 速度を送信
      last_velocity_dec = velocity_dec; // 送信した速度を記録
      lastSendTime = millis(); // 最後にコマンドを送信した時間を更新
    }
  }

  // 前回の送信から一定時間が経過したら、最後に設定した速度を再送
  if (millis() - lastSendTime > sendInterval) {
    sendVelocityDEC(last_velocity_dec); // 最後に受け取った速度を再送
    lastSendTime = millis();
  }
  
  // 実速度の読み取りと表示
  readActualSpeed();

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

void sendVelocityDEC(uint32_t velocity_dec) {
  sendCommand(0x70B2, 0x54, velocity_dec); // DECを設定
}

void sendCommand(uint16_t address, byte cmd, uint32_t data) {
  byte addrH = highByte(address);
  byte addrL = lowByte(address);
  byte dataMSB = (data >> 24) & 0xFF;  // 最上位バイト
  byte dataN1 = (data >> 16) & 0xFF;   // 2番目のバイト
  byte dataN2 = (data >> 8) & 0xFF;    // 3番目のバイト
  byte dataLSB = data & 0xFF;          // 最下位バイト
  byte ErrR = 0x00; // エラー情報

  byte packet[] = {0x01, cmd, addrH, addrL, ErrR, dataMSB, dataN1, dataN2, dataLSB};
  byte checksum = calculateChecksum(packet, sizeof(packet));

  mySerial.write(packet, sizeof(packet)); // コマンドを送信
  mySerial.write(checksum); // チェックサムを送信
  Serial.print("Sent DEC: ");
  Serial.println(data);
  printPacket(packet, sizeof(packet), checksum);
}

void printPacket(byte *packet, int length, byte checksum) {
  Serial.print("Sent: ");
  for (int i = 0; i < length; i++) {
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

uint32_t velocityToDEC(float velocity_mps) {
    float wheel_circumference = 0.11f * 3.14159f; // 車輪の円周 = 直径 * π
    float rpm = (velocity_mps * 60.0f) / wheel_circumference; // 1分間の回転数
    uint32_t dec = static_cast<uint32_t>((rpm * 512.0f * 4096.0f) / 1875.0f); // DEC値の計算
    return dec;}

void readActualSpeed() {
  sendCommand(ACTUAL_SPEED_DEC_ADDRESS, READ_COMMAND, 0x00000000); // 実速度を読み取るコマンドを送信
}