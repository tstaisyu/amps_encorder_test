#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

const uint16_t ACTUAL_SPEED_HIGH_RES_ADDRESS = 0x7077; // 0.001 RPMのデータを持つアドレス
const byte READ_COMMAND = 0xA0; // 読み取りコマンド

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始
  Serial.println("Setup complete. Ready to read high resolution speed data.");
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
  const int sendInterval = 1000; // ミリ秒単位でコマンド送信間隔
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

  // 定期的に速度データを読み取る
  readSpeedData();
//  delay(1000); // 1秒ごとに更新
}

void readSpeedData() {
  sendCommand(ACTUAL_SPEED_HIGH_RES_ADDRESS, READ_COMMAND, 0x00000000); // 速度データの読み取りコマンドを送信

    while (mySerial.available() >= 10) { // バッファ内のデータが10バイト以上あるか確認
        uint8_t response[10];
        mySerial.readBytes(response, 10); // 10バイト読み取り

        // 正しいヘッダー（0x01 0x58 または 0x01 0x64）とID（0x7076 または 0x7077）をチェック
        if ((response[0] == 0x01) && 
            ((response[2] == 0x70) && (response[3] == 0x76 || response[3] == 0x77)) &&
            ((response[1] == 0xA4) || (response[1] == 0x64))) {
            int32_t receivedDec;
            // データ部分を読み取り（5〜8バイト目）
            memcpy(&receivedDec, &response[5], sizeof(receivedDec));

            // エンディアン修正が必要な場合
            receivedDec = reverseBytes(receivedDec);

            // 速度計算
            float velocityMPS = calculateVelocityMPS(receivedDec);
            Serial.print("Received DEC: ");
            Serial.print(receivedDec);
            Serial.print(" - Velocity in mps: ");
            Serial.println(velocityMPS);
        }
    }

/*  if (mySerial.available() >= sizeof(int32_t)) {
    // データを受け取る
    uint8_t bytes[sizeof(int32_t)];
    mySerial.readBytes(bytes, sizeof(bytes));

    // データのエンディアンを修正する必要がある場合は、次の行をアンコメントしてください。
    //std::reverse(std::begin(bytes), std::end(bytes));

    int32_t receivedDec;
    memcpy(&receivedDec, bytes, sizeof(receivedDec));

    float velocityMPS = calculateVelocityMPS(receivedDec);
    if (abs(velocityMPS) < 1000) {
      Serial.print("Received DEC: ");
      Serial.print(receivedDec);
      Serial.print(" - Velocity in mps: ");
      Serial.println(velocityMPS);
    } else {
      Serial.println("Received unrealistic speed, ignoring.");
    }
  }*/
}

uint32_t reverseBytes(uint32_t value) {
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8) |
           ((value & 0x00FF0000) >> 8) |
           ((value & 0xFF000000) >> 24);
}

float calculateVelocityMPS(int32_t dec) {
  // DECからRPMへの変換
  float rpm = (dec * 1875.0) / (512.0 * 4096);

  // ホイールの円周を計算
  float wheel_circumference = 0.11f * 3.14159f; // 車輪の直径 0.11m

  // RPMをメートル毎秒に変換
  float velocity_mps = (rpm * wheel_circumference) / 60.0; // 1分間の回転数に円周を掛けて60で割る

  return velocity_mps;  
}

void sendVelocityDEC(uint32_t velocity_dec) {
  sendCommand(0x70B2, 0x54, velocity_dec); // DECを設定
}

uint32_t velocityToDEC(float velocity_mps) {
    float wheel_circumference = 0.11f * 3.14159f; // 車輪の円周 = 直径 * π
    float rpm = (velocity_mps * 60.0f) / wheel_circumference; // 1分間の回転数
    uint32_t dec = static_cast<uint32_t>((rpm * 512.0f * 4096.0f) / 1875.0f); // DEC値の計算
    return dec;}

void sendCommand(uint16_t address, byte command, uint32_t data) {
  byte addrH = highByte(address);
  byte addrL = lowByte(address);
  byte dataMSB = (data >> 24) & 0xFF;
  byte dataMID1 = (data >> 16) & 0xFF;
  byte dataMID2 = (data >> 8) & 0xFF;
  byte dataLSB = data & 0xFF;
  byte packet[] = {0x01, command, addrH, addrL, 0x00, dataMSB, dataMID1, dataMID2, dataLSB};
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
