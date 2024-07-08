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

  clearInitialNoise();
  initializeMotor();

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

void clearInitialNoise() {
  // 受信データをクリアするために一定時間データを読み捨てる
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // 5秒間データを読み捨てる
    while (mySerial.available()) {
      mySerial.read(); // データを読み取り、無視する
    }
  }
}

void initializeMotor() {
  sendCommand(0x7017, 0x51, 0x00000003); // オペレーションモード設定
  delay(100);
  sendCommand(0x701F, 0x51, 0x00000000); // エマージェンシーストップ解除
  delay(100);
  sendCommand(0x7019, 0x52, 0x0000000F); // モーター有効化
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

  // 定期的に実速度データをリクエスト
  if (millis() - lastReadTime > READ_INTERVAL) {
    requestSpeedData();
    lastReadTime = millis();
  }

  // 受信データを処理
    if (mySerial.available()) {
        // 受信データのバイト数が期待するサイズと一致するかチェック
        uint32_t receivedDec;
        byte *dataPtr = (byte*)&receivedDec;
        int bytesReceived = 0;
        while (mySerial.available() && bytesReceived < sizeof(receivedDec)) {
            dataPtr[bytesReceived++] = mySerial.read();
        }

        if (bytesReceived == sizeof(receivedDec)) {
            float velocityMps = calculateVelocityMPS(receivedDec);
            Serial.print("Received DEC: ");
            Serial.print(receivedDec);
            Serial.print(" - Velocity in mps: ");
            Serial.println(velocityMps, 3);
        } else {
            Serial.println("Incomplete data received");
        }
    }

  delay(50);  // 50msのディレイを設定してループの速度を制御  
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

void sendVelocityDEC(uint32_t velocity_dec) {
  sendCommand(0x70B2, 0x54, velocity_dec); // DECを設定
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

uint32_t velocityToDEC(float velocity_mps) {
    float wheel_circumference = 0.11f * 3.14159f; // 車輪の円周 = 直径 * π
    float rpm = (velocity_mps * 60.0f) / wheel_circumference; // 1分間の回転数
    uint32_t dec = static_cast<uint32_t>((rpm * 512.0f * 4096.0f) / 1875.0f); // DEC値の計算
    return dec;}

float calculateVelocityMPS(uint32_t dec) {
    static float lastValidSpeeds[10] = {0};
    static int index = 0;

    float currentSpeed = (dec * 1875.0) / (512.0 * 4096);
    lastValidSpeeds[index++ % 10] = currentSpeed;

    float averageSpeed = 0;
    for (int i = 0; i < 10; i++) {
        averageSpeed += lastValidSpeeds[i];
    }
    averageSpeed /= 10;

    return averageSpeed;
}
