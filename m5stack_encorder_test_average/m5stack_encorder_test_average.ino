#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を使用

// 定数の設定
const int BUFFER_SIZE = 10; // 平均を取るサンプル数
int speedBuffer[BUFFER_SIZE];
int bufferIndex = 0;
long lastUpdateTime = 0;
const long updateInterval = 100; // 更新間隔（ミリ秒）

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 16, 17);
  memset(speedBuffer, 0, sizeof(speedBuffer)); // バッファをゼロで初期化
}

void loop() {
  if (millis() - lastUpdateTime > updateInterval) {
    // 実速度読み取りコマンドを送信
    sendReadSpeedCommand();
    lastUpdateTime = millis();
  }

  // 速度データを受信してバッファに追加
  if (mySerial.available() >= sizeof(int)) {
    int speedDEC = readSpeedData();
    updateSpeedBuffer(speedDEC);
    int avgSpeed = calculateAverageSpeed();
    Serial.print("Average speed: ");
    Serial.println(avgSpeed);
  }
}

// 速度データ読み取りコマンドを送信する関数
void sendReadSpeedCommand() {
  // ここにコマンド送信のコードを追加
}

// 速度データを読み取る関数
int readSpeedData() {
  // ここにデータ読み取りのコードを追加
  return 0; // 仮の戻り値
}

// 速度バッファを更新する関数
void updateSpeedBuffer(int newSpeed) {
  speedBuffer[bufferIndex % BUFFER_SIZE] = newSpeed;
  bufferIndex++;
}

// 平均速度を計算する関数
int calculateAverageSpeed() {
  long sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += speedBuffer[i];
  }
  return sum / BUFFER_SIZE;
}
