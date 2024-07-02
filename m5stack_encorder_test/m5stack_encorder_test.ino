#include <SoftwareSerial.h>

// UARTピンの設定
const int rxPin = 7; // UART_RXに接続
const int txPin = 8; // UART_TXに接続

// ソフトウェアシリアルの初期化
SoftwareSerial mySerial(rxPin, txPin);

void setup() {
  // デバッグ用シリアル通信の開始
  Serial.begin(115200);
  // ハブホイールモーターとの通信用シリアル通信の開始
  mySerial.begin(115200);
}

void loop() {
  // 「実際の位置」データを要求するコマンドを送信
  sendCommand(0x01, 0xA0, 0x7071);

  // 応答を待つ
  if (waitForResponse()) {
    // 応答から速度データを解析して表示
    displaySpeed();
  }

  delay(1000); // 次のデータ要求まで待機
}

void sendCommand(byte id, byte cmd, int addr) {
  byte addrH = highByte(addr);
  byte addrL = lowByte(addr);
  
  byte command[] = {id, cmd, addrH, addrL, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte checksum = calculateChecksum(command, sizeof(command));
  
  // コマンドの送信
  mySerial.write(command, sizeof(command));
  mySerial.write(checksum);
}

byte calculateChecksum(byte *data, int len) {
  byte sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF; // 下位8ビットのみを使用
}

bool waitForResponse() {
  unsigned long startTime = millis();
  while (millis() - startTime < 500) { // 500ミリ秒以内に応答を待つ
    if (mySerial.available() >= 10) { // 応答の長さが10バイトであることを確認
      return true;
    }
  }
  return false;
}

void displaySpeed() {
  byte response[10];
  for (int i = 0; i < 10; i++) {
    response[i] = mySerial.read();
  }

  if (response[1] == 0xA4) { // 32ビットデータの応答
    long speed = (long)response[5] << 24 | (long)response[6] << 16 | (long)response[7] << 8 | (long)response[8];
    Serial.print("Speed: ");
    Serial.println(speed);
  } else {
    Serial.println("Error or incorrect response");
  }
}