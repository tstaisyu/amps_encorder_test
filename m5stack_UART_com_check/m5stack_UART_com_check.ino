#include <HardwareSerial.h>

HardwareSerial mySerial(2); // RX=16, TX=17を利用

void setup() {
  Serial.begin(115200); // デバッグ用シリアルポートを開始
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // モータードライバとのUART通信を開始

  // モータードライバに何か簡単なコマンドを送信してみる
  mySerial.write(0x01); // 通信テストのための任意のバイトを送信
}

void loop() {
  if (mySerial.available()) {
    // 受信データがある場合はシリアルポートに出力
    Serial.print("Received: ");
    while(mySerial.available()) {
      Serial.print(mySerial.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  delay(1000); // 1秒ごとにループ
}
