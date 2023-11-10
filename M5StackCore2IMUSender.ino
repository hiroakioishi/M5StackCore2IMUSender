#include <M5Core2.h>
#include "BluetoothSerial.h"

// Bluetoothシリアルの名前 (PCで表示されるBluetoothデバイス名)
#define BLUETOOTH_SERIAL_NAME "M5StackCore2BT_01"
// FPS
#define FPS 30

// Bluetoothシリアル
BluetoothSerial serialBT;

// 加速度 (Acceleration)
float accX = 0.0f;
float accY = 0.0f;
float accZ = 0.0f;
// ジャイロ (Gyroscope) (角速度)
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
// 姿勢航法基準装置 (Attitude and Heading Reference System)
float pitch = 0.0f;
float roll  = 0.0f;
float yaw   = 0.0f;

// Bluetoothで値を送信するときに使用するバッファ
char buff[60];

// ------ 初期化 ---------------------
void setup()
{
  // 本体初期化
  M5.begin();
  // Bluetooth 初期化
  serialBT.begin(BLUETOOTH_SERIAL_NAME);
  // 0.5秒待機
  delay(500);
 
  // 慣性測定ユニット初期化
  M5.IMU.Init();
  // 画面の背景を黒で塗る
  M5.Lcd.fillScreen(BLACK);
  // 文字の色を白, 文字背景を黒にセット  
  M5.Lcd.setTextColor(WHITE, BLACK);
}

// ------ 更新 ---------------------
void loop() 
{
  // ボタンの状態を更新
  M5.update();

  // --- IMUの値を取得 ---
  // 加速度
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  // ジャイロ
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  // 姿勢航法基準装置（Attitude and Heading Reference System）
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);

  // --- IMUの値をBluetoothで送信 ---
  // buffに、Accel, Gyro, Ahrsの値をまとめる 
  //sprintf(buff, "%5.2f,%5.2f,%5.2f\r\n", pitch, roll, yaw);
  sprintf(buff, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\r\n", accX, accY, accZ, gyroX, gyroY, gyroZ, pitch, roll, yaw);
  // Bluetoothで、Accel, Gyro, Ahrs の値を送信
  serialBT.printf(buff);

  // --- ボタン操作の処理 ---
  // A（左）ボタンが押されたら
  if (M5.BtnA.wasPressed())
  {
    // 文字色を白、文字背景を赤にする
    M5.Lcd.setTextColor(WHITE, RED);
    // 画面の背景を赤で塗る
    M5.Lcd.fillScreen(RED);
  }
  // A（左）ボタンが離されたら
  if (M5.BtnA.wasReleased())
  {
    // 文字の色を白, 文字背景を黒にセット  
    M5.Lcd.setTextColor(WHITE, BLACK);
    // 画面の背景を黒で塗る
    M5.Lcd.fillScreen(BLACK);
  }
  // B（中）ボタンが押されたら
  if (M5.BtnB.wasPressed())
  {
    // 文字色を白、文字背景を緑にする
    M5.Lcd.setTextColor(WHITE, GREEN);
    // 画面の背景を緑で塗る
    M5.Lcd.fillScreen(GREEN);
  }
  // B（中）ボタンが離されたら
  if (M5.BtnB.wasReleased())
  {
    // 文字の色を白, 文字背景を黒にセット  
    M5.Lcd.setTextColor(WHITE, BLACK);
    // 画面の背景を黒で塗る
    M5.Lcd.fillScreen(BLACK);
  }
  // C（右）ボタンが押されたら
  if (M5.BtnC.wasPressed())
  {
    // 文字色を白、文字背景を青にする
    M5.Lcd.setTextColor(WHITE, BLUE);
    // 画面の背景を青で塗る
    M5.Lcd.fillScreen(BLUE);
  }
  // C（右）ボタンが離されたら
  if (M5.BtnC.wasReleased())
  {
    // 文字の色を白, 文字背景を黒にセット  
    M5.Lcd.setTextColor(WHITE, BLACK);
    // 画面の背景を黒で塗る
    M5.Lcd.fillScreen(BLACK);
  }

  // --- 液晶画面にセンサーの値などの情報を表示する ---
  // 文字の色を白にセット  
  M5.Lcd.setTextColor(WHITE, BLACK);
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 0);
  // 文字を描画
  M5.Lcd.printf("M5STACK CORE2 IMU SENDER");

  // Bluetooth
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 40);
  // 文字を描画
  M5.Lcd.printf("Bluetooth name");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 52);
  // 文字を描画
  M5.Lcd.printf(BLUETOOTH_SERIAL_NAME);
  
  // Acceleration
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 90);
  // 文字を描画
  M5.Lcd.printf("accX,  accY,  accZ");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 102);
  // 文字を描画
  M5.Lcd.printf("%6.2f %6.2f %6.2f", accX, accY, accZ);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(240, 102);
  // 文字を描画
  M5.Lcd.printf(" G");
  
  // Gyro
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 120);
  // 文字を描画
  M5.Lcd.printf("gyroX,  gyroY,  gyroZ");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 132);
  // 文字を描画
  M5.Lcd.printf("%6.2f %6.2f %6.2f", gyroX, gyroY, gyroZ);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(240, 132);
  // 文字を描画
  M5.Lcd.printf(" o/s");
  
  // Ahrl
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 150);
  // 文字を描画
  M5.Lcd.printf("pitch,  roll,  yaw");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 162);
  // 文字を描画
  M5.Lcd.printf("%6.2f %6.2f %6.2f", pitch, roll, yaw);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(240, 162);
  // 文字を描画
  M5.Lcd.printf(" deg");

  // 更新時間の調整のため、動作を停止させる
  delay(int(round(1000.0/FPS)));
}
