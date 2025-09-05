#include <M5Core2.h>
//[追加]MahonyAHRSupdateIMU()を呼べるようにするため
#include <utility/MahonyAHRS.h>

// バージョン
#define VERSION "1.2_NormalSerial"
// FPS
#define FPS 30

// 加速度 (Acceleration)
float accX = 0.0f;
float accY = 0.0f;
float accZ = 0.0f;
// ジャイロ (Gyroscope) (角速度)
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
// 姿勢方位基準システム (Attitude and Heading Reference System)
float pitch = 0.0f;
float roll  = 0.0f;
float yaw   = 0.0f;
// ボタンA, B, Cがそれぞれ押されているかどうか
// 1: 押されている, 0: 押されていない
int buttonA = 0;
int buttonB = 0;
int buttonC = 0;

// Bluetoothで値を送信するときに使用するバッファ
char buff[80];

// GyroZ 補正コードはこの記事を参照
// https://qiita.com/foka22ok/items/53d5271a21313e9ddcbd
//[追加]GyroZのデータを蓄積するための変数
float stockedGyroZs[10];
int stockCnt=0;
float adjustGyroZ=0;
int stockedGyroZLength=0;

// ------ 初期化 ---------------------
void setup()
{
  // 保持しておくGyroZのサイズを計算
  stockedGyroZLength = sizeof(stockedGyroZs) / sizeof(int);

  // 本体初期化
  M5.begin();
  // シリアル初期化
  Serial.begin(9600);
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
  //[変更]これは使わない
  // M5.IMU.getAhrsData(&pitch,&roll,&yaw); 

  //[追加]起動時にstockedGyroZLengthの数だけデータを貯める
  if(stockCnt<stockedGyroZLength){
    stockedGyroZs[stockCnt]=gyroZ;
    stockCnt++;
  }else{
    if(adjustGyroZ==0){
      for(int i=0;i<stockedGyroZLength;i++){
        adjustGyroZ+=stockedGyroZs[i]/stockedGyroZLength;
      }
    }
    //貯めたデータの平均値を使ってgyroZを補正する
    gyroZ-=adjustGyroZ; 
    //ここでaccelデータと補正したgyroデータを使ってpitch・roll・yawを計算する
    MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, &pitch, &roll, &yaw);
  }

  // --- ボタン操作の処理 ---
  // A（左）ボタンが押されたら
  if (M5.BtnA.wasPressed())
  {
    // 文字色を白、文字背景を赤にする
    M5.Lcd.setTextColor(WHITE, RED);
    // 画面の背景を赤で塗る
    M5.Lcd.fillScreen(RED);
    // ボタンAは押されている
    buttonA = 1;
  }
  // A（左）ボタンが離されたら
  if (M5.BtnA.wasReleased())
  {
    // 文字の色を白, 文字背景を黒にセット  
    M5.Lcd.setTextColor(WHITE, BLACK);
    // 画面の背景を黒で塗る
    M5.Lcd.fillScreen(BLACK);
    // ボタンAは押されていない
    buttonA = 0;
  }
  // B（中）ボタンが押されたら
  if (M5.BtnB.wasPressed())
  {
    // 文字色を白、文字背景を緑にする
    M5.Lcd.setTextColor(WHITE, GREEN);
    // 画面の背景を緑で塗る
    M5.Lcd.fillScreen(GREEN);
    // ボタンBは押されている
    buttonB = 1;
  }
  // B（中）ボタンが離されたら
  if (M5.BtnB.wasReleased())
  {
    // 文字の色を白, 文字背景を黒にセット  
    M5.Lcd.setTextColor(WHITE, BLACK);
    // 画面の背景を黒で塗る
    M5.Lcd.fillScreen(BLACK);
    // ボタンBは押されていない
    buttonB = 0;
  }
  // C（右）ボタンが押されたら
  if (M5.BtnC.wasPressed())
  {
    // 文字色を白、文字背景を青にする
    M5.Lcd.setTextColor(WHITE, BLUE);
    // 画面の背景を青で塗る
    M5.Lcd.fillScreen(BLUE);
    // ボタンCは押されている
    buttonC = 1;
  }
  // C（右）ボタンが離されたら
  if (M5.BtnC.wasReleased())
  {
    // 文字の色を白, 文字背景を黒にセット  
    M5.Lcd.setTextColor(WHITE, BLACK);
    // 画面の背景を黒で塗る
    M5.Lcd.fillScreen(BLACK);
    // ボタンCは押されていない
    buttonC = 0;
  }
  
  // --- IMUの値, ボタンの状態をBluetoothで送信 ---
  // buffに、Accel, Gyro, Ahrsの値をまとめる 
  //sprintf(buff, "%5.2f,%5.2f,%5.2f\r\n", pitch, roll, yaw);
  sprintf(buff, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%d,%d,%d\r\n", accX, accY, accZ, gyroX, gyroY, gyroZ, pitch, roll, yaw, buttonA, buttonB, buttonC);
  // Bluetoothで、Accel, Gyro, Ahrs の値を送信
  // serialBT.printf(buff);
  Serial.printf(buff);


  // --- 液晶画面にセンサーの値などの情報を表示する ---
  // 文字の色を白にセット  
  M5.Lcd.setTextColor(WHITE, BLACK);
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 0);
  // 文字を描画
  M5.Lcd.printf("M5STACK CORE2 IMU SENDER");

  // Version
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 18);
  // 文字を描画
  M5.Lcd.printf("Version : %s", VERSION);

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
