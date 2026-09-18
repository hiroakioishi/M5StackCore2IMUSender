#include <M5Core2.h>
//[追加]MahonyAHRSupdateIMU()を呼べるようにするため
#include <utility/MahonyAHRS.h>

// バージョン
#define VERSION "1.3_NormalSerial"
// シリアル通信の速度（Unity側の BaudRate と同じにする）
#define SERIAL_BAUD_RATE 115200
// シリアル通信時の名前
#define SERIAL_NAME "M5StackCore2_DI00"

// --- 更新間隔 ---
// 姿勢の計算に使う MahonyAHRS ライブラリは、「1秒に25回（40ミリ秒ごと）呼ばれる」前提で
// 計算するように作られている（ライブラリの中で sampleFreq = 25 と決められている）。
// 間隔がずれると、回した角度が実際より大きく/小さく計算されてしまうので、40ミリ秒ごとに更新する。
#define UPDATE_INTERVAL_MS 40

// --- ジャイロのゼロ点補正（キャリブレーション） ---
// 起動してから、センサーの値が落ち着くまで待つ時間（ミリ秒）
#define CALIBRATION_WAIT_MS 1000
// ゼロ点を求めるために、ジャイロの値を読む回数
#define CALIBRATION_SAMPLES 200
// 1回読むごとに待つ時間（ミリ秒）  200回 × 10ミリ秒 = 約2秒
#define CALIBRATION_SAMPLE_INTERVAL_MS 10
// 読んでいる間に、値がこれ以上ばらついたら「動かされた」と判断してやり直す（度/秒）
#define CALIBRATION_MOVE_LIMIT 5.0f

// --- 静止判定（デッドゾーン） ---
// ゼロ点補正をしたあとのジャイロの値がこれより小さいときは、「回っていない」とみなして 0 にする（度/秒）
// 大きくするとドリフトは減るが、ゆっくりした回転を感じ取れなくなる
#define GYRO_DEAD_ZONE 0.5f

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

// シリアルで値を送信するときに使用するバッファ
char buff[128];

// ジャイロのゼロ点のずれ（静止していても出てしまう値）
// calibrateGyro() で求めて、毎回ジャイロの値から引く
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

// 前回、値を更新した時刻（ミリ秒）
unsigned long lastUpdateMs = 0;

// ------ ジャイロのゼロ点補正 ---------------------
// 机に置いて動かさない状態で、ジャイロの値を何回も読んで平均を取る。
// 平均した値が「静止していても出てしまう値（ゼロ点のずれ）」なので、
// あとで毎回この値を引くことで補正する。
void calibrateGyro()
{
  // 画面に案内を表示
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Calibrating gyro...");
  M5.Lcd.println();
  M5.Lcd.println("Place it on a desk");
  M5.Lcd.println("and don't move it.");

  // センサーの値が落ち着くまで待つ
  delay(CALIBRATION_WAIT_MS);

  // うまく測れるまで、くり返す
  while (true)
  {
    // 合計（平均を出すため）
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
    // 最小値と最大値（ばらつき = 動かされたかどうか を調べるため）
    float minX =  10000.0f, minY =  10000.0f, minZ =  10000.0f;
    float maxX = -10000.0f, maxY = -10000.0f, maxZ = -10000.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
      float gx, gy, gz;
      M5.IMU.getGyroData(&gx, &gy, &gz);

      sumX += gx;
      sumY += gy;
      sumZ += gz;

      minX = fminf(minX, gx);  maxX = fmaxf(maxX, gx);
      minY = fminf(minY, gy);  maxY = fmaxf(maxY, gy);
      minZ = fminf(minZ, gz);  maxZ = fmaxf(maxZ, gz);

      delay(CALIBRATION_SAMPLE_INTERVAL_MS);
    }

    // ばらつきが小さければ（動かされていなければ）成功
    bool notMoved = (maxX - minX) < CALIBRATION_MOVE_LIMIT
                 && (maxY - minY) < CALIBRATION_MOVE_LIMIT
                 && (maxZ - minZ) < CALIBRATION_MOVE_LIMIT;

    if (notMoved)
    {
      // 平均を、ゼロ点のずれとして記録する
      gyroOffsetX = sumX / CALIBRATION_SAMPLES;
      gyroOffsetY = sumY / CALIBRATION_SAMPLES;
      gyroOffsetZ = sumZ / CALIBRATION_SAMPLES;
      break;
    }

    // 動かされていたので、やり直す
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setCursor(0, 120);
    M5.Lcd.println("Moved! Retrying...");
    M5.Lcd.setTextColor(WHITE, BLACK);
    delay(CALIBRATION_WAIT_MS);
  }

  // 画面を消して、通常の表示に戻る
  M5.Lcd.fillScreen(BLACK);
}

// ------ 小さすぎる値を 0 にする（デッドゾーン） ---------------------
float applyDeadZone(float value)
{
  if (fabsf(value) < GYRO_DEAD_ZONE)
  {
    return 0.0f;
  }
  return value;
}

// ------ 初期化 ---------------------
void setup()
{
  // 本体初期化
  M5.begin();
  // シリアル初期化
  Serial.begin(SERIAL_BAUD_RATE);
  // 0.5秒待機
  delay(500);

  // 慣性測定ユニット初期化
  M5.IMU.Init();

  // ジャイロのゼロ点補正（この間は M5Stack を動かさない）
  calibrateGyro();

  // 画面の背景を黒で塗る
  M5.Lcd.fillScreen(BLACK);
  // 文字の色を白, 文字背景を黒にセット
  M5.Lcd.setTextColor(WHITE, BLACK);

  // 更新時刻の基準を、今の時刻にする
  lastUpdateMs = millis();
}

// ------ 更新 ---------------------
void loop()
{
  // --- 40ミリ秒ごとに1回だけ処理する ---
  // 前回の更新から UPDATE_INTERVAL_MS 経っていなければ、何もせずに戻る
  unsigned long now = millis();
  if (now - lastUpdateMs < UPDATE_INTERVAL_MS)
  {
    return;
  }
  // 次の更新時刻を決める
  // （delay を使わずに、きっちり 40ミリ秒ごとになるように「前回の時刻 + 40」にする）
  lastUpdateMs += UPDATE_INTERVAL_MS;
  // 画面の塗りつぶし等で処理が大きく遅れたときは、今の時刻から数え直す
  if (now - lastUpdateMs >= UPDATE_INTERVAL_MS)
  {
    lastUpdateMs = now;
  }

  // ボタンの状態を更新
  M5.update();

  // --- IMUの値を取得 ---
  // 加速度
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  // ジャイロ
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);

  // --- ジャイロの補正 ---
  // 1. ゼロ点のずれを引く
  gyroX -= gyroOffsetX;
  gyroY -= gyroOffsetY;
  gyroZ -= gyroOffsetZ;
  // 2. とても小さい値は 0 にする（置いたままのときに、Yaw が少しずつ流れるのを防ぐ）
  gyroX = applyDeadZone(gyroX);
  gyroY = applyDeadZone(gyroY);
  gyroZ = applyDeadZone(gyroZ);

  // --- 姿勢（pitch・roll・yaw）を計算する ---
  // 加速度と、補正したジャイロの値を使う（ジャイロは 度/秒 → ラジアン/秒 に変換して渡す）
  MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, &pitch, &roll, &yaw);

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

  // --- IMUの値, ボタンの状態をUSBシリアルで送信 ---
  // buffに、Accel, Gyro, Ahrs, ボタンの値をまとめる
  snprintf(buff, sizeof(buff), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\r\n",
           accX, accY, accZ, gyroX, gyroY, gyroZ, pitch, roll, yaw, buttonA, buttonB, buttonC);
  // USBシリアルで送信
  Serial.print(buff);


  // --- 液晶画面にセンサーの値などの情報を表示する ---
  // 文字の色を白にセット
  M5.Lcd.setTextColor(WHITE, BLACK);
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 0);
  // 文字を描画
  M5.Lcd.print("M5STACK CORE2 IMU SENDER");

  // Version
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 18);
  // 文字を描画
  M5.Lcd.printf("Version : %s", VERSION);

  // 通信方法
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 40);
  // 文字を描画
  M5.Lcd.print("Connection");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 52);
  // 文字を描画
  M5.Lcd.print(SERIAL_NAME);

  // Acceleration
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 90);
  // 文字を描画
  M5.Lcd.print("accX,  accY,  accZ");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 102);
  // 文字を描画
  M5.Lcd.printf("%6.2f %6.2f %6.2f", accX, accY, accZ);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(240, 102);
  // 文字を描画
  M5.Lcd.print(" G");

  // Gyro
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 120);
  // 文字を描画
  M5.Lcd.print("gyroX,  gyroY,  gyroZ");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 132);
  // 文字を描画
  M5.Lcd.printf("%6.2f %6.2f %6.2f", gyroX, gyroY, gyroZ);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(240, 132);
  // 文字を描画
  M5.Lcd.print(" o/s");

  // Ahrs
  // 文字のサイズを 1 にセット
  M5.Lcd.setTextSize(1);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 150);
  // 文字を描画
  M5.Lcd.print("pitch,  roll,  yaw");
  // 文字のサイズを 2 にセット
  M5.Lcd.setTextSize(2);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(0, 162);
  // 文字を描画
  M5.Lcd.printf("%6.2f %6.2f %6.2f", pitch, roll, yaw);
  // 文字の描画位置を移動
  M5.Lcd.setCursor(240, 162);
  // 文字を描画
  M5.Lcd.print(" deg");

  // ※ 以前はここで delay() していたが、loop() の最初で 40ミリ秒ごとに処理するようにしたので不要
}
