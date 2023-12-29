//M5StickCの3DアンテナパターンをESP-NOWで測定する送信機・受信機を制作する
//まずは二次元のアンテナパターンを測定する。
//送信機は姿勢を変えながらESP-NOW信号を送信し、受信機で送信機の3Dパターンを測定し、結果をcsvでシリアル出力する。

//2Dアンテナパターンの測定手順
//モード選択ボタンにより、送信機または受信機をモード選択する。
//開始ボタンを押すことで、送信機・受信機の動作を開始する。
//送受信機共に初めに静止下で加速度・ジャイロのキャリブレーションを行い、変数として3軸加速度・3軸ジャイロの値をキャリブレーション値を持つ。
//送信機は、ESP-NOWで送信電力と姿勢(2Dアンテナパターンではyawのみ)を受信機に送信する。
//受信機は、ESP-NOWで送信機から送信された姿勢を受信、並びにRSSIをもとに送信機の2Dパターンを測定する。
//送信機・受信機共に、測定終了後にシリアル出力を行う。
//受信機がM5StickCやM5Stack等の画面付きのマイコンであれば、2Dアンテナパターンを画面に描画する。


//後日実装する3Dアンテナパターンの実装内容
//-メイン機能の手順
//モード選択ボタンにより、送信機または受信機をモード選択する。
//開始ボタンを押すことで、送信機・受信機の動作を開始する。
//送受信機共に初めに静止下で加速度・ジャイロのキャリブレーションを行い、変数として3軸加速度・3軸ジャイロの値をキャリブレーション値を持つ。
//送信機は、ESP-NOWで送信電力と姿勢(roll・pitch・yaw)を受信機に送信する。
//受信機は、ESP-NOWで送信機から送信された電力と姿勢を受信し、送信機の3Dパターンを測定する。
//送信機・受信機共に、測定終了後にシリアル出力を行う。
//受信機がM5StickCやM5Stack等の画面付きのマイコンであれば、3Dアンテナパターンを画面に描画する。
//-サブ機能
//EPS32以外でWiFi送信できる任意の送信機の3Dアンテナパターンを測定するため、送信機の姿勢をパンチルトモーターで制御するパンチルト制御器を設ける。
//パンチルト制御器は、受信機からのI2CまたはESP-NOWの制御信号をもとに、パンチルト(roll・pitch・yaw)を制御する。



//2Dアンテナパターンの測定
//Initialization
//モード選択ボタンのピン番号
//#define MODE_SELECT_BUTTON_PIN 0
//開始ボタンのピン番号
//#define START_BUTTON_PIN 0


//M5StickCによる2Dアンテナパターンの測定
#include <M5StickC.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <esp_now.h>


void calibrateMPU6886();


//送信機・受信機のESP-NOWチャンネル
#define CHANNEL 10
//送信機のESP-NOW送信間隔(ms)
#define TX_INTERVAL 100
//受信機のESP-NOW受信間隔(ms)
#define RX_INTERVAL 500
//送信機のESP-NOW送信電力(dBm)
#define TX_POWER 20

//送信機または受信機の3軸加速度・3軸ジャイロ変数、並びにキャリブレーション値
float accX = 0, accY = 0, accZ = 0;
float accOffsetX = -0.01, accOffsetY = -0.01, accOffsetZ = 0.09;
float gyroX = 0, gyroY = 0, gyroZ = 0;  
float gyroOffsetX = 3.93, gyroOffsetY = -14.56, gyroOffsetZ = 2.48;  
float pitch = 0.0,roll=0.0,yaw=0.0;
float pitchRef=0 , rollRef=0 , yawRef=0;
float pitchOffset=0 , rollOffset=0 , yawOffset=-8.5;

//送信機の姿勢(yaw)
float tx_yaw;
//受信機の姿勢(yaw)
float rx_yaw;

//モード変数の設定
String trxmode;

//===ESPNOWの設定===
// 送信先のESP32のMACアドレス。ブロードキャストアドレスを利用。
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// 送信するデータ構造
typedef struct struct_message {
    int id;
    float yaw;
} struct_message;

// 送信するデータ
struct_message txData;
// コールバック関数（送信完了時）
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success");
  } else {
    Serial.println("Delivery Fail");
  }
}

// 受信データ
struct_message rxData;
// コールバック関数（データ受信時）
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&rxData, incomingData, sizeof(rxData));
  /*
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("ID: ");
  Serial.print(rxData.id);
  Serial.print(" yaw: ");
  Serial.println(rxData.yaw);
  */
  Serial.printf("Recv: %d,%d,%3f \n",len,rxData.id,rxData.yaw);
}

//===ESPNOWの設定ここまで====

//====IMU====
TaskHandle_t IMUHandle;
#define TASK_SLEEP_IMU 20 //10ms delay
static void IMULoop(void* arg);
boolean DEBUG_IMU=false;


//Setup
void setup() {
  //M5StickCの画面を初期化
  M5.begin();
  //M5StickCの画面を白くする
  M5.Lcd.fillScreen(TFT_WHITE);
  //M5StickCの画面に文字を表示
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("2D Antenna Pattern Measurement");
  M5.Lcd.println("Mode Select");
  M5.Lcd.println("1:TX");
  M5.Lcd.println("2:RX");

  //ボタンAが押された場合TxでSoftAPを起動、ボタンBが押された場合RxモードでWiFi STAを起動。ボタンが押されるまで待つ。
    while (1) {
    if (M5.BtnA.isPressed()) {
      M5.Lcd.fillScreen(TFT_WHITE);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.setTextColor(TFT_BLACK, TFT_WHITE);
      M5.Lcd.setTextSize(2);
      M5.Lcd.println("TX Mode");
      trxmode = "tx";

      break;
    }else if (M5.BtnB.isPressed()) {
      M5.Lcd.fillScreen(TFT_WHITE);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.setTextColor(TFT_BLACK, TFT_WHITE);
      M5.Lcd.setTextSize(2);
      M5.Lcd.println("RX Mode");
      trxmode ="rx";
      break;
    }
    Serial.println("waiting push");
    M5.update();
    delay(100);
  }

 //Calibration
  Wire.begin();
  M5.IMU.Init();
  calibrateMPU6886();
  xTaskCreatePinnedToCore(IMULoop, "IMULoop", 4096, NULL, 1, &IMUHandle, 1);

  
  // ESP-NOWの初期化
  if(trxmode=="tx"){
    WiFi.mode(WIFI_AP);
    WiFi.softAP("AntennaPatternerAP", "", CHANNEL, 0, 4,0, 500);
  }else if (trxmode=="rx"){
    WiFi.mode(WIFI_STA);
    WiFi.begin("AntennaPatternerAP", "");
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
  }


  sta.config(pm=sta.PM_NONE)

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // プロミスキャスモードを有効化
  //esp_wifi_set_promiscuous(true);
  //esp_wifi_set_promiscuous_rx_cb(promiscuousCallback);

 s
}

static void IMULoop(void* arg){
  //Detail is here. https://github.com/moononournation/Arduino_GFX
  while (1) {
    uint32_t entryTime = millis();

    //姿勢を取得
    M5.update();
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX,&accY,&accZ);
    M5.IMU.getAhrsData(&pitch,&roll,&yaw,accOffsetX,accOffsetY,accOffsetZ,gyroOffsetX,gyroOffsetY,gyroOffsetZ);
    accX -= accOffsetX;
    accY -= accOffsetY;
    accZ -= accOffsetZ;
    gyroX -=gyroOffsetX;
    gyroY -=gyroOffsetY;
    gyroZ -=gyroOffsetZ;       
    pitch -=pitchOffset; 
    roll -= rollOffset;
    yaw -= yawOffset; 

    if(DEBUG_IMU)Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f \n", accX, accY, accZ, roll,pitch,yaw);

    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_IMU  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}


WiFiUDP udp;
unsigned int localUdpPort = 4210;
char packetBuffer[255]; // 受信データバッファ


void loop() {

  //TX Mode
  if(trxmode == "tx"){
    //送信機の姿勢(yaw)を取得
    tx_yaw = yaw;
    
    //===送信機のESP-NOWでyawを送信===
    txData.id = 1;
    txData.yaw = tx_yaw;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    Serial.printf("%6.2f \n", txData.yaw);
    if (result == ESP_OK) {
      Serial.println("Send success");
    } else {
      Serial.println("Send error");
    }
    
    //===送信機のWiFi UDPでyawを送信===
    // UDPで送信
    /*udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(txData.yaw.c_str());
    udp.endPacket();
    */
    
    delay(TX_INTERVAL);
  }

  //RX Mode
  if(trxmode == "rx"){
    //受信機の姿勢(yaw)を取得
    rx_yaw = yaw;
    //送信機、受信機の姿勢とRSSIをシリアル出力
    Serial.printf("%6.2f %6.2f %3d %6.2f %6.2f %6.2f \n", rxData.yaw ,rx_yaw, WiFi.RSSI(), pitch, roll, yaw);


    delay(RX_INTERVAL);

    //2Dアンテナパターンを描画。RSSIとrxData.yawの関係を描画する。
  }
}


void calibrateMPU6886(){
  float gyroSumX,gyroSumY,gyroSumZ;
  float accSumX,accSumY,accSumZ;
  int calibCount = 1000;

  Serial.println("Calibrating...");
  digitalWrite(10, LOW);
  delay(2000);
  digitalWrite(10, HIGH);
  delay(100);
  digitalWrite(10, LOW); 
  for(int i = 0; i < calibCount; i++){
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX,&accY,&accZ);
      gyroSumX += gyroX;
      gyroSumY += gyroY;
      gyroSumZ += gyroZ;
      accSumX += accX;
      accSumY += accY;
      accSumZ += accZ;
      delay(2);
      //Serial.printf("%6.2f, %6.2f, %6.2f\r\n", accX, accY, accZ);
  }
  gyroOffsetX = gyroSumX/calibCount;
  gyroOffsetY = gyroSumY/calibCount;
  gyroOffsetZ = gyroSumZ/calibCount;
  accOffsetX = accSumX/calibCount;
  accOffsetY = accSumY/calibCount;
  accOffsetZ = (accSumZ/calibCount) - 1.0;//重力加速度1G、つまりM5ボタンが上向きで行う想定
  //accOffsetZ = (accSumZ/calibCount) + 1.0;//重力加速度1G、つまりM5ボタンが下向きで行う想定
  //accOffsetZ = (accSumZ/calibCount);//
  Serial.println("Calibrating...OK");
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", accOffsetX, accOffsetY, accOffsetZ, gyroOffsetX , gyroOffsetY , gyroOffsetZ);
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", accOffsetX, accOffsetY, accOffsetZ, gyroOffsetX , gyroOffsetY , gyroOffsetZ);
  digitalWrite(10, HIGH);
}