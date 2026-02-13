/*
  스마트 안전관리 웨어러블 디바이스 펌웨어
  BLE BroadCasting Transmitter Ver 2.0
  Edited in 2025.07.08 by Dr.Yoo / Softronics
  HS300x : humidity, temperature
  LPS22HB : Pressure
  BMI270_BMM150 : IMU
  PDM : Microphone, Noise
*/

#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>

// 송신기 ID 설정 //
const char* LocalName = "SENSE01"; // 로칼 네임
const uint16_t MANUFACTURER_ID = 0xFFFF; // 임의의 제조사 ID

// 센서 데이터 총 6 * 4bytes로 구성
int dataType; // 데이터유형
float temperature, humidity, pressure;
float accel, angle, stretch;
float noise = 0.0;

// IMU 변수 //
const float alpha = 0.9; // 자이로 의존도 (max=1.0) 
unsigned long lastIMUTime = 0;
float dt; // 시간 간격(초)
float pitch, roll;
//float lastGyroX, lastGyroY;

// 스트레치 센서 변수 //
const int analogPin = A0;
unsigned long lastStretchTime = 0;
int stretchCount = 0;
float midR =  60.0; // 스트레치 센서 특성(중간값)
float preR; // 이전 저항값

// 마이크 버퍼
#define SAMPLE_BUFFER_SIZE 256
int16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
volatile int samplesRead = 0;

// BLE 송신 - 총 패킷 24byes로 구성
byte manufacturerData[24];

// 마이크 데이터 수신 콜백
void onPDMdata() {
  int bytesAvailable = PDM.available();
  Serial.print("BytesAvailable: "); Serial.println(bytesAvailable);
  if (bytesAvailable > 0) {
    PDM.read(sampleBuffer, bytesAvailable);
    samplesRead = bytesAvailable / 2;
  }
}

// 초기 설정 //
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial Started");

  // 온습도 초기화
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  Serial.println("Humidity and temperature sensor initialized successfully.");

  // 기압 초기화
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
  Serial.println("Pressure sensor initialized successfully.");
  
  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU sensor initialized successfully.");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  // 마이크(PDM) 초기화
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize microphone!");
    while (1);
  }
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(SAMPLE_BUFFER_SIZE);
  Serial.println("PDM initialized successfully.");

  // 스트레치 중간값 보정 (10초 측정 후 midR)
  float maxR = 0.0;
  float minR = 10000.0;
  unsigned long startTime = millis();
  while (millis() - startTime < 10000.0) {
    int sensorValue = analogRead(analogPin);
    float voltage = sensorValue * (3.3 / 1023.0); // 전압 계산
    float knownResistor = 216.0; // 기준 저항 216 Ohms
    float resistor = knownResistor * (3.3 - voltage) / voltage; // 저항 계산
    
    //Serial.println(); Serial.print("R= "); Serial.print(resistor);
  
    if (resistor > maxR) maxR = resistor;
    if (resistor < minR) minR = resistor;
  }
  midR = (maxR + minR) / 2.0;
  //Serial.println(); Serial.print("midR= "); Serial.print(midR);

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("BLE 초기화 실패!");
    while (1);
  }
  Serial.println("BLE initialized successfully.");

  BLE.setLocalName(LocalName);
  if (!BLE.advertise()) {
    Serial.println("Failed to start BLE advertising.");
    while(1);
  }
  
}

//// 메인 루프 ////
void loop() {

  unsigned long currentTime = millis(); // 현재 경과 시간
  dataType = 1; // 기본 데이터 유형

  // 센서 측정 //
  temperature = HS300x.readTemperature(); // 온도
  humidity    = HS300x.readHumidity(); // 습도
  pressure = BARO.readPressure(); // 기압
  //Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");

  // 마이크 소음(dB) 측정
  if (samplesRead > 0) {
  long sumSquares = 0;
  for (int i = 0; i < samplesRead; i++) {
    sumSquares += sampleBuffer[i] * sampleBuffer[i];
  }
  float rms = sqrt(sumSquares / (float)samplesRead);
  noise = 20.0 * log10(rms);
  if (isnan(noise) || noise < 0) noise = 0.0;
  samplesRead = 0;

  Serial.print("Noise (dB): ");
  Serial.println(noise);
} else {
  //Serial.println("No mic data read yet.");
}

  // IMU 가속도, 각도 측정 //
  dt = (currentTime - lastIMUTime) / 1000.0; // ms
  lastIMUTime = currentTime;

  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {  
    IMU.readAcceleration(ax, ay, az); // 가속도
    IMU.readGyroscope(gx, gy, gz); // 자이로

    // 가속도계로 Pitch와 Roll 계산
    float accelPitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float accelRoll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

    // 자이로스코프 각속도를 각도로 변환
    float gyroPitch = pitch + gx * dt; // Pitch = 이전 각도 + 자이로스코프 각속도 * 시간
    float gyroRoll = roll + gy * dt;   // Roll = 이전 각도 + 자이로스코프 각속도 * 시간

    // 보완 필터 적용: 자이로스코프와 가속도계 데이터를 결합
    pitch = alpha * gyroPitch + (1 - alpha) * accelPitch; // standing angle 계산
    roll = alpha * gyroRoll + (1 - alpha) * accelRoll;

    accel = ax * ax + ay * ay + az * az;
    angle = pitch; // 보드 세로 각도

    // Serial.print("Pitch: "); Serial.print(pitch);
    // Serial.print(", Roll: "); Serial.println(roll);
  }

  // 저항 측정 //
  int sensorValue = analogRead(analogPin);
  float voltage = sensorValue * (3.3 / 1023.0); // 전압 계산
  float knownResistor = 216.0; // 기준 저항 216 Ohms
  float resistor = knownResistor * (3.3 - voltage) / voltage; // 저항 계산
  float deltaT;

  //Serial.println(); Serial.print("R= "); Serial.print(resistor);
  
  // midR 상향 돌파
  if ((preR < midR) && (resistor > midR)) {
    stretchCount = stretchCount + 1;
    Serial.print(" 들숨 확인 "); Serial.print(stretchCount);
  }  

  // 들숨 10회 시간으로 호흡수 측정
  if (stretchCount == 10) { 
    currentTime = millis();
    deltaT = currentTime - lastStretchTime;
    stretch = 60000.0 / (deltaT / 10.0); // 분당 호흡수
    Serial.println(); Serial.print("호흡 10회, 시간: "); Serial.print(deltaT/1000.0); Serial.print("sec, 호흡수: "); Serial.println(stretch);
    
    stretchCount = 0;
    lastStretchTime = currentTime;
  }
  preR = resistor; // 이전 저항값 업데이트
  
  // BLE Broadcasting
  memcpy(manufacturerData, &temperature, sizeof(temperature));
  memcpy(manufacturerData + 4, &humidity, sizeof(humidity));
  memcpy(manufacturerData + 8, &accel, sizeof(accel));
  memcpy(manufacturerData + 12, &angle, sizeof(angle));
  memcpy(manufacturerData + 16, &stretch, sizeof(stretch));
  memcpy(manufacturerData + 20, &noise, sizeof(noise));
  BLE.setManufacturerData(MANUFACTURER_ID, manufacturerData, sizeof(manufacturerData));
  BLE.advertise();
  
  // BLE 송신 센서값 터미널 출력
  //Serial.print("온도: "); Serial.print(temperature);  // °C
  //Serial.print(", 습도: "); Serial.print(humidity);     // %
  //Serial.print(", 가속: "); Serial.print(accel);      // m/sec2
  //Serial.print(", 각도: "); Serial.print(angle);      // °
  //Serial.print(", 호흡: "); Serial.print(stretch);    // 회/분
  Serial.print(", 소음: "); Serial.print(noise);        // dB  

  delay(100); // 다음 광고 주기
}

///// 호흡수 코드 개발 중
  // float deltaR, limitR;
  // // MaxR, MidR 업데이트
  // resistorCount = resistorCount + 1;
  // resistorSum = resistorSum + resistor;
  // midR = resistorSum / resistorCount; // 저항 평균
  // if (resistor > maxR) {
  //   maxR = resistor; // 최고점 업데이트
  // }
  
  // // 들숨/날숨 호흡 모드 분석
  // deltaR = resistor - midR; // 저항 변화
  // limitR = (maxR - midR) * 0.90; // 90%
  // if (deltaR > limitR) {
  //   stretchMode = 1; // 최고점과 비교
  // }
  // if (deltaR < -limitR) {
  //   stretchMode = -1; // 날숨
  // }

  // Serial.println(); Serial.print("limit= "); Serial.print(limitR);
  // Serial.print(" delta= "); Serial.print(deltaR);
  // Serial.print(" Mode: "); Serial.print(stretchMode);

  // // 들숨/날숨 모드 전환 판단
  // if ((lastStretchMode * stretchMode) == -1) {
  //   stretchCount = stretchCount + 1; // 호흡수 증가
  //   Serial.println(); Serial.print("모드전환"); Serial.println(stretchMode);

  //   lastStretchMode = stretchMode;
  // }

  // // 분당 호흡수 계산
  // if (stretchCount > 9) { 
  //   currentTime = millis();
  //   deltaT = currentTime - lastStretchTime;
  //   stretch = 60000.0 / (deltaT / 5.0);
  //   Serial.print("호흡모드 전환: 10회, 시간: "); Serial.print(deltaT/1000.0); Serial.print("sec, 호흡수: "); Serial.println(stretch);
    
  //   stretchCount = 0;
  //   lastStretchTime = currentTime;
  // }

  // // 저항 변동 리셋
  // if (resistorCount == 10) {
  //   maxR = midR + (maxR - midR) * 0.80; // 10회 마다 최대값 Reset
  //   resistorCount = 0;
  //   resistorSum = 0.0;  
  // }
