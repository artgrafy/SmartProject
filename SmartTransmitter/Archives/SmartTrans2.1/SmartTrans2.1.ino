/*
  스마트 안전관리 웨어러블 디바이스 펌웨어
  BLE BroadCasting Transmitter Ver 2.1
  Edited in 2025.07.21 by Dr.Yoo / Softronics
  
  Lib list
  - HS300x : humidity, temperature
  - LPS22HB : Pressure
  - BMI270_BMM150 : IMU
  - PDM : Microphone, Noise
*/

#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>

/////////////// 디바이스 고유 이름 설정 ////////////////
const char* LocalName = "SENSE03"; // 고정 디바이스는 "CONTROL", 개인 휴대 디바이스는 "SENSE01" ~ "SENSE10"
///////////////

// 임의의 제조사 ID
const uint16_t MANUFACTURER_ID = 0xFFFF;

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

// BLE 광고 타이머
unsigned long lastAdvertiseTime = 0;
const unsigned long ADVERTISE_INTERVAL = 1000; // 1초 간격

// 스트레치 센서 변수 //
const int analogPin = A0;
unsigned long lastStretchTime = 0;
int stretchCount = 0;
float midR =  60.0; // 스트레치 센서 특성(중간값)
float preR; // 이전 저항값

// 마이크 버퍼
#define SAMPLE_BUFFER_SIZE 256
short sampleBuffer[SAMPLE_BUFFER_SIZE];
volatile int samplesRead;

// BLE 송신 - 총 패킷 24byes로 구성
byte manufacturerData[24];

// 마이크 데이터 수신 콜백
void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

// 초기 설정 //
void setup() {
  Serial.begin(9600);
  delay(3000);  // 시리얼 포트가 준비될 때까지 잠시 대기
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
  PDM.onReceive(onPDMdata);  // 콜백 함수 설정
  
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize microphone!");
    while (1);
  }
  Serial.println("PDM initialized successfully.");

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

  // 스트레치 초기화 (10초 측정 후 midR 계산)
  float maxR = 0.0;
  float minR = 10000.0;
  unsigned long startTime = millis();
  while (millis() - startTime < 10000.0) {
    int sensorValue = analogRead(analogPin);
    float voltage = sensorValue * (3.3 / 1023.0); // 전압 계산
    float knownResistor = 216.0; // 기준 저항 216 Ohms
    float resistor = knownResistor * (3.3 - voltage) / voltage; // 저항 계산
    
    if (resistor > maxR) maxR = resistor;
    if (resistor < minR) minR = resistor;
  }
  midR = (maxR + minR) / 2.0;

}

void loop() {
  unsigned long currentTime = millis(); // 현재 경과 시간
 
  temperature = HS300x.readTemperature(); // 온도
  humidity    = HS300x.readHumidity(); // 습도
  pressure = BARO.readPressure(); // 기압

  // 마이크 소음(dB) 측정
  if (samplesRead) {
    long sumSquares = 0;
    
    // RMS 계산
    for (int i = 0; i < samplesRead; i++) {
      sumSquares += (long)sampleBuffer[i] * sampleBuffer[i];
    }
    
    float rms = sqrt(sumSquares / (float)samplesRead);
    noise = 20.0 * log10(rms);
    if (isnan(noise) || noise < 0) noise = 0.0;
    
    samplesRead = 0;  // 버퍼 초기화
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
    float gyroPitch = pitch + gx * dt;
    float gyroRoll = roll + gy * dt;

    // 보완 필터 적용
    pitch = alpha * gyroPitch + (1 - alpha) * accelPitch;
    roll = alpha * gyroRoll + (1 - alpha) * accelRoll;

    accel = ax * ax + ay * ay + az * az;
    angle = pitch; // 보드 세로 각도
  }

  ///////////////////////////////// 호흡수 루틴 - 개발 중
  
  // 저항 측정
  int sensorValue = analogRead(analogPin);
  float voltage = sensorValue * (3.3 / 1023.0); // 전압 계산
  float knownResistor = 10000.0; // 기준 저항 10KOhms
  float resistor = knownResistor * (3.3 - voltage) / voltage; // 저항 계산
  float deltaT;
  
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
  ///////////////////////////////// 호흡수 루틴


  /////////////// BLE 전송 간격 ///////////////
  // 1초 간격으로 BLE 광고 전송
  if (currentTime - lastAdvertiseTime >= ADVERTISE_INTERVAL) {
    // BLE Broadcasting
    memcpy(manufacturerData, &temperature, sizeof(temperature));
    memcpy(manufacturerData + 4, &humidity, sizeof(humidity));
    memcpy(manufacturerData + 8, &accel, sizeof(accel));
    memcpy(manufacturerData + 12, &angle, sizeof(angle));
    memcpy(manufacturerData + 16, &stretch, sizeof(stretch));
    memcpy(manufacturerData + 20, &noise, sizeof(noise));
    BLE.setManufacturerData(MANUFACTURER_ID, manufacturerData, sizeof(manufacturerData));
    BLE.advertise();
    
    // 센서값 출력
    Serial.print("온도: "); Serial.print(temperature);
    Serial.print(", 습도: "); Serial.print(humidity);
    Serial.print(", 가속: "); Serial.print(accel);
    Serial.print(", 각도: "); Serial.print(angle);
    Serial.print(", 호흡: "); Serial.print(resistor);
    Serial.print(", 소음: "); Serial.println(noise);
    
    lastAdvertiseTime = currentTime;  // 마지막 전송 시간 업데이트
  }
}

