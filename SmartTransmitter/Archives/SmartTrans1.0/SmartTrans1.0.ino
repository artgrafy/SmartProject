/*
  안전관리 웨어러블 디바이스
  BLE BroadCasting Transmitter
  Ver 1.0
  by Softronics
  by Dr.Yoo

  HS300x : humidity, temperature
  LPS22HB : Pressure
  BMI270_BMM150 : IMU
*/

#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_BMI270_BMM150.h>

// 송신기 ID 설정 //
const char* LocalName = "SENSE01"; // 로칼 네임
const uint16_t MANUFACTURER_ID = 0xFFFF; // 임의의 제조사 ID

// 센서 특성 데이터 4+20 bytes
int dataType; // 송신데이터 유형
float temperature;
float humidity;
float accel;
float angle;
float stretch;

// IMU 변수 설정 //
const float alpha = 0.9; // 자이로 의존도 (max=1.0) 
unsigned long lastIMUTime = 0;
float dt; // 시간 간격(초)
float pitch, roll;
float lastGyroX, lastGyroY;

// 스트레치 센서 설정 //
const int analogPin = A0;
unsigned long lastStretchTime = 0;
int stretchCount = 0;
float midR =  60.0; // 스트레치 센서 특성(중간값)
float preR; // 이전 저항값

//// 초기 설정 ////
void setup() {

  Serial.begin(9600);
  //while (!Serial);
  Serial.println("Serial Started");

  // temperature & humidity 초기화
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  Serial.println("Humidity and temperature sensor initialized successfully.");

  // pressure 초기화
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    //while (1);
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
  Serial.println();
  
  // 스트레치 센서 초기화, 10초 측정 후 midR 계산
  float maxR = 0.0;
  float minR = 10000.0;
  unsigned long startTime = millis();
  unsigned long stopTime;
  unsigned long elapseTime = 0;
  while (elapseTime < 10000) {
    int sensorValue = analogRead(analogPin);
    float voltage = sensorValue * (3.3 / 1023.0); // 전압 계산
    float knownResistor = 216.0; // 기준 저항 216 Ohms
    float resistor = knownResistor * (3.3 - voltage) / voltage; // 저항 계산
    
    Serial.println(); Serial.print("R= "); Serial.print(resistor);
  
    if (resistor > maxR) {
      maxR = resistor;
    }

    if (resistor < minR) {
      minR = resistor;
    }
    stopTime = millis();
    elapseTime = (stopTime - startTime / 1000.0);
    Serial.println(elapseTime);
  }

  midR = (maxR + minR) / 2.0;
  Serial.println(); Serial.print("midR= "); Serial.print(midR);

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
  Serial.println("BLE advertising.");

// print an empty line
  Serial.println();
}

//// 메인 루프 ////
void loop() {

  unsigned long currentTime = millis(); // 현재 경과 시간
  dataType = 1; // 기본 데이터 유형

  // 환경 센서 측정 //
  temperature = HS300x.readTemperature(); // 온도
  humidity    = HS300x.readHumidity(); // 습도
  // pressure = BARO.readPressure(); // 기압
  
  // // 환경 센서값 출력
  // Serial.print("Temperature = ");
  // Serial.print(temperature);
  // Serial.println(" °C");
  // Serial.print("Humidity    = ");
  // Serial.print(humidity);
  // Serial.println(" %");
  // Serial.println();

  // IMU 측정 //
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

  // // 가속도 & 각도 출력
  // Serial.print("Acceleration = ");
  // Serial.println(accel);
  // Serial.print("Angle = ");
  // Serial.println(angle);
  // Serial.println();
  
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

  // 10회 호흡 마다 분당 호흡수 계산
  if (stretchCount == 10) { 
    currentTime = millis();
    deltaT = currentTime - lastStretchTime;
    stretch = 60000.0 / (deltaT / 10.0);
    Serial.println(); Serial.print("호흡 10회, 시간: "); Serial.print(deltaT/1000.0); Serial.print("sec, 호흡수: "); Serial.println(stretch);
    
    stretchCount = 0;
    lastStretchTime = currentTime;
  }

  stretch = resistor;
  preR = resistor;
  
  // BLE 송신 - 
  byte manufacturerData[24];
  memcpy(manufacturerData, &dataType, sizeof(dataType));
  memcpy(manufacturerData + 4, &temperature, sizeof(temperature));
  memcpy(manufacturerData + 8, &humidity, sizeof(humidity));
  memcpy(manufacturerData + 12, &accel, sizeof(accel));
  memcpy(manufacturerData + 16, &angle, sizeof(angle));
  memcpy(manufacturerData + 20, &stretch, sizeof(stretch));
  
  // 제조자 데이터
  BLE.setManufacturerData(MANUFACTURER_ID, manufacturerData, sizeof(manufacturerData));
  
   // 광고 다시 시작
  BLE.advertise();  // 데이터 갱신 후 광고 다시 시작
  // Serial.println("Broadcasting new data.");
  // Serial.println(); 
  
  // delay to transmit again
  delay(10);
}

// 코드 보관
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
