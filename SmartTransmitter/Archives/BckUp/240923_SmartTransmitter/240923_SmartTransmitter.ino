/*
  안전관리 웨어러블 디바이스
  BLE BroadCasting Transmitter
  Ver 0.1
  by Softronics
  by Dr.Yoo

  - Arduino Nano 33 BLE Sense R2
  - HS300x
    humidity
    temperature
  - LPS22HB
    Pressure
  - BMI270_BMM150
    IMU
*/
#include <ArduinoBLE.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_BMI270_BMM150.h>

// 송신기 ID 설정 //
const char* LocalName = "SENSE03"; // 로칼 네임
const uint16_t MANUFACTURER_ID = 0xFFFF; // 임의의 제조사 ID

// RF 설정
// RF24 radio(7, 8); // CE, CSN 핀 
// const byte address[6] = "00002"; // 송신 주소

// 센서 특성 데이터 20bytes
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
int lastStretching = -1;
float lastResistor = 0.0;
int stretchCount = 0;

// 초기 설정 //
void setup() {

  Serial.begin(9600);
  while (!Serial);
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
  Serial.println();

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

//   //  RF 초기화
//   if (!radio.begin()) {
//     Serial.println("Failled to initialze RF!");
//     while (1);
//   }
//   Serial.println("RF initialzed successfully.");
//   radio.openWritingPipe(address);
//   radio.setPALevel(RF24_PA_MAX);
//   radio.setDataRate(RF24_1MBPS);
//   radio.setChannel(76);

//   // RF 초기화 후 추가 설정 확인 (선택 사항)
//   if (!radio.isChipConnected()) {
//     Serial.println("nRF24L01+ 칩 연결 실패");
//     while (1);
//   }
//   Serial.println("nRF24L01+ 칩 연결 확인");
//   radio.stopListening();

//   radio.printDetails();
//   Serial.println(radio.getPALevel());
//   Serial.println(radio.getDataRate());
//   Serial.println(radio.getChannel());

// print an empty line
  Serial.println();
}

// 메인 루프 //
void loop() {

 unsigned long currentTime = millis(); // 현재 경과 시간

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
  
  // 스트레치와 호흡수 측정 //

  // 저항값 계산
  int sensorValue = analogRead(analogPin);
  float voltage = sensorValue * (3.3 / 1023.0); // 전압계산
  float knownResistor = 9810.0; // 기준 저항 9.81KOhms
  float resistor = knownResistor * (3.3 - voltage) / voltage; // 저항 계산
  // Serial.print("Resister = ");
  // Serial.println(resistor);

  // 스트레칭 모드 판단
  int stretching = 0; // 스트레칭 판단
  float deltaT = 1000.0;

  if (resistor > (lastResistor * 1.15)) { // 저항 증가 - 들숨
    stretching = 1;
    // Serial.print("lastResistor: "); Serial.println(lastResistor);
    // Serial.print("CurrentResistor: "); Serial.println(resistor);  
    // Serial.print("CurrentStretching: "); Serial.println(stretching); Serial.println();

    lastResistor = resistor;
  } 
  if (resistor < (lastResistor * 0.85)) {  // 저항 감소 - 날숨
    stretching = -1;
    // Serial.print("lastResistor: "); Serial.println(lastResistor);
    // Serial.print("CurrentResistor: "); Serial.println(resistor);
    // Serial.print("CurrentStretching: "); Serial.println(stretching); Serial.println();

    lastResistor = resistor;
  }

  // 들숨/날숨 전환 Count //
  if ((stretching * lastStretching) == -1) { // 스트레칭 모드 전환 판단
    
    stretchCount = stretchCount + 1;

    // Serial.print("lastStretching: "); Serial.println(lastStretching);
    Serial.print(" 들숨/날숨 Count"); Serial.print(stretchCount);
   
    if (stretchCount == 10) { // 5회 호흡
      currentTime = millis();
      deltaT = currentTime - lastStretchTime; // 10회 전환(5회 호홉)에 걸린 시간
      
      stretch = 5.0 * 60000.0 / deltaT; // 분당 호흡수 계산 
      Serial.print(" 호흡수: ");
      Serial.println(stretch);
      Serial.println();

      lastStretchTime = currentTime;
      stretchCount = 0; // reset timer
    }
    lastStretching = stretching;
  }
  
  // BLE 송신
  byte manufacturerData[20];
  memcpy(manufacturerData, &temperature, sizeof(temperature));
  memcpy(manufacturerData + 4, &humidity, sizeof(humidity));
  memcpy(manufacturerData + 8, &accel, sizeof(accel));
  memcpy(manufacturerData + 12, &angle, sizeof(angle));
  memcpy(manufacturerData + 16, &stretch, sizeof(stretch));
  
  // 제조자 데이터
  BLE.setManufacturerData(MANUFACTURER_ID, manufacturerData, sizeof(manufacturerData));
  
   // 광고 다시 시작
  BLE.advertise();  // 데이터 갱신 후 광고 다시 시작
  // Serial.println("Broadcasting new data.");
  // Serial.println(); 
  
  // // RF 송신
  // const char text[32] = "Hello!";
  // bool success = radio.write(&text, sizeof(text));
  // if (success) {
  //   Serial.println("RF transmitted successfully.");
  // } else {
  //   Serial.println("Failed to transnmit RF.");
  //   radio.printDetails();
  //   Serial.println(radio.getPALevel());
  //   Serial.println(radio.getDataRate());
  //   Serial.println(radio.getChannel());
  // }

  // wait to transmit again
  delay(10);
}