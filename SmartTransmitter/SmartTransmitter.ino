/*
  스마트 안전관리 웨어러블 디바이스 온보드 펌웨어
  BLE BroadCasting Transmitter Ver 2.3
*/

#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <PDM.h>

/////////////// 디바이스 고유 이름 및 UUID 설정 ////////////////
const char *LocalName = "CONTROL01";
const char *SERVICE_UUID =
    "e9321e36-6211-11ee-8c99-0242ac120002"; // Dashboard 필터링용 고유 ID
///////////////

BLEService smartService(SERVICE_UUID);

// 임의의 제조사 ID
const uint16_t MANUFACTURER_ID = 0xFFFF;

// 센서 데이터 float
float temperature, humidity, pressure;
float accel, angle;
float noise = 0.0;

// 센서 데이터 int
short iDataType;
short iTemp, iHumidity, iPressure;
short iAccelX, iAccelY, iAccelZ;
short iAngleX, iAngleY, iAngleZ;
short iResistance; // Raw 저항값 (kOhm * 100)
short iNoise;

// IMU 변수
const float alpha = 0.9; // 자이로 의존도 (max=1.0)
unsigned long lastIMUTime = 0;
float dt; // 시간 간격(초)
float pitch, roll;

// BLE 광고 타이머
unsigned long lastAdvertiseTime = 0;
unsigned long lastIntDataTime = 0;
const unsigned long ADVERTISE_INTERVAL = 200; // 0.2초 간격 (초당 5회 샘플링)

// 스트레치 센서 변수
const int SENSOR_PIN = A0;
const float R1 = 10000.0; // 기준 저항 10kΩ

const float THRESHOLD = 0.3;      // 변화 감지를 위한 임계값 (kΩ)
const int MIN_INTERVAL = 1000;    // 최소 호흡 간격 (1초)
const int MAX_INTERVAL = 10000;   // 최대 호흡 간격 (10초)
const int BREATH_TIMEOUT = 15000; // 호흡 감지 타임아웃 시간 (15초)
const int WINDOW_SIZE = 3;        // 이동 평균 윈도우 크기

float filterBuffer[WINDOW_SIZE]; // 이동 평균 필터 버퍼
int filterIndex = 0;

float maxResistance;               // 현재 고점
float minResistance;               // 현재 저점
unsigned long lastExtremeTime = 0; // 마지막 극값 시간
bool findingMax = true;            // true: 고점 탐색 중, false: 저점 탐색 중
float lastValidResistance;         // 마지막으로 감지된 유효한 저항값

unsigned long lastBreathTime = 0;        // 마지막 호흡 시간
unsigned long lastExtremeDetectTime = 0; // 마지막 고점/저점 감지 시간
float currentBreathRate = 0;             // 현재 호흡수

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
  delay(3000); // 시리얼 포트가 준비될 때까지 잠시 대기
  Serial.println("Serial Started");

  // 온습도 초기화
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1)
      ;
  }
  Serial.println("Humidity and temperature sensor initialized successfully.");

  /* 기압 초기화 삭제
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
  Serial.println("Pressure sensor initialized successfully.");
  */

  // IMU 초기화
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  Serial.println("IMU sensor initialized successfully.");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  // 마이크(PDM) 초기화
  PDM.onReceive(onPDMdata); // 콜백 함수 설정

  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize microphone!");
    while (1)
      ;
  }
  Serial.println("PDM initialized successfully.");

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("BLE 초기화 실패!");
    while (1)
      ;
  }
  Serial.println("BLE initialized successfully.");

  BLE.setLocalName(LocalName);
  BLE.setAdvertisedService(smartService); // 광고에 Service UUID 포함
  BLE.addService(smartService);           // 서비스 등록

  if (!BLE.advertise()) {
    Serial.println("Failed to start BLE advertising.");
    while (1)
      ;
  }

  // 스트레치 초기화
  pinMode(SENSOR_PIN, INPUT);

  // 초기값 설정을 위해 여러 번 읽기
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    int rawValue = analogRead(SENSOR_PIN);
    sum += calculateResistance(rawValue);
    delay(10);
  }
  float initialResistance = sum / 10;

  // 버퍼 및 변수 초기화
  for (int i = 0; i < WINDOW_SIZE; i++) {
    filterBuffer[i] = initialResistance;
  }
  maxResistance = initialResistance;
  minResistance = initialResistance;
  lastValidResistance = initialResistance;
}

float calculateResistance(int rawValue) {
  if (rawValue < 20)
    return 999999.0; // 노이즈 필터링
  return R1 * (1023.0 / rawValue - 1.0) / 1000.0;
}

float movingAverage(float newValue) {
  filterBuffer[filterIndex] = newValue;
  filterIndex = (filterIndex + 1) % WINDOW_SIZE;

  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum += filterBuffer[i];
  }
  return sum / WINDOW_SIZE;
}

// 호흡수 계산 및 업데이트
void checkBreathTimeout(unsigned long currentTime) {
  if (lastExtremeDetectTime > 0 &&
      (currentTime - lastExtremeDetectTime) > BREATH_TIMEOUT) {
    currentBreathRate = 0;
    lastBreathTime = 0; // 호흡 시간도 리셋
    Serial.println("호흡 감지 타임아웃 - 호흡수 리셋");
  }
}

void updateBreathRate(unsigned long currentTime) {
  if (lastBreathTime == 0) {
    lastBreathTime = currentTime;
    return;
  }

  unsigned long interval = currentTime - lastBreathTime;
  if (interval >= MIN_INTERVAL && interval <= MAX_INTERVAL) {
    currentBreathRate = 60000.0 / interval;
    lastBreathTime = currentTime;

    Serial.print("호흡 간격: ");
    Serial.print(interval);
    Serial.print("ms, 호흡수: ");
    Serial.print(currentBreathRate);
    Serial.println(" 회/분");
  }
}

// 메인 루틴
void loop() {

  iDataType = 2; // 2byte형 데이터

  unsigned long currentTime = millis(); // 현재 시간

  temperature = HS300x.readTemperature(); // 온도
  humidity = HS300x.readHumidity();       // 습도
  pressure = 0.0;                         // 기압 사용 안함

  // 센서 데이터를 int로 변환 (소수점 1자리까지)
  iTemp = int(temperature * 10);
  iHumidity = int(humidity * 10);
  iPressure = int(pressure * 10);

  // 마이크 소음(dB) 측정
  if (samplesRead) {
    long sumSquares = 0;

    // RMS 계산
    for (int i = 0; i < samplesRead; i++) {
      sumSquares += (long)sampleBuffer[i] * sampleBuffer[i];
    }

    float rms = sqrt(sumSquares / (float)samplesRead);
    noise = 20.0 * log10(rms);
    if (isnan(noise) || noise < 0)
      noise = 0.0;

    iNoise = int(noise * 10); // 소수점 1자리까지 측정

    samplesRead = 0; // 버퍼 초기화
  }

  // IMU 가속도, 각도 측정 //
  dt = (currentTime - lastIMUTime) / 1000.0; // ms
  lastIMUTime = currentTime;

  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az); // 가속도
    IMU.readGyroscope(gx, gy, gz);    // 자이로

    // 가속도계로 Pitch와 Roll 계산
    float accelPitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float accelRoll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

    // 자이로스코프 각속도를 각도로 변환
    float gyroPitch = pitch + gx * dt;
    float gyroRoll = roll + gy * dt;

    // 보완 필터 적용
    pitch = alpha * gyroPitch + (1 - alpha) * accelPitch;
    roll = alpha * gyroRoll + (1 - alpha) * accelRoll;

    accel = ax * ax + ay * ay + az * az; // 가속도 절대값

    iAccelX = int(ax * 10); // X축 가속도
    iAccelY = int(ay * 10); // Y축 가속도
    iAccelZ = int(az * 10); // Z축 가속도

    angle = pitch; // 보드 세로 각도

    iAngleX = int(pitch * 10);     // 보드 세로 각도
    iAngleY = int(roll * 10);      // 보드 가로 각도
    iAngleZ = int(gyroPitch * 10); // 자이로 각도
  }

  ///////////////////////////////// 호흡수 루틴
  // 호흡 타임아웃 체크
  checkBreathTimeout(currentTime);

  // 센서 값 읽기 및 저항 계산
  int rawValue = analogRead(SENSOR_PIN);
  float resistance = calculateResistance(rawValue);
  float filteredResistance = movingAverage(resistance);

  if (findingMax) {
    // 고점 탐색 중
    if (filteredResistance > maxResistance) {
      maxResistance = filteredResistance;
    }
    // 고점 감지 조건: 현재값이 최대값보다 THRESHOLD 이상 감소
    else if (filteredResistance < maxResistance - THRESHOLD) {
      // Serial.println("------------------------");
      // Serial.print("고점 감지! 저항: ");
      // Serial.print(maxResistance);
      // Serial.println(" kΩ");

      updateBreathRate(currentTime);
      lastExtremeDetectTime = currentTime; // 고점 감지 시간 업데이트
      Serial.println("------------------------");

      // 상태 전환
      findingMax = false;
      minResistance = filteredResistance; // 새로운 저점 탐색 시작
      lastValidResistance = maxResistance;
    }
  } else {
    // 저점 탐색 중
    if (filteredResistance < minResistance) {
      minResistance = filteredResistance;
    }
    // 저점 감지 조건: 현재값이 최소값보다 THRESHOLD 이상 증가
    else if (filteredResistance > minResistance + THRESHOLD) {
      // Serial.println("------------------------");
      // Serial.print("저점 감지! 저항: ");
      // Serial.print(minResistance);
      // Serial.println(" kΩ");

      updateBreathRate(currentTime);
      lastExtremeDetectTime = currentTime; // 저점 감지 시간 업데이트
      Serial.println("------------------------");

      // 상태 전환
      findingMax = true;
      maxResistance = filteredResistance; // 새로운 고점 탐색 시작
      lastValidResistance = minResistance;
    }
  }

  iResistance =
      int(filteredResistance *
          100); // AI 분석을 위해 저항값 소수점 2자리까지 보존 (kOhm * 100)

  ///////////////////////////////// 호흡수 루틴

  // 설정된 간격(200ms)으로 int 센서 데이터 BLE 광고 전송
  if (currentTime - lastIntDataTime >= ADVERTISE_INTERVAL) {
    memcpy(manufacturerData, &iDataType, sizeof(iDataType));
    memcpy(manufacturerData + 2, &iTemp, sizeof(iTemp));
    memcpy(manufacturerData + 4, &iHumidity, sizeof(iHumidity));
    memcpy(manufacturerData + 6, &iPressure, sizeof(iPressure));
    memcpy(manufacturerData + 8, &iAccelX, sizeof(iAccelX));
    memcpy(manufacturerData + 10, &iAccelY, sizeof(iAccelY));
    memcpy(manufacturerData + 12, &iAccelZ, sizeof(iAccelZ));
    memcpy(manufacturerData + 14, &iAngleX, sizeof(iAngleX));
    memcpy(manufacturerData + 16, &iAngleY, sizeof(iAngleY));
    memcpy(manufacturerData + 18, &iAngleZ, sizeof(iAngleZ));
    memcpy(manufacturerData + 20, &iResistance, sizeof(iResistance));
    memcpy(manufacturerData + 22, &iNoise, sizeof(iNoise));
    BLE.setManufacturerData(MANUFACTURER_ID, manufacturerData,
                            sizeof(manufacturerData));
    BLE.advertise();

    // 전송 주기가 빠르므로 시리얼 출력은 생략하거나 확인용으로만 사용
    // Serial.print("Res: "); Serial.println(iResistance);

    // 센서값 확인 출력
    Serial.print("온도: ");
    Serial.print(iTemp);
    Serial.print(", 습도: ");
    Serial.print(iHumidity);
    Serial.print(", 기압: ");
    Serial.print(iPressure);
    Serial.print(", 가속(X,Y,Z): ");
    Serial.print(iAccelX);
    Serial.print(",");
    Serial.print(iAccelY);
    Serial.print(",");
    Serial.print(iAccelZ);
    Serial.print(", 각도(X,Y,Z): ");
    Serial.print(iAngleX);
    Serial.print(",");
    Serial.print(iAngleY);
    Serial.print(",");
    Serial.print(iAngleZ);
    Serial.print(", 호흡: ");
    Serial.print(iBreathRate);
    Serial.print(", 소음: ");
    Serial.println(iNoise);

    lastIntDataTime = currentTime; // 마지막 전송 시간 업데이트
  }

  /* 1초 간격으로 float 센서 데이터 BLE 광고 전송
  if (currentTime - lastAdvertiseTime >= ADVERTISE_INTERVAL) {
    // BLE Broadcasting
    memcpy(manufacturerData, &temperature, sizeof(temperature));
    memcpy(manufacturerData + 4, &humidity, sizeof(humidity));
    memcpy(manufacturerData + 8, &accel, sizeof(accel));
    memcpy(manufacturerData + 12, &angle, sizeof(angle));
    memcpy(manufacturerData + 16, &currentBreathRate,
  sizeof(currentBreathRate)); memcpy(manufacturerData + 20, &noise,
  sizeof(noise)); BLE.setManufacturerData(MANUFACTURER_ID, manufacturerData,
  sizeof(manufacturerData)); BLE.advertise();

    // 센서값 출력
    Serial.print("온도: "); Serial.print(temperature);
    Serial.print(", 습도: "); Serial.print(humidity);
    Serial.print(", 가속: "); Serial.print(accel);
    Serial.print(", 각도: "); Serial.print(angle);
    Serial.print(", 호흡: "); Serial.print(currentBreathRate);
    Serial.print(", 소음: "); Serial.println(noise);

    lastAdvertiseTime = currentTime;  // 마지막 전송 시간 업데이트
  }
  */
}
