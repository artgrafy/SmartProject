#include <ArduinoBLE.h>

// 여러 로컬 네임 필터 설정
const char* targetLocalNames[] = {"SENSE01", "SENSE02", "SENSE03"}; // 필터링할 로컬 이름들
const int numOfTargets = sizeof(targetLocalNames) / sizeof(targetLocalNames[0]);

unsigned long scanStartTime = 0; // 스캔이 시작된 시간
const unsigned long scanDuration = 500;  // 스캔 시간

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial Started");

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("BLE 초기화 실패!");
    while (1);
  }
  Serial.println("BLE initialized successfully.");

  // BLE 스캔 시작
  BLE.scan();  // 모든 BLE 장치 스캔
  Serial.println("BLE scanning for devices...");
  scanStartTime = millis();  // 스캔 시작 시간 기록
}

void loop() {
  // 스캔 시간이 지났는지 확인 (5초 동안 스캔 후 중지)
  if (millis() - scanStartTime > scanDuration) {
    BLE.stopScan();  // 스캔 중지
    Serial.println("BLE scan stopped.");
    delay(100);  // 잠시 대기
    BLE.scan();  // 다시 스캔 시작
    Serial.println("BLE scan restarted.");
    scanStartTime = millis();  // 새로운 스캔 시작 시간 기록
  }

  // 광고 패킷 수신 확인
  BLEDevice device = BLE.available();
  if (device) {
    // 수신된 디바이스의 로컬 이름 확인
    const char* localName = device.localName().c_str();

    // 수신된 로컬 이름이 targetLocalNames 배열에 있는지 확인
    bool found = false;
    for (int i = 0; i < numOfTargets; i++) {
      if (strcmp(localName, targetLocalNames[i]) == 0) {
        found = true;
        break;
      }
    }

    if (found) {
      Serial.print("Target device found: ");
      Serial.println(localName);
      Serial.print("Device address: ");
      Serial.println(device.address());
      Serial.print("Device RSSI: ");
      Serial.println(device.rssi());

      // 제조자 데이터 수신
      int dataLength = device.manufacturerDataLength();
      Serial.print("Received manufacturer data length: ");
      Serial.println(dataLength);

      if (dataLength == 22) { // 제조자 데이터가 예상한 22바이트인지 확인
        byte manufacturerData[dataLength];
        device.manufacturerData(manufacturerData, dataLength);

        // 데이터를 float 배열로 변환
        float receivedTemperature, receivedHumidity, receivedAccel, receivedAngle, receivedStretch;
        memcpy(&receivedTemperature, manufacturerData + 2, sizeof(receivedTemperature));
        memcpy(&receivedHumidity, manufacturerData + 6, sizeof(receivedHumidity));
        memcpy(&receivedAccel, manufacturerData + 10, sizeof(receivedAccel));
        memcpy(&receivedAngle, manufacturerData + 14, sizeof(receivedAngle));
        memcpy(&receivedStretch, manufacturerData + 18, sizeof(receivedStretch));

        // 수신된 데이터 출력
        Serial.print("Temperature: ");
        Serial.print(receivedTemperature);
        Serial.println(" °C");

        Serial.print("Humidity: ");
        Serial.print(receivedHumidity);
        Serial.println(" %");

        Serial.print("Acceleration: ");
        Serial.println(receivedAccel);

        Serial.print("Angle: ");
        Serial.println(receivedAngle);

        Serial.print("Stretch: ");
        Serial.println(receivedStretch);

        Serial.println();
      } else {
        Serial.println("Unexpected manufacturer data length or no manufacturer data received.");
      }
    }
  }
}