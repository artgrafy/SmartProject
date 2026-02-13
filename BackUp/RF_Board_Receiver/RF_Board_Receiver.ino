/*
  안전관리 웨어러블 디바이스 - Receiver

  - Arduino Nano 33 BLE R2
  
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

// 센서 특성 데이터
struct SensorData {
  int deviceNo; // 송신기 번호
  int dataNo; // 패킷 번호
  float temperature, humidity; // 환경 센서
  float accel, angle, stretch; // 동작 센서
} data;

// RF 설정
RF24 radio(7, 8); // CE, CSN 핀 
const byte address[6] = "00002"; // 송신 주소

// // BLE UUID, 특성 정의
// BLEService sensorService("181A");  // 환경 센시 표준 서비스 UUID
// BLECharacteristic sensorCharacteristic("12345678-1234-5678-1234-56789abcdef0"
// , BLERead | BLENotify, sizeof(SensorData));

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial Started");

 // RF 초기화
  if (!radio.begin()) {
    Serial.println("Failled to initialze RF!");
    while (1);
  } else {
    Serial.println("RF initialzed successfully.");
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);

  // RF 초기화 후 추가 설정 확인 (선택 사항)
  if (radio.isChipConnected()) {
    Serial.println("nRF24L01+ 칩 연결 확인");
  } else {
    Serial.println("nRF24L01+ 칩 연결 실패");
  }
  
  radio.startListening();

  radio.printDetails();    
  Serial.println(radio.getPALevel());
  Serial.println(radio.getDataRate());
  Serial.println(radio.getChannel());

  // // BLE 초기화
  // if (!BLE.begin()) {
  //   Serial.println("BLE 초기화 실패!");
  //   while (1);
  // }  else {
  //   Serial.println("BLE initialized successfully.");
  // }
  // BLE.setLocalName("Nano33BLE_01 ");  // 블루투스 기기 이름 설정
  // BLE.setAdvertisedService(sensorService);  // 서비스 UUID 설정
  // sensorService.addCharacteristic(sensorCharacteristic);  // 특성 추가
  // BLE.addService(sensorService); // 서비스 등록

  // String macAddress = BLE.address(); // MAC 주소
  // Serial.print("BLE MAC 주소: ");
  // Serial.println(macAddress);

  // BLE.advertise();  // 블루투스 광고 시작
  // Serial.println("BLE advertising.");

  // print an empty line
  Serial.println();
}

void loop() {

  if (radio.available()) {
    
    // RF 수신
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println("RF transmitted successfullly.");
    Serial.println(text);
  } else {
    // Serial.print("No RF. ");
  }

  //radio.read(&data, sizeof(SensorData));

  // 수신 데이터 출력
  // Serial.print("Device No.: ");
  // Serial.println(data.deviceNo);

  // Serial.print("Data No.: ");
  // Serial.println(data.dataNo);
  
  // Serial.print("Temperature = ");
  // Serial.print(data.temperature);
  // Serial.println(" °C");

  // Serial.print("Humidity    = ");
  // Serial.print(data.humidity);
  // Serial.println(" %");

  // Serial.print("Acceleration = ");
  // Serial.println(data.accel);
  
  // Serial.print("Stretch = ");
  // Serial.println(data.stretch);

  // print an empty line

  // // BLE 송신
  // BLEDevice central = BLE.central(); // 블루투스 기기 탐색
  // if (central) {
  //   Serial.print("BLE 연결: ");
  //   Serial.println(central.address());

  //   if (central.connected()) {
  //     sensorCharacteristic.writeValue((byte*)&data, sizeof(SensorData));
  //   } 
  // } else {
  //     Serial.println("BLE 해제");
  //     BLE.advertise();
  // }

  // Serial.println();
}
