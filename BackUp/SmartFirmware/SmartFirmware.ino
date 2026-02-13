// Smart Firmware 1.0
// This code reads all sensor values from the BLE Senser board
// including temperature, humidity, pressure, and IMU data
// transmits via Bluetooth Low Energy (BLE)

#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>      // HS300x library
#include <Arduino_LPS22HB.h>     // LPS22HB library
#include <Arduino_LSM9DS1.h>     // BMI270_BMM150 library

BLEService sensorService("180A");  // Set up BLE service
BLEFloatCharacteristic temperatureChar("2A6E", BLERead | BLENotify);
BLEFloatCharacteristic humidityChar("2A6F", BLERead | BLENotify);
BLEFloatCharacteristic pressureChar("2A6D", BLERead | BLENotify);
BLECharacteristic imuChar("2A76", BLERead | BLENotify, 12); // IMU data as three floats (12 bytes)

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("Nano33BLE");   // Set device name
  BLE.setAdvertisedService(sensorService);

  // Add BLE characteristics
  sensorService.addCharacteristic(temperatureChar);
  sensorService.addCharacteristic(humidityChar);
  sensorService.addCharacteristic(pressureChar);
  sensorService.addCharacteristic(imuChar);
  BLE.addService(sensorService);

  BLE.advertise();
  Serial.println("BLE device advertising started");

  // Initialize sensors
  if (!HTS.begin()) Serial.println("Temperature/Humidity sensor initialization failed!");
  if (!BARO.begin()) Serial.println("Pressure sensor initialization failed!");
  if (!IMU.begin()) Serial.println("IMU sensor initialization failed!");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to device: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Temperature and humidity
      float temperature = HTS.readTemperature();
      float humidity = HTS.readHumidity();
      temperatureChar.writeValue(temperature);
      humidityChar.writeValue(humidity);

      // Pressure
      float pressure = BARO.readPressure();
      pressureChar.writeValue(pressure);

      // IMU data (acceleration x, y, z)
      float x, y, z;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);

        // Send IMU data as an array of three floats
        uint8_t imuData[12];
        memcpy(imuData, &x, 4);
        memcpy(imuData + 4, &y, 4);
        memcpy(imuData + 8, &z, 4);
        imuChar.writeValue(imuData, 12);
      }

      // Data transmission interval
      delay(500); // Set transmission interval
    }

    Serial.print("Disconnected from device: ");
    Serial.println(central.address());
  }
}
