#include <PDM.h>

void onPDMdata() {
  Serial.println(">> onPDMdata called!");
  int bytesAvailable = PDM.available();
  if (bytesAvailable > 0) {
    Serial.print("Bytes: "); Serial.println(bytesAvailable);
    int16_t buffer[256];
    PDM.read(buffer, bytesAvailable);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("PDM init fail");
    while (1);
  }

  Serial.println("PDM init success");
}

void loop() {
  delay(1000);
  Serial.println("loop...");
}