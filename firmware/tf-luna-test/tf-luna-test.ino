#include "Wire.h"
#include "TFLI2C.h"

TFLI2C sensor;
uint8_t addr = 0x10;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);

  Serial.println("=== TF-Luna Heavy Reset & Test ===");
  Serial.println();

  // Scan I2C bus
  Serial.println("Step 1: Scanning I2C bus...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      Serial.println(a, HEX);
      found++;
    }
  }
  Serial.print("  ");
  Serial.print(found);
  Serial.println(" devices found");
  Serial.println();

  // Hard reset to factory defaults
  Serial.println("Step 2: Hard reset to factory defaults...");
  Serial.print("  Result: ");
  Serial.println(sensor.Hard_Reset(addr) ? "OK" : "FAIL");
  Serial.println("  Waiting 5 seconds for reboot...");
  delay(5000);

  // Check sensor is back
  Wire.beginTransmission(addr);
  Serial.print("  Sensor after reset: ");
  Serial.println(Wire.endTransmission() == 0 ? "FOUND" : "NOT FOUND");
  Serial.println();

  // Soft reset
  Serial.println("Step 3: Soft reset...");
  Serial.print("  Result: ");
  Serial.println(sensor.Soft_Reset(addr) ? "OK" : "FAIL");
  delay(2000);
  Serial.println();

  // Enable sensor
  Serial.println("Step 4: Enable sensor...");
  Serial.print("  Result: ");
  Serial.println(sensor.Set_Enable(addr) ? "OK" : "FAIL");
  delay(100);

  // Set continuous mode
  Serial.println("Step 5: Set continuous mode...");
  Serial.print("  Result: ");
  Serial.println(sensor.Set_Cont_Mode(addr) ? "OK" : "FAIL");
  delay(100);

  // Set frame rate 100Hz
  Serial.println("Step 6: Set frame rate 100Hz...");
  uint16_t fps = 100;
  Serial.print("  Result: ");
  Serial.println(sensor.Set_Frame_Rate(fps, addr) ? "OK" : "FAIL");
  delay(100);

  // Save settings
  Serial.println("Step 7: Save settings...");
  Serial.print("  Result: ");
  Serial.println(sensor.Save_Settings(addr) ? "OK" : "FAIL");
  delay(500);

  // Read firmware version
  Serial.println("Step 8: Read firmware version...");
  uint8_t ver[3];
  if (sensor.Get_Firmware_Version(ver, addr)) {
    Serial.print("  Firmware: ");
    Serial.print(ver[2]);
    Serial.print(".");
    Serial.print(ver[1]);
    Serial.print(".");
    Serial.println(ver[0]);
  } else {
    Serial.println("  FAILED to read firmware");
  }

  Serial.println();
  Serial.println("Step 9: Waiting 3 seconds then reading continuously...");
  delay(3000);
  Serial.println();
  Serial.println("dist(cm) | flux  | temp");
  Serial.println("---------|-------|-----");
}

void loop() {
  int16_t dist, flux, temp;
  if (sensor.getData(dist, flux, temp, addr)) {
    Serial.print(dist);
    Serial.print(" cm\t | ");
    Serial.print(flux);
    Serial.print("\t | ");
    Serial.println(temp);
  } else {
    sensor.printDataArray();
  }
  delay(200);
}
