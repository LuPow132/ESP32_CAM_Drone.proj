#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// Create the sensor object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // 1. Initialize custom I2C pins: SDA = 38, SCL = 39
  // This must happen BEFORE lox.begin()
  Wire.begin(38, 39);

  // Wait for serial port to connect (needed for native USB boards)
  while (!Serial) {
    delay(1);
  }
  
  Serial.println(F("Adafruit VL53L0X Custom I2C Test"));

  // 2. Initialize the sensor using the Wire object we just configured
  // Parameters: (Address, DebugMode, Pointer to Wire)
  if (!lox.begin(VL53L0X_I2C_ADDR, false, &Wire)) {
    Serial.println(F("Failed to boot VL53L0X. Check your wiring and pins 38/39!"));
    while(1);
  }
  
  Serial.println(F("VL53L0X Ready!\n")); 
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  
  // Get the ranging data
  lox.rangingTest(&measure, false); 

  // Phase failures (Status 4) usually mean the signal was too weak or out of range
  if (measure.RangeStatus != 4) {  
    Serial.print("Distance (mm): "); 
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" Out of range ");
  }
    
  delay(100);
}
