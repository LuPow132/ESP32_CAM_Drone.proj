#include <Wire.h>
#include <Adafruit_BMP280.h>

// Define your custom I2C pins for ESP32-S3
#define I2C_SDA 38
#define I2C_SCL 39

Adafruit_BMP280 bmp; // I2C interface

void setup() {
  Serial.begin(115200); // S3 usually uses 115200
  while ( !Serial ) delay(100); 
  
  Serial.println(F("BMP280 test with custom I2C pins"));

  // 1. Initialize Wire with your specific pins first
  Wire.begin(I2C_SDA, I2C_SCL);

  // 2. Pass the Wire object to the bmp.begin() function
  // Note: 0x77 is the default address. If it fails, try 0x76.
  unsigned status = bmp.begin(0x77); 
  
  if (!status) {
    // If 0x77 failed, try the alternative address 0x76
    status = bmp.begin(0x76);
  }

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), HEX);
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
}

void loop() {
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.println();
    delay(2000);
}
