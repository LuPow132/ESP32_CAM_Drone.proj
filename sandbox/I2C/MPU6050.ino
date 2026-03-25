#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

void setup() {
  Serial.begin(115200);
  Wire.begin(38, 39); 

  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  // Optional: Print header labels for the Plotter
  // Note: Some versions of Arduino IDE require these labels to be the first thing printed.
  Serial.println("AccX(g) AccY(g) AccZ(g) GyroX(deg/s) GyroY(deg/s) GyroZ(deg/s)");
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, true);

  // Read Raw Data
  int16_t rawAx = Wire.read() << 8 | Wire.read();
  int16_t rawAy = Wire.read() << 8 | Wire.read();
  int16_t rawAz = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature bytes for the plotter
  int16_t rawGx = Wire.read() << 8 | Wire.read();
  int16_t rawGy = Wire.read() << 8 | Wire.read();
  int16_t rawGz = Wire.read() << 8 | Wire.read();

  // Convert to Real Units
  // Default sensitivity is +/- 2g (16384 LSB/g) and +/- 250 deg/s (131 LSB/deg/s)
  float ax = (float)rawAx / 16384.0;
  float ay = (float)rawAy / 16384.0;
  float az = (float)rawAz / 16384.0;
  
  float gx = (float)rawGx / 131.0;
  float gy = (float)rawGy / 131.0;
  float gz = (float)rawGz / 131.0;

  // Output format for Serial Plotter: "Value1 Value2 Value3..."
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.println(gz); // println signals the end of the data packet

  delay(20); // Faster updates (50Hz) make for a smoother graph
}
