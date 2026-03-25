#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C // Changed to 0x3C (most common), use 0x3D if yours is different
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);

  // 1. CRITICAL: Initialize I2C with your specific pins FIRST
  Wire.begin(38, 39);

  // 2. Initialize the display
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed. Check wiring or Address!"));
    for(;;); 
  }

  // Clear the buffer and show splash
  display.display();
  delay(2000); 
  display.clearDisplay();

  // Write something to test
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("I2C Pins 38 & 39"));
  display.println(F("System Online..."));
  display.display(); 
}

void loop() {
  // Your code here
}
