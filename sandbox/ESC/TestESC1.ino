const int ledPin = 4;
const int freq = 50;
const int resolution = 10; 

// These are your "Magic Numbers" for this specific ESC
const int armSignal = 40;   // The "Zero/Stop" point (-121 offset)
const int minSpin = 65;     // The lowest signal that makes it spin
const int maxSpin = 250;    // A safe upper limit for this ESC

void setup() {
  Serial.begin(115200);
  ledcAttach(ledPin, freq, resolution);

  Serial.println("--- ESC Manual Control Mode ---");
  Serial.println("STEP 1: Type '0' to ARM the motor.");
  Serial.println("STEP 2: Once armed, type '1' to '255' for speed.");
}

void loop() {
  if (Serial.available() > 0) {
    int inputVal = Serial.parseInt();
    
    // Clear the buffer
    while(Serial.available() > 0) Serial.read();

    if (inputVal == 0) {
      // STOP / ARM
      Serial.println("Command: STOP/ARM (Sending 40)");
      ledcWrite(ledPin, armSignal);
    } 
    else {
      // MAP 1-255 to the ESC's usable range (65-250)
      // This ensures even a '1' gives enough juice to maybe spin
      int mappedValue = map(inputVal, 1, 255, minSpin, maxSpin);
      
      Serial.print("User Input: "); Serial.print(inputVal);
      Serial.print(" -> Sending to ESC: "); Serial.println(mappedValue);
      
      ledcWrite(ledPin, mappedValue);
    }
  }
}
