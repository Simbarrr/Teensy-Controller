#include <Arduino.h>
#include <Adafruit_BNO08x.h> 

Adafruit_BNO08x BNO(-1);          // Create BNO object. -1 means no reset pin is used
sh2_SensorValue_t sensorValue;    // Struct that will hold incoming sensor data

// put function declarations here:


void setup() {
    Serial.begin(115200);           // Start serial communication
    delay(500);                     // Small delay to allow serial monitor to open


  // Try to initialize the BNO over I2C
  if (!BNO.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }      // Freeze program if chip not found
  }

  Serial.println("BNO found");
}

void loop() {
Serial.println("I am ALIVE");
delay(100000);
}

// put function definitions here:
