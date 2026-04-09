#include <Arduino.h>              // Core Arduino functions
#include <Adafruit_BNO08x.h>      // Library for the BNO08x IMU

Adafruit_BNO08x BNO(-1);          // Create BNO object. -1 means no reset pin is used
sh2_SensorValue_t sensorValue;    // Struct that will hold incoming sensor data

// Quaternion state variables (current orientation estimate)
float q0 = 1.0;   // w (scalar part) - start at identity rotation
float q1 = 0.0;   // x
float q2 = 0.0;   // y
float q3 = 0.0;   // z
float roll = 0.0;
float yaw = 0.0;
float pitch = 0.0;
int StateMchn = 0;
float biasx = 0;
float biasy = 0;
float biasz = 0;
int calibrationsteps = 100;

unsigned long lastTime = 0;       // Stores previous time for dt calculation

void setup() {
  Serial.begin(115200);           // Start serial communication
  delay(500);                     // Small delay to allow serial monitor to open

  // Try to initialize the BNO over I2C
  if (!BNO.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }      // Freeze program if chip not found
  }

  Serial.println("BNO found");

  // Enable calibrated gyroscope report
  // This tells the chip to send angular velocity data (rad/s)
  BNO.enableReport(SH2_GYROSCOPE_CALIBRATED);
  BNO.enableReport(SH2_ACCELEROMETER);
  // Initialize time reference for integration
  lastTime = micros();
   Serial.println("Setup Complete");
}

void loop() {

  switch(StateMchn) {
    case 0: {
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "update") {
        Serial.println("Switching to State 1");
        StateMchn = 1;
      }
    }
    break;
    }

    case 1: {
    int z = 0;
    float ratesumx = 0;
    float ratesumy = 0;
    float ratesumz = 0;
    while(z < calibrationsteps){
      // Check if a new sensor event has arrived
      if (BNO.getSensorEvent(&sensorValue)) {
        // Make sure the event is calibrated gyro data
        if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
          // Extract angular velocity components (rad/s)
          float wx = sensorValue.un.gyroscope.x;
          float wy = sensorValue.un.gyroscope.y;
          float wz = sensorValue.un.gyroscope.z;
          Serial.print("Roll Rate ");
          Serial.print(wx);
          Serial.print("  Pitch Rate ");
          Serial.print(wy);
          Serial.print("  Yaw Rate ");
          Serial.println(wz);
          ratesumx += wx;
          ratesumy += wy;
          ratesumz += wz;
          z += 1;
        }
      }
    }

    biasx = ratesumx/calibrationsteps; // all in radians
    biasy = ratesumy/calibrationsteps;
    biasz = ratesumz/calibrationsteps;
    Serial.print("Biasx: ");
    Serial.print(biasx);
    Serial.print("  Biasy: ");
    Serial.print(biasy);
    Serial.print("  Biasz: ");
    Serial.println(biasz);

    Serial.println("Switching to State 2");
    //delay(500);
    StateMchn = 2;
    break;
    }

    case 2: {
    //Check for acceleromter data in y > 9.81m/s
      if (BNO.getSensorEvent(&sensorValue)) {
        // Make sure the event is calibrated gyro data
        if (sensorValue.sensorId == SH2_ACCELEROMETER) {
          if (sensorValue.un.accelerometer.y > 13){
            Serial.println("Switching to State 3");
            //delay(500);
            lastTime = micros();
            StateMchn = 3;
          }
        }
      }
    break;
    }

    case 3: {

    if (BNO.getSensorEvent(&sensorValue)) {
    // Make sure the event is calibrated gyro data
      if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Extract angular velocity components (rad/s)
        float wx = sensorValue.un.gyroscope.x;
        float wy = sensorValue.un.gyroscope.y;
        float wz = sensorValue.un.gyroscope.z;

        // Compute time step (seconds)
        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;

        // -------------------------------
        // Quaternion differential equation
        // q_dot = 0.5 * q ⊗ omega
        // -------------------------------

        float dq0 = 0.5 * (-q1*wx - q2*wy - q3*wz);
        float dq1 = 0.5 * ( q0*wx + q2*wz - q3*wy);
        float dq2 = 0.5 * ( q0*wy - q1*wz + q3*wx);
        float dq3 = 0.5 * ( q0*wz + q1*wy - q2*wx);

        // Integrate quaternion using Euler integration
        q0 += dq0 * dt;
        q1 += dq1 * dt;
        q2 += dq2 * dt;
        q3 += dq3 * dt;

        // Normalize quaternion to prevent numerical drift
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;

        // ---------------------------------
        // Convert quaternion to Euler angles
        // ---------------------------------

        roll = 57.29578 * atan2(2*(q0*q1 + q2*q3),
                                      1 - 2*(q1*q1 + q2*q2));
        pitch = 57.29578 * asin(2*(q0*q2 - q3*q1));
        yaw = 57.29578 * atan2(2*(q0*q3 + q1*q2),
                                    1 - 2*(q2*q2 + q3*q3));
      
        // Print Euler angles in degrees
        Serial.print("Roll: ");
        Serial.print(roll);
        Serial.print("  Pitch: ");
        Serial.print(pitch);
        Serial.print("  Yaw: ");
        Serial.println(yaw);
        if(abs(roll) > 90 || abs(yaw) > 90 || abs(pitch) > 90) {
          StateMchn = 5;
        }
      }

    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
    if (sensorValue.un.accelerometer.z == 0) {
      StateMchn = 4; 
      }
    }
    }
    break;
    }


    case 4: {
    Serial.print("unpowered ascent");
    if (BNO.getSensorEvent(&sensorValue)) {
    // Make sure the event is calibrated gyro data
      if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Extract angular velocity components (rad/s)
        float wx = sensorValue.un.gyroscope.x;
        float wy = sensorValue.un.gyroscope.y;
        float wz = sensorValue.un.gyroscope.z;

        // Compute time step (seconds)
        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;

        // -------------------------------
        // Quaternion differential equation
        // q_dot = 0.5 * q ⊗ omega
        // -------------------------------

        float dq0 = 0.5 * (-q1*wx - q2*wy - q3*wz);
        float dq1 = 0.5 * ( q0*wx + q2*wz - q3*wy);
        float dq2 = 0.5 * ( q0*wy - q1*wz + q3*wx);
        float dq3 = 0.5 * ( q0*wz + q1*wy - q2*wx);

        // Integrate quaternion using Euler integration
        q0 += dq0 * dt;
        q1 += dq1 * dt;
        q2 += dq2 * dt;
        q3 += dq3 * dt;

        // Normalize quaternion to prevent numerical drift
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;

        // ---------------------------------
        // Convert quaternion to Euler angles
        // ---------------------------------

        roll = 57.29578 * atan2(2*(q0*q1 + q2*q3),
                                      1 - 2*(q1*q1 + q2*q2));
        pitch = 57.29578 * asin(2*(q0*q2 - q3*q1));
        yaw = 57.29578 * atan2(2*(q0*q3 + q1*q2),
                                    1 - 2*(q2*q2 + q3*q3));
      
        // Print Euler angles in degrees
        Serial.print("Roll: ");
        Serial.print(roll);
        Serial.print("  Pitch: ");
        Serial.print(pitch);
        Serial.print("  Yaw: ");
        Serial.println(yaw);
        }
    }
    break;
    }

    case 5: {
      Serial.print("LOCK OUT STRAIGHT AND STOP CONTROLLING");
      Serial.print("DEPLOY PARACHUTE");
      delay(500);
    }
  }
}