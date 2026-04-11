/* #include <Arduino.h>              // Core Arduino functions
#include <Adafruit_BNO08x.h>     // Library for the BNO08x IMU
#include <PWMServo.h>             // Library for PWM     Servo functions

Adafruit_BNO08x BNO(-1);          // Create BNO object. -1 means no reset pin is used
sh2_SensorValue_t sensorValue;    // Struct that will hold incoming sensor data


// Quaternion state variables (current orientation estimate)
float q0 = 1.0;   // w (scalar part) - start at identity rotation
float q1 = 0.0;   // x
float q2 = 0.0;   // y
float q3 = 0.0;   // z
float kq2 = 10/4;
float kq3 = 10/4;
float kwy = 3.4319/4;
float kwz = 3.4319/4;
unsigned long lastTime = 0;
float roll = 0;
float yaw = 0;
float pitch = 0;


PWMServo pitchservo;
PWMServo yawservo;


void setup(){
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
    pitchservo.attach(1);
    yawservo.attach(2);
    Serial.println("finish setup");
    pitchservo.write(86);
    yawservo.write(86);
    delay(1000);
    lastTime = micros();

}

void loop() {

//integrate to new position and get new rates
    if (BNO.getSensorEvent(&sensorValue)) {
    // Make sure the event is calibrated gyro data
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Extract angular velocity components (rad/s)
        float wx = -sensorValue.un.gyroscope.x;
        float wy = -sensorValue.un.gyroscope.y;
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

//Apply k matrix to get control angles for servo and write.
        float gopitch = min(max(57.29578*(-kq2*q2 - kwy*wy),-20),20);
        Serial.println(gopitch+45);
        float goyaw = min(max(57.29578*(-kq3*q3 - kwz*wz),-20),20);
        Serial.println(goyaw+90);
        pitchservo.write(gopitch+90);
        yawservo.write(goyaw+90);
    }
}

}
 */