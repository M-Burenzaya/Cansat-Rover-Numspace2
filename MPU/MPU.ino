#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

unsigned long previousTime = 0, lastPrintTime = 0;
float yaw = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float angleKp = 1.0, angleKi = 0.0, angleKd = 0.1;

float angleIntegral = 0, previousAngleError = 0;

void controlAngle(float desiredAngle) {
    static float yaw = 0;
    static unsigned long previousTime = 0;

    // Read the MPU6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float Gz = gz / 131.0;
    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
    yaw += Gz * dt;

    // Angle PID control
    float angleError = desiredAngle - yaw;
    angleIntegral += angleError * dt;
    float angleDerivative = (angleError - previousAngleError) / dt;
    float angleCorrection = angleKp * angleError + angleKi * angleIntegral + angleKd * angleDerivative;
    previousAngleError = angleError;

    // Adjust motor PWM based on angle correction
    pwmA = constrain(pwmA + angleCorrection, 0, MAX_PWM);
    pwmB = constrain(pwmB - angleCorrection, 0, MAX_PWM);

    // Set motor speeds
    ledcWrite(speedA, pwmA);
    ledcWrite(speedB, pwmB);

    // Print debug information every second
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 1000) {
        lastPrintTime = currentTime;
        Serial.print("Yaw: ");
        Serial.print(yaw);
        Serial.print("\tPWMA: ");
        Serial.print(pwmA);
        Serial.print("\tPWMB: ");
        Serial.println(pwmB);
    }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("MPU6050 connection successful");
}

void loop() {
  float Gz = 0.0;
  float dt = 0.0;
  float error = 0.0;

  for (int i = 0; i < 100; i++) {
     varset = constrain(abs(setpoint / (0.5 * yaw)), setpoint/3, setpoint);

    int errorA = varset - pulseCountA;
    integralA += errorA;
    int derivativeA = errorA - previousErrorA;
    pwmA = Kp * errorA + Ki * integralA + Kd * derivativeA;
    previousErrorA = errorA;

    int errorB = varset - pulseCountB;
    integralB += errorB;
    int derivativeB = errorB - previousErrorB;
    pwmB = Kp * errorB + Ki * integralB + Kd * derivativeB;
    previousErrorB = errorB;

    // Set motor speeds
    ledcWrite(speedA, pwmA);
    ledcWrite(speedB, pwmB);

    // Call the controlAngle function with the desired angle
    float desiredAngle = 90.0;  // Example: 90 degrees
    controlAngle(desiredAngle);

    // Reset encoder counts
    pulseCountA = 0;
    pulseCountB = 0;

    delay(50);
  }
}