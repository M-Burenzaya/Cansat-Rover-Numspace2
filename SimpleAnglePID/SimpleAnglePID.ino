#include <Wire.h>
#include <MPU6050.h>

// Motor pins
#define PWMA 26
#define INA2 14
#define INA1 13
#define STBY 33
#define PWMB 25
#define INB2 27
#define INB1 12  

#define M1_ENA 19
#define M1_ENB 18

#define M2_ENA 5
#define M2_ENB 4

const int MAX_PWM = 255;
const int freq = 1000;
const int speedA = 0;
const int speedB = 1;
const int resolution = 8;

unsigned int pulseCountA = 0, pulseCountB = 0;
const int setpoint = 30;
int varset;

// PID coefficients
float Kp = 1, Ki = 0.7, Kd = 0.4;
float integralA = 0, integralB = 0;
float previousErrorA = 0, previousErrorB = 0;

// Yaw correction PID coefficients
float yawKp = 10.0;

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long previousTime = 0, lastPrintTime = 0;
float yaw = 0;

void IRAM_ATTR encoderPulseA() {
  pulseCountA++;
}

void IRAM_ATTR encoderPulseB() {
  pulseCountB++;
}

void Motor_setup() {
  pinMode(M1_ENB, INPUT);
  pinMode(M1_ENA, INPUT);
  pinMode(M2_ENB, INPUT);
  pinMode(M2_ENA, INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(INB1, OUTPUT);

  digitalWrite(STBY, 1);

  ledcSetup(speedA, freq, resolution);
  ledcAttachPin(PWMA, speedA);

  ledcSetup(speedB, freq, resolution);
  ledcAttachPin(PWMB, speedB);
}

void MPU_setup() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connection successful");
}

void setup() {
  Serial.begin(115200);
  Motor_setup();
  MPU_setup();

  attachInterrupt(digitalPinToInterrupt(M1_ENB), encoderPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENB), encoderPulseB, RISING);

  // Initial motor directions
  digitalWrite(INA2, 0);
  digitalWrite(INA1, 1);
  digitalWrite(INB2, 1);
  digitalWrite(INB1, 0);
}

void turnToAngle(float desiredYaw) {
  float Gz = 0.0;
  float dt = 0.0;
  float err = 0.0;
  int pwm = 0;

  int consecutiveSmallErrors = 0;
  const int maxConsecutiveSmallErrors = 3;
  const float errorThreshold = 1.0;
  const unsigned long settlingTime = 2000;
  unsigned long settleStartTime = millis();

  for (int i = 0; i < 100; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    Gz = gz / 131.0;
    err += Gz * dt;

    delay(10);
  }

  err = err / 100;
  
  float currentYaw = 0.0;
  float integral = 0;
  float previousError = 0;

  while (1) {
    if (desiredYaw > 0) {
      
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      unsigned long currentTime = millis();
      dt = (currentTime - previousTime) / 1000.0;
      previousTime = currentTime;

      Gz = gz / 131.0;

      currentYaw = currentYaw + (Gz * dt - err);

      int direction = (desiredYaw > currentYaw) ? 1 : 0;
      float error = abs(desiredYaw - currentYaw);

      if (direction == 1) {
        digitalWrite(INA2, 0);
        digitalWrite(INA1, 1);
        digitalWrite(INB2, 1);
        digitalWrite(INB1, 0);
      } else {
        digitalWrite(INA2, 1);
        digitalWrite(INA1, 0);
        digitalWrite(INB2, 0);
        digitalWrite(INB1, 1);
      }

      integral += error * dt;
      float derivative = (error - previousError) / dt;
      float controlSignal = Kp * error + Ki * integral + Kd * derivative;
      previousError = error;

      pwm = constrain(controlSignal, 0, MAX_PWM);

      ledcWrite(speedA, pwm);

      Serial.print("Current Yaw: ");
      Serial.print(currentYaw);
      Serial.print("\tDesired Yaw: ");
      Serial.print(desiredYaw);
      Serial.print("\tPWM: ");
      Serial.println(pwm);

      if (abs(error) < errorThreshold) {
        consecutiveSmallErrors++;
      } else {
        consecutiveSmallErrors = 0;
      }

      if (consecutiveSmallErrors >= maxConsecutiveSmallErrors && 
          (currentTime - settleStartTime) >= settlingTime) {
        break;
      }

    } else {

    }

    delay(10);
  }
  // Stop the motors
  ledcWrite(speedA, 0);
  ledcWrite(speedB, 0);
}

void loop() {

  // Example: Turn the rover to 90 degrees
  turnToAngle(90.0);
  // delay(2000);
  // turnToAngle(-90.0);
  // delay(2000);
  // turnToAngle(180.0);
  // delay(2000);
  // turnToAngle(-180.0);

  // Wait for a while before another action
  delay(2000);
}
