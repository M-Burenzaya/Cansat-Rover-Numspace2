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

//----------------------------------------------------------

const int MAX_PWM = 255;

const int freq = 1000;
const int speedA = 0;
const int speedB = 1;
const int resolution = 8;

unsigned int pulseCountA = 0, pulseCountB = 0;

//----------------------------------------------------------

const int setpoint = 30;
int varset;

int pwmA = 0, pwmB = 0;

// PID coefficients
float Kp = 1, Ki = 0.2, Kd = 0.5;

float integralA = 0, integralB = 0;
float previousErrorA = 0, previousErrorB = 0;

// Yaw correction PID coefficients
float yawKp = 10.0;

//----------------------------------------------------------

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long previousTime = 0, lastPrintTime = 0;
float yaw = 0;

//----------------------------------------------------------

// Encoder ISR functions
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

//----------------------------------------------------------

void MPU_setup() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connection successful");
}

//----------------------------------------------------------

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

//----------------------------------------------------------

void loop() {
  // PID control for motors
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

  // Yaw correction

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float Gz = gz / 131.0;
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  yaw += Gz * dt;

  // Corrective action based on yaw
  float yawCorrection = yawKp * yaw;

  pwmA = constrain(pwmA - yawCorrection - abs(yaw / 2), 0, MAX_PWM);
  pwmB = constrain(pwmB + yawCorrection - abs(yaw / 2), 0, MAX_PWM);

  // Set motor speeds
  ledcWrite(speedA, pwmA);
  ledcWrite(speedB, pwmB);

  if (currentTime - lastPrintTime >= 1000) {
    lastPrintTime = currentTime;
    Serial.print("EnA: ");
    Serial.print(pulseCountA);
    Serial.print("\tEnB: ");
    Serial.print(pulseCountB);
    Serial.print("\tYaw: ");
    Serial.print(yaw);
    Serial.print("\tPWMA: ");
    Serial.print(pwmA);
    Serial.print("\tPWMB: ");
    Serial.print(pwmA);
    Serial.print("\tvarset: ");
    Serial.println(varset);
  }

  pulseCountA = 0;
  pulseCountB = 0;

  delay(50);
}

