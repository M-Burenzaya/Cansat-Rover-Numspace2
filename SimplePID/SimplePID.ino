#include <Arduino.h>

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

// Define maximum PWM value
const int MAX_PWM = 255;

const int freq = 1000;
const int speedA = 0;
const int speedB = 1;
const int resolution = 8;

unsigned int pulseCountA = 0, pulseCountB = 0;

const int setpoint = 10;
int pwmA = 0, pwmB = 0;

// PID coefficients
float Kp = 1, Ki = 0.7, Kd = 0.5;

float integralA = 0, integralB = 0;
float previousErrorA = 0, previousErrorB = 0;

void IRAM_ATTR encoderPulseA() {
  pulseCountA++;
}

void IRAM_ATTR encoderPulseB() {
  pulseCountB++;
}

void setup() {
  Serial.begin(115200);

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

  attachInterrupt(digitalPinToInterrupt(M1_ENB), encoderPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENB), encoderPulseB, RISING);

  // Initial motor directions
  digitalWrite(INA2, 0);
  digitalWrite(INA1, 1);
  digitalWrite(INB2, 1);
  digitalWrite(INB1, 0);
}

void loop() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();

  int errorA = setpoint - pulseCountA;
  integralA += errorA;
  int derivativeA = errorA - previousErrorA;
  pwmA = Kp * errorA + Ki * integralA + Kd * derivativeA;
  previousErrorA = errorA;

  int errorB = setpoint - pulseCountB;
  integralB += errorB;
  int derivativeB = errorB - previousErrorB;
  pwmB = Kp * errorB + Ki * integralB + Kd * derivativeB;
  previousErrorB = errorB;

  pwmA = constrain(pwmA, 0, MAX_PWM);
  pwmB = constrain(pwmB, 0, MAX_PWM);

  // Set motor speeds
  ledcWrite(speedA, pwmA);
  ledcWrite(speedB, pwmB);

  if (currentTime - lastPrintTime >= 1000) {
    lastPrintTime = currentTime;
    Serial.print("EnA: ");
    Serial.print(pulseCountA);

    Serial.print("     EnB: ");
    Serial.println(pulseCountB);
  }

  pulseCountA = 0;
  pulseCountB = 0;

  delay(50);
}
