#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

unsigned long previousTime = 0, lastPrintTime = 0;
float yaw = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

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
  // Read raw accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw data to 'g' and 'deg/s'
  // float Ax = ax / 16384.0;  // assuming the accelerometer is set to ±2g
  // float Ay = ay / 16384.0;
  // float Az = az / 16384.0;

  // float Gx = gx / 131.0;  // assuming the gyroscope is set to ±250 deg/s
  // float Gy = gy / 131.0;
  float Gz = gz / 131.0;

  // Calculate roll and pitch
  // float roll = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180 / PI;
  // float pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180 / PI;

  // Calculate yaw
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0; 
  previousTime = currentTime;


  yaw += Gz * dt;

  // Print the results
  // Serial.print("Roll: ");
  // Serial.print(roll);
  // Serial.print(" Pitch: ");
  // Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);

  delay(50);

  while(1){
    float Gz = gz / 131.0;

  // Calculate roll and pitch
  // float roll = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180 / PI;
  // float pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180 / PI;

  // Calculate yaw
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0; 
  previousTime = currentTime;


  yaw += Gz * dt;

  // Print the results
  // Serial.print("Roll: ");
  // Serial.print(roll);
  // Serial.print(" Pitch: ");
  // Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);

  delay(50);
  }


}
