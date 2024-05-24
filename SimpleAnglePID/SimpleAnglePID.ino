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
  float Gz = 0.0;
  float dt = 0.0;
  float error = 0.0;

  for (int i = 0; i < 100; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    Gz = gz / 131.0;
    error += Gz * dt;
    
    delay(50);
  }

  Serial.print(error);
  Serial.print("\t");
  error = error / 100;
  Serial.println(error);
 

  while(1){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    Gz = gz / 131.0;
    yaw = yaw + (Gz * dt - error);

    Serial.print(" Yaw: ");
    Serial.println(yaw);

    delay(50);
  }
}