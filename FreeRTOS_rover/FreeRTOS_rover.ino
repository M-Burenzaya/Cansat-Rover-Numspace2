//----------------------------------------------------------------------------------------------------------
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
//----------------------------------------------------------------------------------------------------------
#include <iostream>
#include <cmath>


//----------------------------------------------------------------------------------------------------------

#include <Wire.h>                 // MPU6050
#include <MPU6050.h>

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long previousTime = 0, lastPrintTime = 0;
float yaw = 0;
//----------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------MOTOR-----------------------------------------

#define PWMA 14
#define INA2 26 //temuulen pin oorchlov
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

//----------------------------------------------------------------------------------------------------------


// Parachute
#define parachute 32

// Robot dimension 
#define r_length 22
#define w_radius 5

//Other constant
#define EARTH_RADIUS 6371
#define MOVE_DISTANCE 10.0 // Distance in meters

//----------------------------------------------------------------------------------------------------------
#include <TinyGPSPlus.h>

TinyGPSPlus gps;

//----------------------------------------------------------------------------------------------------------
struct Location
{
  double lat;
  double lon;
};
typedef struct Location Location;

struct Rover
{
  // 0 - unasan eseh, unasan ued daraagiin tuluv -> 1,
  // 1 - shuher taslah -> 2,
  // 2 - gps1 in utga avah -> 3,
  // 3 - chigeeree 10m yvah -> 4,
  // 4 - gps2 in utga avah -> 5,
  // 5 - untsug bodoh ->6,
  // 6 - bodson untsuguur ergeh -> 2,
  unsigned int r_status = 2;

  float fall_accelerate;
  double rot_angle;
  bool enableForward = false;

  Location gps1; 
  Location gps2; 
  Location gps3; 
  Location gps_current;
   
};
typedef struct Rover Rover;

Rover myRover;
//----------------------------------------------------------------------------------------------------------

void IRAM_ATTR encoderPulseA()
{
  pulseCountA++;
}

void IRAM_ATTR encoderPulseB()
{
  pulseCountB++;
}

//----------------------------------------------------------------------------------------------------------

void MPU_task( void *pvParameters );
void Break_Parachute_task( void *pvParameters );
void GPS_task( void *pvParameters );
void Motor_task( void *pvParameters );
void Calculate_Angle_task( void *pvParameters );


void setup()
{
  
  Serial.begin(115200); // Communication with the computer
  Serial2.begin(9600); // // Communication with the GPS module

  Wire.begin();
  mpu.initialize();
  while(!mpu.testConnection()){
    vTaskDelay(100/portTICK_PERIOD_MS);
    Serial.println("MPU6050 connection failed");
  }
  Serial.println("MPU6050 connection successful");
  
  pinMode(parachute, OUTPUT);

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

  // digitalWrite(STBY, 1);
  ledcSetup(speedA, freq, resolution);
  ledcAttachPin(PWMA, speedA);

  ledcSetup(speedB, freq, resolution);
  ledcAttachPin(PWMB, speedB);

  attachInterrupt(digitalPinToInterrupt(M1_ENB), encoderPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENB), encoderPulseB, RISING);

  //-------------------lat-----------------------------
  // myRover.gps1.lat = 47.91879; myRover.gps1.lon = 106.91733; 
  // // myRover.gps3.lat = 47.91868; myRover.gps3.lon = 106.91786;  // uragshaa
  // myRover.gps3.lat = 47.91898; myRover.gps3.lon = 106.91781;  // hoishoo

  // myRover.gps2.lat = 47.91884; myRover.gps2.lon = 106.91783; 

  //-------------------long----------------------------

  
  
  // myRover.gps1.lat = 47.91867; myRover.gps1.lon = 106.91762; 
  // //myRover.gps3.lat = 47.91895; myRover.gps3.lon = 106.91733;  // zuun 
  // myRover.gps3.lat = 47.91899; myRover.gps3.lon = 106.91779;  // baruun
  
  // myRover.gps2.lat = 47.91898; myRover.gps2.lon = 106.91757; 


  //--------------  RANDOM -----------------------------------

  // myRover.gps1.lat = 47.97924; myRover.gps1.lon = 106.76013; 
  // myRover.gps2.lat = 47.98363; myRover.gps2.lon = 106.89401; 
  // myRover.gps3.lat = 47.87563; myRover.gps3.lon = 106.90876;  
  
   //--------------  RANDOM 1 -----------------------------------

  // myRover.gps1.lat = 47.91957; myRover.gps1.lon = 106.91703; 
  // myRover.gps2.lat = 47.91833; myRover.gps2.lon = 106.91802; 
  // myRover.gps3.lat = 47.91672; myRover.gps3.lon = 106.91726;  

   //--------------  RANDOM 2 -----------------------------------

  
  //myRover.gps1.lat = 47.91836; myRover.gps1.lon = 106.91715; 
  //--------- DISTANCE 1  
  // myRover.gps2.lat = 47.1; myRover.gps2.lon = 106.6; 
  // myRover.gps3.lat = 47.91846; myRover.gps3.lon = 106.91667;  
  
  // ----------DISTANCE 2 
  // myRover.gps2.lat = 47.91935; myRover.gps2.lon = 106.91753; 

  // myRover.gps3.lat = 47.7206753; myRover.gps3.lon = 107.4117167;   //real

  // myRover.gps3.lat = 47.7213445; myRover.gps3.lon = 107.4120319;  

  myRover.gps3.lat = 47.7211681; myRover.gps3.lon = 107.4127105;  
  // myRover.gps3.lat = 47.72074; myRover.gps3.lon = 107.412424;  
  // myRover.gps3.lat = 47.720973; myRover.gps3.lon = 107.411963;  //home
  // myRover.gps3.lat = 47.720889; myRover.gps3.lon = 107.412804;  

  xTaskCreate(MPU_task, "MPU task", 2048,  NULL, 2,  NULL);
  xTaskCreate(GPS_task, "GPS task", 3000,  NULL, 2,  NULL);
  xTaskCreate(Break_Parachute_task, "Break_Parachute_task", 2048,  NULL, 2,  NULL);
  xTaskCreate(Motor_task, "Motor task", 6000,  NULL, 2,  NULL);
  xTaskCreate(Calculate_Angle_task, "Calculate_Angle_task", 2048,  NULL, 2,  NULL);

}

void MPU_task(void *pvParameters)
{  
  bool one = true ; // false uyd draagiin stage ruu shiljene 

  float Az, Ay, Ax;
  float resultant, preResultant;
  int i=0;
  for (;;){ // A Task shall never return or exit.
    if(myRover.r_status==0)
    {
        for( i=0; i<100; i++){
          mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          Az = az / 16384.0;
          Ay = ay / 16384.0;
          Ax = ax / 16384.0;
          preResultant = resultant;
          resultant = 0.0;
          resultant = sqrt(sq(Az) + sq(Ay) + sq(Az));
          
          Serial.print("Ax: ");
          Serial.println(Ax);
          Serial.print("Ay: ");
          Serial.println(Ay);
          Serial.print("Az: ");
          Serial.println(Az);
          Serial.print("resultantG acc: ");
          Serial.println(resultant);
          vTaskDelay(100/portTICK_PERIOD_MS);//1000
          if(resultant - preResultant > 0.05 || resultant - preResultant < -0.05){
            Serial.println("--------------BREAK----------- ");
            break;
          }
            
        }
        if(i==100){//800
          myRover.r_status = 1; 
          Serial.println("My Rover dropped on the ground");
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("MPU task stack high watermark: ");
    // Serial.println(uxHighWaterMark);

    vTaskDelay(1000/portTICK_PERIOD_MS);

  }
}
void Break_Parachute_task( void *pvParameters )
{
  for(;;)
  {
    if(myRover.r_status == 1)
    {
      // Shuher taslah uildel
      digitalWrite(parachute, HIGH); // Set high
      Serial.print("HIGH") ; 

      vTaskDelay(1000 / portTICK_PERIOD_MS);//5000

      digitalWrite(parachute, LOW); // Set low 
      Serial.println("LOW") ; 

      myRover.r_status = 2; // 2 baih ystoi  
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void GPS_task(void *pvParameters) 
{
  for (;;)
  {
    while (Serial2.available() > 0)
    {
      
      if(gps.encode(Serial2.read()))
      {
        // Serial.println("Data arrived");
        if(myRover.r_status == 2)
        {
          // Serial.println("Data arrived");
          if( myRover.gps1.lat != gps.location.lat() && myRover.gps1.lon != gps.location.lng() && 0 != gps.location.lat() && 0 !=gps.location.lng())
          {
            myRover.gps1.lat = gps.location.lat();
            myRover.gps1.lon = gps.location.lng();
            Serial.print("GPS 1 Latitude: ");
            Serial.println(myRover.gps1.lat, 6);
            Serial.print("GPS 1 Longitude: ");
            Serial.println(myRover.gps1.lon, 6);
            myRover.r_status = 3;
          }
        }
        // if(myRover.r_status == 3)
        // {
        //   if( myRover.gps1.lat != gps.location.lat() && myRover.gps1.lon != gps.location.lng() && 0 != gps.location.lat() && 0 !=gps.location.lng())
        //   {
        //     myRover.gps_current.lat = gps.location.lat();
        //     myRover.gps_current.lon = gps.location.lng();
        //     Serial.print("GPS Current Latitude: ");
        //     Serial.println(myRover.gps_current.lat, 6);
        //     Serial.print("GPS Current Longitude: ");
        //     Serial.println(myRover.gps_current.lon, 6);
        //     myRover.enableForward = true;
        //     // myRover.r_status = 4;
        //   }
        // }
        if(myRover.r_status == 4)
        {
          if(myRover.gps2.lat != gps.location.lat() && myRover.gps2.lon != gps.location.lng() && 0 != gps.location.lat() && 0 !=gps.location.lng())
          {
            myRover.gps2.lat = gps.location.lat();
            myRover.gps2.lon = gps.location.lng();
            Serial.print("GPS 2 Latitude: ");
            Serial.println(myRover.gps2.lat, 6);
            Serial.print("GPS 2 Longitude: ");
            Serial.println(myRover.gps2.lon, 6);
            myRover.r_status = 5; // 5
          }
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Motor_task( void *pvParameters )
{
  double angleInRadians, arcLength;
  int encoderNum;

  //----------------------------------------------------------

 

  //-----------------------DISTANCE---------------------------
  double dlon2_3, dlat2_3, a2_3, c2_3, a;
  //----------------------------------------------------------

  

 

  for(;;)
  {
    if(myRover.r_status == 3)
    {
       const int setpoint = 90;
      int varset;

      int pwmA = 0, pwmB = 0;

      // PID coefficients
      float Kp = 0.8, Ki = 0.5, Kd = 1;

      float integralA = 0, integralB = 0;
      float previousErrorA = 0, previousErrorB = 0;

      // Yaw correction PID coefficients
      float yawKp = 15;


      float yaw = 0;
      float Gz = 0.0;
      float dt = 0.0;
      float err = 0.0;
      unsigned int cnt=0;

      for (int i = 0; i < 100; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        unsigned long currentTime = millis();
        dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        Gz = gz / 131.0;
        err = err + Gz * dt;

        vTaskDelay(50/portTICK_PERIOD_MS);
      }
      err = err / 100;
      Serial.println(err);

      //------------------------------------Forward_path----------------------
      

      while(1){
        // Serial.println("Forward moving");
        digitalWrite(INA2, 0);
        digitalWrite(INA1, 1);
        digitalWrite(INB2, 1);
        digitalWrite(INB1, 0);
        varset = constrain(abs(setpoint / (0.1 * yaw)), setpoint/3, setpoint);

        //-------------------------------------------------------------------

        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        unsigned long currentTime = millis();
        dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        Gz = gz / 131.0;
        yaw = yaw + (Gz * dt - err);

        //-------------------------------------------------------------------

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

        //-------------------------------------------------------------------

        // Corrective action based on yaw
        float yawCorrection = yawKp * yaw;

        pwmA = constrain(pwmA - yawCorrection - abs(yaw * 2), 0, MAX_PWM);
        pwmB = constrain(pwmB + yawCorrection - abs(yaw * 2), 0, MAX_PWM);

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
    
        vTaskDelay(50/portTICK_PERIOD_MS);
        
        // if(abs(yaw)>10)
        // {
        //   // digitalWrite(INA2, 0);
        //   // digitalWrite(INA1, 0);
        //   // digitalWrite(INB2, 0);
        //   // digitalWrite(INB1, 0);
        //   // vTaskDelay(1000/portTICK_PERIOD_MS);
        //   integralA = 0, integralB = 0;
        //   // previousErrorA = 0, previousErrorB = 0;
        //   // yaw = 0;
        // }
        //----------------------------------------------------------

        // while(!myRover.enableForward){
        //   vTaskDelay(10/portTICK_PERIOD_MS);
        // }
        // myRover.enableForward=false;
        // dlon2_3 = (myRover.gps1.lon - myRover.gps_current.lon) * PI / 180.0;
        // dlat2_3 = (myRover.gps1.lat - myRover.gps_current.lat) * PI / 180.0;
        // a2_3 = sin(dlat2_3 / 2) * sin(dlat2_3 / 2) +
        //               cos(myRover.gps_current.lat * PI / 180.0) * cos(myRover.gps1.lat * PI / 180.0) *
        //               sin(dlon2_3 / 2) * sin(dlon2_3 / 2);
        // c2_3 = 2 * atan2(sqrt(a2_3), sqrt(1 - a2_3));
        // a = EARTH_RADIUS * c2_3 * 1000;

        // Serial.print("Forward Distance : ");
        // Serial.println(a) ;
        cnt++;
        if(cnt > 150){
          digitalWrite(INA2, 0);
          digitalWrite(INA1, 0);
          digitalWrite(INB2, 0);
          digitalWrite(INB1, 0);
          ledcWrite(speedA, 0);
          ledcWrite(speedB, 0);

          myRover.r_status = 4;
          break;
          cnt=0;
        }
      }
    }
    if(myRover.r_status == 6)
    {
      // Serial.println("Rotating");
      // PID coefficients
      float Kp_r = 5, Ki_r = 1, Kd_r = 0.4;
      float Gz_r = 0.0;
      float dt_r = 0.0;
      float err_r = 0.0;
      int pwm = 0;

      float currentYaw = 0.0;
      float integral = 0.0;
      float previousError_r = 0;

      int consecutiveSmallErrors = 0;
      const int maxConsecutiveSmallErrors = 3;
      const float errorThreshold = 1.0;
      const unsigned long settlingTime = 2000;
      unsigned long settleStartTime = millis();

      unsigned long currentTime_r, previousTime_r;
      int direction;
      float error_r;
      float derivative_r;
      float controlSignal;

      for (int i = 0; i < 100; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        currentTime_r = millis();
        dt_r = (currentTime_r - previousTime_r) / 1000.0;
        previousTime_r = currentTime_r;
        Gz_r = gz / 131.0;
        err_r += Gz_r * dt_r;

        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      err_r = err_r / 100;

      
      if (myRover.rot_angle > 0) {
        
        while (1) {
          mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

          currentTime_r = millis();
          dt_r = (currentTime_r - previousTime_r) / 1000.0;
          previousTime_r = currentTime_r;

          Gz_r = gz / 131.0;

          currentYaw = currentYaw + (Gz_r * dt_r - err_r);

          direction = (myRover.rot_angle > currentYaw) ? 1 : 0;
          error_r = myRover.rot_angle - currentYaw;

          if (direction == 1) {
            digitalWrite(INA2, 0);
            digitalWrite(INA1, 1);

          } else {
            digitalWrite(INA2, 1);
            digitalWrite(INA1, 0);
          }

          integral += error_r * dt_r;
          derivative_r = (error_r - previousError_r) / dt_r;
          controlSignal = 80 + Kp_r * error_r + Ki_r * integral + Kd_r * derivative_r;
          previousError_r = error_r;

          pwm = constrain(controlSignal, 0, MAX_PWM);

          ledcWrite(speedA, pwm);

          Serial.print("Current Yaw: ");
          Serial.print(currentYaw);
          Serial.print("\tDesired Yaw: ");
          Serial.print(myRover.rot_angle);
          Serial.print("\tPWM: ");
          Serial.println(pwm);

          if (abs(error_r) < errorThreshold) {
            consecutiveSmallErrors++;
          } else {
            consecutiveSmallErrors = 0;
          }

          if (consecutiveSmallErrors >= maxConsecutiveSmallErrors && (currentTime_r - settleStartTime) >= settlingTime)
          {
            break;
          }
          if(abs(currentYaw)>180 )
            break;

          vTaskDelay(10/portTICK_PERIOD_MS);
        }
      } 
      else {
        while (1) {
          mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

          currentTime_r = millis();
          dt_r = (currentTime_r - previousTime_r) / 1000.0;
          previousTime_r = currentTime_r;

          Gz_r = gz / 131.0;

          currentYaw = currentYaw + (Gz_r * dt_r - err_r);

          direction = (myRover.rot_angle > currentYaw) ? 1 : 0;
          error_r = abs(myRover.rot_angle) - abs(currentYaw);

          if (direction == 1) {
            digitalWrite(INB2, 0);
            digitalWrite(INB1, 1);

          } else {
            digitalWrite(INB2, 1);
            digitalWrite(INB1, 0);
          }

          integral += error_r * dt_r;
          derivative_r = (error_r - previousError_r) / dt_r;
          controlSignal = 80 + Kp_r * error_r + Ki_r * integral + Kd_r * derivative_r;
          previousError_r = error_r;

          pwm = constrain(controlSignal, 0, MAX_PWM);

          ledcWrite(speedB, pwm);

          Serial.print("Current Yaw: ");
          Serial.print(currentYaw);
          Serial.print("\tDesired Yaw: ");
          Serial.print(myRover.rot_angle);
          Serial.print("\tPWM: ");
          Serial.println(pwm);

          if (abs(error_r) < errorThreshold) {
            consecutiveSmallErrors++;
          } else {
            consecutiveSmallErrors = 0;
          }

          if (consecutiveSmallErrors >= maxConsecutiveSmallErrors && (currentTime_r - settleStartTime) >= settlingTime) {
            break;
          }
          if(abs(currentYaw)>180 )
            break;
          vTaskDelay(10/portTICK_PERIOD_MS);
        }
      }

      ledcWrite(speedA, 0);
      ledcWrite(speedB, 0);
  
      pulseCountA=0;
      pulseCountB=0;

      dlon2_3 = (myRover.gps3.lon - myRover.gps2.lon) * PI / 180.0;
      dlat2_3 = (myRover.gps3.lat - myRover.gps2.lat) * PI / 180.0;
      a2_3 = sin(dlat2_3 / 2) * sin(dlat2_3 / 2) +
                    cos(myRover.gps2.lat * PI / 180.0) * cos(myRover.gps3.lat * PI / 180.0) *
                    sin(dlon2_3 / 2) * sin(dlon2_3 / 2);
      c2_3 = 2 * atan2(sqrt(a2_3), sqrt(1 - a2_3));
      a = EARTH_RADIUS * c2_3 * 1000;

      Serial.print("Distance : ");
      Serial.println(a) ;

      if(a < 4)
      {
        myRover.r_status = 7 ; 
      }
      else
        myRover.r_status = 2 ;
    }
    if(myRover.r_status != 3 && myRover.r_status != 6 ){
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
    
  }
}

void Calculate_Angle_task(void *pvParameters)
{
  bool clockwise; 
  for (;;) {
    if (myRover.r_status == 5)
    {
      clockwise = false ; 
      // Calculate distances using haversine formula
      double dlon2_3 = (myRover.gps3.lon - myRover.gps2.lon) * PI / 180.0;
      double dlat2_3 = (myRover.gps3.lat - myRover.gps2.lat) * PI / 180.0;
      double a2_3 = sin(dlat2_3 / 2) * sin(dlat2_3 / 2) +
                    cos(myRover.gps2.lat * PI / 180.0) * cos(myRover.gps3.lat * PI / 180.0) *
                    sin(dlon2_3 / 2) * sin(dlon2_3 / 2);
      double c2_3 = 2 * atan2(sqrt(a2_3), sqrt(1 - a2_3));
      double a = EARTH_RADIUS * c2_3 * 1000;

      double dlon1_3 = (myRover.gps3.lon - myRover.gps1.lon) * PI / 180.0;
      double dlat1_3 = (myRover.gps3.lat - myRover.gps1.lat) * PI / 180.0;
      double a1_3 = sin(dlat1_3 / 2) * sin(dlat1_3 / 2) +
                    cos(myRover.gps1.lat * PI / 180.0) * cos(myRover.gps3.lat * PI / 180.0) *
                    sin(dlon1_3 / 2) * sin(dlon1_3 / 2);
      double c1_3 = 2 * atan2(sqrt(a1_3), sqrt(1 - a1_3));
      double b = EARTH_RADIUS * c1_3 * 1000;

      double dlon1_2 = (myRover.gps2.lon - myRover.gps1.lon) * PI / 180.0;
      double dlat1_2 = (myRover.gps2.lat - myRover.gps1.lat) * PI / 180.0;
      double a1_2 = sin(dlat1_2 / 2) * sin(dlat1_2 / 2) +
                    cos(myRover.gps1.lat * PI / 180.0) * cos(myRover.gps2.lat * PI / 180.0) *
                    sin(dlon1_2 / 2) * sin(dlon1_2 / 2);
      double c1_2 = 2 * atan2(sqrt(a1_2), sqrt(1 - a1_2));
      double c = EARTH_RADIUS * c1_2 * 1000;

      // Calculate rotation angle
      myRover.rot_angle = 180 - (acos((a * a + c * c - b * b) / (2 * a * c)) * 180.0 / PI);
      
      if(myRover.gps3.lat < myRover.gps1.lat && myRover.gps3.lat < myRover.gps2.lat){
        if(myRover.gps2.lon > myRover.gps1.lon)
          clockwise = true ; 
        else if(myRover.gps2.lon < myRover.gps3.lon && myRover.gps1.lon < myRover.gps3.lon && myRover.gps2.lat > myRover.gps1.lat)
          clockwise = true; 
      }
      else if(myRover.gps3.lat > myRover.gps1.lat && myRover.gps3.lat > myRover.gps2.lat){
        if(myRover.gps2.lon < myRover.gps1.lon)
          clockwise = true; 
        else if(myRover.gps2.lon > myRover.gps3.lon && myRover.gps1.lon > myRover.gps3.lon && myRover.gps2.lat < myRover.gps1.lat)
          clockwise = true;
      }
      else if(myRover.gps1.lat > myRover.gps3.lat && myRover.gps2.lat < myRover.gps3.lat && myRover.gps3.lon < myRover.gps1.lon && myRover.gps3.lon < myRover.gps2.lon ){
        if(myRover.gps2.lat < myRover.gps1.lat)
          clockwise = true ; 
      }
      else if(myRover.gps1.lat < myRover.gps3.lat && myRover.gps2.lat > myRover.gps3.lat && myRover.gps3.lon > myRover.gps1.lon && myRover.gps3.lon > myRover.gps2.lon ){
        if(myRover.gps2.lat > myRover.gps1.lat)
          clockwise = true ; 
      }

      if(clockwise)
        myRover.rot_angle = 360 - myRover.rot_angle ;  

      if(myRover.rot_angle >180){
        myRover.rot_angle = myRover.rot_angle -360;
      }
      Serial.print("Rotate Angle: ");
      Serial.println(myRover.rot_angle);
      myRover.r_status = 6;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


void loop()
{

}
