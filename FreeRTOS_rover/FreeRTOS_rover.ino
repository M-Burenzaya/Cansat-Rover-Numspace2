//----------------------------------------------------------------------------------------------------------

//adding comment
// Za 2 oo alaad ogyaa
//test 3

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
//----------------------------------------------------------------------------------------------------------
#include <iostream>
#include <cmath> 

// Motor 
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

// Parachute
#define parachute 32

// Robot dimension 
#define r_length 22
#define w_radius 5

//Other constant
#define EARTH_RADIUS 6371
#define MOVE_DISTANCE 10.0 // Distance in meters

const int freq = 50;
const int speedA = 0;
const int speedB = 1;
const int resolution = 8;

unsigned int pulseCountA=0, pulseCountB=0;

//----------------------------------------------------------------------------------------------------------
#include <TinyGPSPlus.h>

TinyGPSPlus gps;

//----------------------------------------------------------------------------------------------------------
#include <MPU9250_WE.h>


// // MPU9250 and GPS configuration
// #define MPU9250_ADDR 0x68 
// MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

//----------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

//----------------------------------------------------------------------------------------------------------
struct Location{
  double lat;
  double lon;
};
typedef struct Location Location;

struct Rover{
  // 0 - unasan eseh, unasan ued daraagiin tuluv -> 1,
  // 1 - shuher taslah -> 2,
  // 2 - gps1 in utga avah -> 3,
  // 3 - chigeeree 10m yvah -> 4,
  // 4 - gps2 in utga avah -> 5,
  // 5 - untsug bodoh ->6,
  // 6 - bodson untsuguur ergeh -> 2,
  unsigned int r_status = 4;

  float fall_accelerate;
  double rot_angle=90;

  Location gps1; 
  Location gps2; 
  Location gps3; 
   
};
typedef struct Rover Rover;

Rover myRover;
//----------------------------------------------------------------------------------------------------------

void IRAM_ATTR encoderPulseA() {
  pulseCountA++;
  
}

void IRAM_ATTR encoderPulseB() {
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
  Wire.begin(); // i2c wire 
  // Calibrate MPU9250
  // Serial.println("Position your MPU9250 flat and don't move it - calibrating...");
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // myMPU9250.autoOffsets();
  // Serial.println("Done!");
  // vTaskDelay(2000 / portTICK_PERIOD_MS);

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

  digitalWrite(STBY, 1);
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

  myRover.gps3.lat = 47.91936; myRover.gps3.lon = 106.91755;  


  xTaskCreate(MPU_task, "MPU task", 2048,  NULL, 2,  NULL);
  // xTaskCreate(GPS_task, "GPS task", 2048,  NULL, 2,  NULL);
  // xTaskCreate(Break_Parachute_task, "Break_Parachute_task", 2048,  NULL, 2,  NULL);
  // xTaskCreate(Motor_task, "Motor task", 3000,  NULL, 2,  NULL);
  // xTaskCreate(Calculate_Angle_task, "Calculate_Angle_task", 2048,  NULL, 2,  NULL);

}

void MPU_task(void *pvParameters) // Temuulen MPU 9250 iig 100ms zaitai utga avj 10 udaa utga ter toond 1 ees oor too bnu gdgiig shalgan draagiin tolov false - draagiin tolov, true baival dahiad 10 utga avna 
{  // This is a task.
  bool one = true ; // false uyd draagiin stage ruu shiljene 

  for (;;){ // A Task shall never return or exit.
    if(myRover.r_status==0)
    {
      // for(int i = 0 ; i < 10 ; i++){
      //   gValue = myMPU9250.getGValues();
      //   myRover.fall_accelerate = myMPU9250.getResultantG(gValue);

      //   vTaskDelay(100 /portTICK_PERIOD_MS);

      //   if(myRover.fall_accelerate != 1) {
      //     one = false ;
      //   }
      // }
      // if(one == true){
      //   myRover.r_status = 1 ; 
      // }

      // Serial.print("MPU_9250 g : ");
      // Serial.println(myRover.fall_accelerate);

    }
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

      vTaskDelay(1500 / portTICK_PERIOD_MS);

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
      if(gps.encode(Serial2.read())){
        
        if(myRover.r_status == 2){
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Motor_task( void *pvParameters )
{
  double angleInRadians, arcLength;
  int encoderNum;

  for(;;)
  {

    
    if(myRover.r_status == 3)
    {
      digitalWrite(INA2, 0);
      digitalWrite(INA1, 1);
      
      digitalWrite(INB2, 1);
      digitalWrite(INB1, 0);

      ledcWrite(speedA, 160);
      ledcWrite(speedB, 150);

      vTaskDelay(3000/portTICK_PERIOD_MS);
      
      digitalWrite(INA2, 0);
      digitalWrite(INA1, 0);
      
      digitalWrite(INB2, 0);
      digitalWrite(INB1, 0);

      ledcWrite(speedA, 0);
      ledcWrite(speedB, 0);

      myRover.r_status = 7;
    }
    if(myRover.r_status == 6)
    {
      // Tootsoolson ontsogoor ergeh uildel
      double angleInRadians, arcLength;
      int encoderNum;
      angleInRadians = myRover.rot_angle * (M_PI / 180.0); // Convert degrees to radians
      arcLength = (angleInRadians / (2 * M_PI)) * (2 * M_PI * r_length); // Arc length formula
      encoderNum = (int)((80*arcLength)/(2*w_radius*M_PI));
      
      digitalWrite(INA2, 0);
      digitalWrite(INA1, 1);
      ledcWrite(speedA, 100);
      pulseCountA=0;
      Serial.printf("Encoder number: %d\n", encoderNum);
      // Serial.printf("Pulse count: %d\n", pulseCountA);/
      while(pulseCountA <= encoderNum)
      {
        Serial.printf("Pulse count: %d\n", pulseCountA);
      }
      Serial.printf("Pulse count: %d\n", pulseCountA);
      digitalWrite(INA2, 1);
      digitalWrite(INA1, 0);
      ledcWrite(speedA, 255);
      vTaskDelay((int)(myRover.rot_angle/2)/portTICK_PERIOD_MS);

      digitalWrite(INA2, 0);
      digitalWrite(INA1, 0);
      ledcWrite(speedA, 0);
  
      pulseCountA=0;

      double dlon2_3 = (myRover.gps3.lon - myRover.gps2.lon) * PI / 180.0;
      double dlat2_3 = (myRover.gps3.lat - myRover.gps2.lat) * PI / 180.0;
      double a2_3 = sin(dlat2_3 / 2) * sin(dlat2_3 / 2) +
                    cos(myRover.gps2.lat * PI / 180.0) * cos(myRover.gps3.lat * PI / 180.0) *
                    sin(dlon2_3 / 2) * sin(dlon2_3 / 2);
      double c2_3 = 2 * atan2(sqrt(a2_3), sqrt(1 - a2_3));
      double a = EARTH_RADIUS * c2_3 * 1000;

      Serial.print("Distance : ");
      Serial.println(a) ;

      if(a2_3 < 10) {
        myRover.r_status = 7 ; 
      }
      myRover.r_status = 2 ;
    }
    
    
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void Calculate_Angle_task(void *pvParameters) {
  bool clockwise; 
  for (;;) {
    if (myRover.r_status == 5) {
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

      Serial.print("Rotate Angle: ");
      Serial.println(myRover.rot_angle);
      myRover.r_status = 7;
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void loop()
{

}
