#include "GY521.h"
#include <Wire.h>
#include <math.h>

GY521 sensor(0x68);

int XValue, XVALUE, servoVal;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double avAcX = 0, avAcY = 0, avAcZ = 0, avGyX =0, avGyY=0, avGyZ=0;

double GyXBias = 4.5096, GyYBias = 0.32667, GyZBias = -0.96360;
double AcXBias = 0.05383, AcYBias = -0.0198175, AcZBias = -0.066235;
double AcXScale = 0.991449, AcYScale = 0.995614, AcZScale = 0.974915;

float alpha = 0.9;
double compRoll = 0, accRoll = 0;

int counter = 0;
float ax, ay,az,aix,aiy,aiz;
float gx,gy,gz,gix,giy,giz;
float roll, pitch, heading;
unsigned long microsPerReading, microsPrevious;
float lastError=0, iError=0;
float keel_angle;

int servoPin = 12;

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.print("GY521_LIB_VERSION: ");
  Serial.println(GY521_LIB_VERSION);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);       // PWR_MGMT_1 register
  Wire.write(0);          // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);
  while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(1000);
  }
  sensor.setAccelSensitivity(0);  //  2g
  sensor.setGyroSensitivity(0);   //  250 degrees/s

  sensor.setThrottle();
  //Serial.println("start...");

  microsPerReading = 1000000 / 10;
  microsPrevious = micros();
}


void loop()
{
  unsigned long microsNow;
  int consts = 0;

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // convert from raw data to gravity and degrees/second units
    sensor.read();

    float dt = (microsNow - microsPrevious) / 1000000;
    
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // Request a total of 14 registers

    AcX = (correctRawAccSign((Wire.read() << 8 | Wire.read())/16384.0)-AcXBias)*AcXScale;  // ACCEL_XOUT_H & ACCEL_XOUT_L
    AcY = (correctRawAccSign((Wire.read() << 8 | Wire.read())/16384.0)-AcYBias)*AcYScale;  // ACCEL_YOUT_H & ACCEL_YOUT_L
    AcZ = (correctRawAccSign((Wire.read() << 8 | Wire.read())/16384.0)-AcZBias)*AcZScale;  // ACCEL_ZOUT_H & ACCEL_ZOUT_L
    Tmp = (Wire.read() << 8 | Wire.read())/16384.0;  // TEMP_OUT_H & TEMP_OUT_L
    GyX = correctRawGyroSign((Wire.read() << 8 | Wire.read())/131.0) - GyXBias;  // GYRO_XOUT_H & GYRO_XOUT_L
    GyY = correctRawGyroSign((Wire.read() << 8 | Wire.read())/131.0) - GyYBias;  // GYRO_YOUT_H & GYRO_YOUT_L
    GyZ = correctRawGyroSign((Wire.read() << 8 | Wire.read())/131.0) - GyZBias;  // GYRO_ZOUT_H & GYRO_ZOUT_L

    //Complementary Filter implementation

    accRoll = atan2(AcY, sqrt(AcX*AcX + AcZ*AcZ)) * 180.0 / PI;
    compRoll = alpha *  (compRoll + GyX * dt) + (1.0 - alpha)*accRoll;

    Serial.print("Comp Roll ");
    Serial.print(compRoll);
    Serial.print(" \n");
    Serial.print("Const ");
    Serial.print(consts);
    Serial.print(" \n");

    // Calibration section
    /*if(counter < 1500){
        avGyX += GyX;
        avGyY += GyY;
        avGyZ += GyZ;
        Serial.println(counter);
    }
    if(counter < 1500){
        avAcX += AcX;
        avAcY += AcY;
        avAcZ += AcZ;
        counter++;
        Serial.println(counter);
    }*/
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
  
  delay(3);
  // Calibration section
  /*if(counter>=1500){
    Serial.println(avAcX / 1500, 5);
    Serial.println(avAcY / 1500, 5);
    Serial.println(avAcZ / 1500, 5);
    Serial.println(" ---------------------  ");
  }*/
  
}

double correctRawAccSign(double aRaw) {
  double a;
  if(aRaw > 2){
      a = aRaw - 4;
  }
  else{
      a = aRaw;
  }
  return a;
}

double correctRawGyroSign(double gRaw) {
  double g;
  if(gRaw > 250){
      g = gRaw - 500;
  }
  else{
      g = gRaw;
  }
  return g;
}