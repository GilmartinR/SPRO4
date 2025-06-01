#include "GY521.h"
#include <MadgwickAHRS.h>
#include <Wire.h>

Madgwick filter;

GY521 sensor(0x68);

int XValue, XVALUE, servoVal;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

uint32_t counter = 0;
float ax, ay,az,aix,aiy,aiz;
float gx,gy,gz,gix,giy,giz;
float roll, pitch, heading;
unsigned long microsPerReading, microsPrevious;
float lastError=0, iError=0, prevFilteredError=0;
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
  filter.begin(10);

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
  Serial.println("start...");

  //  set calibration values from calibration sketch.
  sensor.axe = -0.2271997;
  sensor.aye = 0.0152197;
  sensor.aze = -0.9619946;
  sensor.gxe = -4.5626717;
  sensor.gye = -0.4598473;
  sensor.gze = 1.1456488;

  pinMode(servoPin, OUTPUT);
  writeServo(90);

  microsPerReading = 1000000 / 10;
  microsPrevious = micros();
}


void loop()
{
  unsigned long microsNow;

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // convert from raw data to gravity and degrees/second units
    sensor.read();
    /*aix = sensor.getAccelX();
    aiy = sensor.getAccelY();
    aiz = sensor.getAccelZ();
    gix = sensor.getAngleX();
    giy = sensor.getAngleY();
    giz = sensor.getAngleZ();*/

    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // Request a total of 14 registers

    AcX = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H & ACCEL_XOUT_L
    AcY = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H & ACCEL_YOUT_L
    AcZ = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H & ACCEL_ZOUT_L
    Tmp = Wire.read() << 8 | Wire.read();  // TEMP_OUT_H & TEMP_OUT_L
    GyX = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H & GYRO_XOUT_L
    GyY = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H & GYRO_YOUT_L
    GyZ = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H & GYRO_ZOUT_L

    ax = convertRawAcceleration(AcX);
    ay = convertRawAcceleration(AcY);
    az = convertRawAcceleration(AcZ);
    gx = convertRawGyro(GyX);
    gy = convertRawGyro(GyY);
    gz = convertRawGyro(GyZ);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();

    keel_angle = PID(roll)+90; //+90 because the servo angle is 0-180 and PID calculates for -90 90

    Serial.print("Keel angle ");
    Serial.print(keel_angle);
    
    Serial.print(" Roll ");
    Serial.println(roll);

    writeServo(keel_angle);
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
  
  delay(3);
}

float convertRawAcceleration(float aRaw) {
  // since we are using 4 g range
  // -4 g maps to a raw value of -32768
  // +4 g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(float gRaw) {
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

float PID(float input){



  float sampletime = microsPerReading/1e6;

  input = input * 3.14159/180;
  //declrate controller values
  const float P = -0.4;
  const float I = 0;
  const float D = 0;
  const float N = 100; //filter coefficient
  const float desired_angle = 0;
  float filtereddError;
  float alpha = (N*sampletime)/(1+N*sampletime);

  float Error, dError; //iError declared global due to having to be saved
  float output;

  //calculate error
  Error = desired_angle - input;
  //calculate I
  iError += Error*sampletime;
  //calculate D
  dError = (Error-lastError)/sampletime;
  filtereddError = alpha*dError+(1-alpha)*prevFilteredError;
  //combine  
  output = P*Error + I*iError + D*filtereddError;
  //recalculate to angle
  output = output*180/3.14159;
  //save last error
  prevFilteredError=filteredError;
  lastError = Error; //lastError declared global due to having to be saved

  return output;
}

void writeServo(int angle) {
  // Map angle (0 to 180) to pulse width (500 to 2400 microseconds)
  int pulseWidth = map(angle, 0, 180, 500, 2400);

  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);

  // Wait the rest of the 20 ms cycle
  delay(20 - pulseWidth / 1000);  // compensate for pulse time
}