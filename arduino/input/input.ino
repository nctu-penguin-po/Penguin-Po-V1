#include <Wire.h>
#include "MS5837.h"
#include "MPU9250.h"
#include <MadgwickAHRS.h>
#include <math.h>

//#define PI 3.1415926535898

MS5837 sensor;
Madgwick filter;
MPU9250 IMU(Wire,0x68);
int status;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  status = IMU.begin();
  if (status < 0) {
    while(1) {}
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(19);
  filter.begin(5);
  
  microsPerReading = 100;
  microsPrevious = millis();
}

void loop(){
  int V=analogRead(A0);
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float mangle;
  float vol=V*(5.0/1023.0);
  float roll, pitch, yaw;
  unsigned long microsNow;
  
  IMU.readSensor();
  sensor.read();
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  mangle = atan(my/mx)*180/PI;
  //mangle = (my < 0) ? mangle+180 : (mx < 0) ? mangle+360 : mangle;
  
  microsNow = millis();
  if (microsNow - microsPrevious >= microsPerReading) {
    microsPrevious = microsPrevious + microsPerReading;
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    
    Serial.print(vol,1);
    Serial.print(" ");
    Serial.print(sensor.depth(),3); 
    Serial.print(" ");
    Serial.print(sensor.temperature(),0);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.println(IMU.getTemperature_C(),0);
    //Serial.print(" ");
    /*
    Serial.print(mangle);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.println(mz);
    */
  }
  
}
