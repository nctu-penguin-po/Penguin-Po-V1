#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <DHT.h>
#include <DHT_U.h>
#include "MS5837.h"
#include <LiquidCrystal_I2C.h>
#include "MPU9250.h"
#include <MadgwickAHRS.h>

Madgwick filter;
MPU9250 IMU(Wire,0x68);
LiquidCrystal_I2C lcd(0x3F, 20, 4);

#define STOP 41
#define SOSPIN1 42
#define SOSPIN2 43
#define SOSPIN3 44
#define SOSPIN4 45
#define SOSPIN5 46
#define SOSPIN6 47
#define DHTPIN1 48
#define DHTPIN2 49
#define DHTPIN3 50
#define DHTPIN4 51
#define DHTPIN5 52
#define DHTPIN6 53

int status;
int trigPin = 12;
int echoPin = 13;
long duration,cm;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
float temperature;
float DHTT1,DHTT2,DHTT3,DHTT4,DHTT5,DHTT6;
float DHTH1,DHTH2,DHTH3,DHTH4,DHTH5,DHTH6;
float BMPP1;
float WorkingT=45;
float WorkingH=80;
float WorkingP=102000;


#define DHTTYPE DHT11

MS5837 sensor;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(1);
DHT_Unified dht1 = DHT_Unified(DHTPIN1, DHTTYPE,6,1,1);
DHT_Unified dht2 = DHT_Unified(DHTPIN2, DHTTYPE,6,2,2);
DHT_Unified dht3 = DHT_Unified(DHTPIN3, DHTTYPE,6,3,3);
DHT_Unified dht4 = DHT_Unified(DHTPIN4, DHTTYPE,6,4,4);
DHT_Unified dht5 = DHT_Unified(DHTPIN5, DHTTYPE,6,5,5);
DHT_Unified dht6 = DHT_Unified(DHTPIN6, DHTTYPE,6,6,6);

uint32_t delayMS;

void setup(void) 
{
  pinMode(STOP,OUTPUT);
  pinMode(SOSPIN1, INPUT);
  pinMode(SOSPIN2, INPUT);
  pinMode(SOSPIN3, INPUT);
  pinMode(SOSPIN4, INPUT);
  pinMode(SOSPIN5, INPUT);
  pinMode(SOSPIN6, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);

  while(!Serial) {}

  status = IMU.begin();
  /*
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  */
  IMU.setSrd(19);
  filter.begin(25);
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
  
  lcd.begin();
  lcd.backlight();
  lcd.print("Hello, SAUVC");
  lcd.setCursor ( 0, 1 );
  lcd.print("We are team NCTU");
  lcd.setCursor ( 0, 2 );
  lcd.print("My name is PO");
  lcd.setCursor ( 0, 3 );
  lcd.print("I am a penguin");
  
  //Serial.println("Sensors Test"); Serial.println("");
  Wire.begin();
  
  while (!sensor.init()) {
    lcd.clear();
    lcd.setCursor ( 0, 1 );
    lcd.println("Init failed!");
    //lcd.println("Are SDA/SCL Connected Correctly?");
    //lcd.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_02BA);
  //sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  /*
  if(!bmp.begin())
  {
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  */
//-------------------------------------------------------------------------------
/*
  dht1.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();
  dht5.begin();
  dht6.begin();
  */
  delay(1000);
}

void loop(void) {
  delay(delayMS);
  
  IMU.readSensor();
  
  sensors_event_t event;
  //Serial.print("before read ");
  sensor.read();
  //Serial.print("after read ");
  float depth = sensor.depth();
  
  int leakState1 = digitalRead(SOSPIN1);
  int leakState2 = digitalRead(SOSPIN2);
  int leakState3 = digitalRead(SOSPIN3);
  int leakState4 = digitalRead(SOSPIN4);
  int leakState5 = digitalRead(SOSPIN5);
  int leakState6 = digitalRead(SOSPIN6);
  
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  
  unsigned long microsNow;
  
//--------------------------------------------------------------------
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (/*microsNow - microsPrevious >= microsPerReading*/1) {

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(IMU.getAccelX_mss());
    ay = convertRawAcceleration(IMU.getAccelY_mss());
    az = convertRawAcceleration(IMU.getAccelZ_mss());
    gx = convertRawGyro(IMU.getGyroX_rads());
    gy = convertRawGyro(IMU.getGyroY_rads());
    gz = convertRawGyro(IMU.getGyroZ_rads());

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(depth);
    Serial.print(" ");

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
    lcd.clear();
    lcd.setCursor ( 0, 0 );
    lcd.print(heading);
    lcd.setCursor ( 0, 1 );
    lcd.print(pitch);
    lcd.setCursor ( 0, 2 );
    lcd.print(roll);
    lcd.setCursor ( 0, 3 );
    lcd.print(depth);
  }
  Serial.print("\n");
  delay(5);
  return;
//--------------------------------------------------------------------
/*
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration/2) / 29.1;          // 29.1 in air ; 6.8 in water
*/
//--------------------------------------------------------------------
  dht1.temperature().getEvent(&event);
  DHTT1=event.temperature;
  dht1.humidity().getEvent(&event);
  DHTH1=event.relative_humidity;
  bmp.getEvent(&event);
  BMPP1=event.pressure;
  
  dht2.temperature().getEvent(&event);
  DHTT2=event.temperature;
  dht2.humidity().getEvent(&event);
  DHTH2=event.relative_humidity;

  dht3.temperature().getEvent(&event);
  DHTT3=event.temperature;
  dht3.humidity().getEvent(&event);
  DHTH3=event.relative_humidity;

  dht4.temperature().getEvent(&event);
  DHTT4=event.temperature;
  dht4.humidity().getEvent(&event);
  DHTH4=event.relative_humidity;

  dht5.temperature().getEvent(&event);
  DHTT5=event.temperature;
  dht5.humidity().getEvent(&event);
  DHTH5=event.relative_humidity;

  dht6.temperature().getEvent(&event);
  DHTT6=event.temperature;
  dht6.humidity().getEvent(&event);
  DHTH6=event.relative_humidity;
 
//--------------------------------------------------------------------
  if(isnan(DHTT1)||isnan(DHTH1)||isnan(BMPP1)/*||isnan(DHTT2)||isnan(DHTH2)||isnan(DHTT3)||isnan(DHTH3)||
  isnan(DHTT4)||isnan(DHTH4)*/||isnan(DHTT5)||isnan(DHTH5)||isnan(DHTT6)||isnan(DHTH6)){
    //Serial.println("Check Wiring");
    //Serial.println("------------------------------------");
    lcd.clear();
    lcd.setCursor ( 0, 0 );
    lcd.print("Check Wiring");
  }
  else{
    int DHTS1 = (DHTT1>WorkingT||DHTH1>WorkingH||BMPP1>WorkingP) ? 1 : 0;
    //int DHTS2 = (DHTT2>WorkingT||DHTH2>WorkingH) ? 1 : 0;
    //int DHTS3 = (DHTT3>WorkingT||DHTH3>WorkingH) ? 1 : 0;
    //int DHTS4 = (DHTT4>WorkingT||DHTH4>WorkingH) ? 1 : 0;
    int DHTS5 = (DHTT5>WorkingT||DHTH5>WorkingH) ? 1 : 0;
    int DHTS6 = (DHTT6>WorkingT||DHTH6>WorkingH) ? 1 : 0;

    int DHTStates = 0;
    DHTStates |= (DHTS1 << 1);
    //DHTStates |= (DHTS2 << 2);
    //DHTStates |= (DHTS3 << 3);
    //DHTStates |= (DHTS4 << 4);
    DHTStates |= (DHTS5 << 5);
    DHTStates |= (DHTS6 << 6);

    Serial.print(DHTStates);
    Serial.print(" ");

    int ALLleakState = 0;
    ALLleakState |= (leakState1 << 1);
    ALLleakState |= (leakState2 << 2);
    ALLleakState |= (leakState3 << 3);
    ALLleakState |= (leakState4 << 4);
    ALLleakState |= (leakState5 << 5);
    ALLleakState |= (leakState6 << 6);

    Serial.print(ALLleakState);
    Serial.print(" ");
    
    if(DHTT1>WorkingT||DHTH1>WorkingH||BMPP1>WorkingP||leakState1==HIGH){
      //Serial.println("Main Pod Fail");
      //Serial.println("------------------------------------");
      lcd.clear();
      lcd.setCursor ( 0, 0 );
      lcd.print("Main Pod Fail");
      digitalWrite(STOP, HIGH);
      //while(1) {}
    }
    /*
    else if(DHTT2>WorkingT||DHTH2>WorkingH||leakState2==HIGH){
      //Serial.println("Camara L Fail");
      //Serial.println("------------------------------------");
      lcd.clear();
      lcd.setCursor ( 0, 0 );
      lcd.print("Camara L Fail");
      digitalWrite(STOP, HIGH);
      //while(1) {}
    }
    else if(DHTT3>WorkingT||DHTH3>WorkingH||leakState3==HIGH){
      //Serial.println("Camara R Fail");
      //Serial.println("------------------------------------");
      lcd.clear();
      lcd.setCursor ( 0, 0 );
      lcd.print("Camara R Fail");
      digitalWrite(STOP, HIGH);
      //while(1) {}
    }
    else if(DHTT4>WorkingT||DHTH4>WorkingH||leakState4==HIGH){
      //Serial.println("Camara D Fail");
      //Serial.println("------------------------------------");
      lcd.clear();
      lcd.setCursor ( 0, 0 );
      lcd.print("Camara D Fail");
      digitalWrite(STOP, HIGH);
      while(1) {}
    }*/
    else if(DHTT5>WorkingT||DHTH5>WorkingH||leakState5==HIGH){
      //Serial.println("Battery L Fail");
      //Serial.println("------------------------------------");
      lcd.clear();
      lcd.setCursor ( 0, 0 );
      lcd.print("Battery L Fail");
      digitalWrite(STOP, HIGH);
      //while(1) {}
    }
    else if(DHTT6>WorkingT||DHTH6>WorkingH||leakState6==HIGH){
      //Serial.println("Battery R Fail");
      //Serial.println("------------------------------------");
      lcd.clear();
      lcd.setCursor ( 0, 0 );
      lcd.print("Battery R Fail");
      digitalWrite(STOP, HIGH);
      //while(1) {}
    }
    
    else{
      //Serial.println("All System Well");
      //Serial.println("------------------------------------");
      lcd.clear();
      lcd.setCursor ( 0, 0 );
      lcd.print("All System Well");
    }
  }
  Serial.println();
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 8.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 500.0) / 32768.0;
  return g;
}
