#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include <ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int16.h>
//#include <std_msgs/Float32.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Int32MultiArray.h>

// called this way, it uses the default address 0x40

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int motor_data[8];
//ros::NodeHandle  nh;
//std_msgs::Int32MultiArray ;

/*
void motor_cb( const std_msgs::Int32MultiArray& msg_motor){
   //motor_data=msg_motor;
   //int i = 0;
  // print all the remaining numbers
  for(int i=0; i<8;i++)
  {
    motor_data[i] = msg_motor.data[i];
  }
  return;
}
*/

//ros::Subscriber<std_msgs::Int32MultiArray> sub1("motor", motor_cb);
int state;

void setup() {
  for(int i=0; i<8;i++)
  {
    motor_data[i] = 1500;
    
  }
  //nh.initNode();
  //nh.subscribe(sub1);
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);
  
  pwm.begin();
  pwm.setPWMFreq(52);  // Analog servos run at ~60 Hz updates
  
  state = -1;

  for (int i=0; i<8; i++){
    int tt = map(1500, 0, 20000, 0, 4096);
    pwm.setPWM(i, 0, tt);
  }
  
  delay(10);
}

/*motor_num ahead_right:3 
  */
void loop() {
  //Serial.println(motor_data[0]);
  if(Serial.available()){
    if (state == -1){
      unsigned char c = Serial.read();
      if ((int)c == 50) state = 0;
      int t = (int)c;
      //Serial.println(t);
    }
    else{
      unsigned char c = Serial.read();
      int t = (int)c;
      //Serial.print(t);
      if (t == 50){
        state = 0;
        return;
      }
      motor_data[state] = 10*t;
      int tt = map(motor_data[state], 0, 20000, 0, 4096);
      pwm.setPWM(state, 0, tt);
      state++;
      if (state == 8) state = 0;
      //Serial.println(motor_data[0]);
    }
    if(motor_data[0] > 1500) digitalWrite(13, 1);
    else digitalWrite(13, 0);
  }
  /*
  for (int i=0; i<8; i++){
    int t = map(motor_data[i], 0, 20000, 0, 4096);
    //pwm.setPWM(i, 0, t);
  }
  if(motor_data[0] > 1500) digitalWrite(13, 1);
  else digitalWrite(13, 0);
  delay(10);

  */
  
  /*if(depth_1==0){
    if(depth.data < 0.5){
      pwm.setPWM(4, 0, 1700);
      pwm.setPWM(5, 0, 1700);
      pwm.setPWM(6, 0, 1700);
      pwm.setPWM(7, 0, 1700);  
    }
   else{
     depth_1 = 1;
      pwm.setPWM(4, 0, 1500);
      pwm.setPWM(5, 0, 1500);
      pwm.setPWM(6, 0, 1500);
      pwm.setPWM(7, 0, 1500); 
    }
  }
 
    //depthING
    if(depth.data < 0.5){
      //row right
      if(posture.data(1) < 0){
        //pitch down
        if(posture.data(2) < 0){
          pwm.setPWM(2, 0, 1600); 
          pwm.setPWM(3, 0, 1600);          
          pwm.setPWM(4, 0, 1500);
          pwm.setPWM(5, 0, 1600);
          pwm.setPWM(6, 0, 1400);
          pwm.setPWM(7, 0, 1500);
          
        }
        //pitch up
        else{
          pwm.setPWM(2, 0, 1600); 
          pwm.setPWM(3, 0, 1600);          
          pwm.setPWM(4, 0, 1700);
          pwm.setPWM(5, 0, 1600);
          pwm.setPWM(6, 0, 1800);
          pwm.setPWM(7, 0, 1700);         
        }
      }

    }*/
    
  
  // Drive each servo one at a time
 /* Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);

  servonum ++;
  if (servonum > 7) servonum = 0;*/
  
}
