#include <Servo.h>
#include <TimerOne.h>
#include <ads7843.h>

#define DCLK     2
#define CS       3  
#define DIN      4 
#define DOUT     6
#define IRQ      5 

#define sampling 15.0f 

ADS7843 touch(CS, DCLK, DIN, DOUT, IRQ);
Point p;

Servo servo1;
Servo servo2;

float angle1;
float angle2;

float pointX;
float pointY;

float previous_error1;
float integral1;
float setpoint1;
float measured_value1;
float derivative1;
float error1;
float output1;

float previous_error2;
float integral2;
float setpoint2;
float measured_value2;
float derivative2;
float error2;
float output2;

float Kp = 15;
float Ki = 0;
float Kd = 4.5;


void setup() {
  servo1.attach(9); 
  servo2.attach(8); 
  angle1 = 90;
  angle2 = 90;

  pointX = 2.0;
  pointY = 2.0f;

  setpoint1 = pointX;
  setpoint2 = pointY;
  
  Serial.begin(115200);
  touch.begin();
 
  servo1.write(90);
  servo2.write(90);
  
  // Timer1.initialize(sampling); // sampling rate 1 kHz
  // Timer1.attachInterrupt( PID );

  
}

void loop() {

  uint32_t flag;

  p=touch.getpos(&flag) ;
  
  if(flag) {
   /** get position successfully */
   
   /** print the coordinate */
//   Serial.print(p.x, DEC);
//   Serial.print("   ");
//   Serial.print(p.y, DEC);
//   Serial.println();
  
    pointX = p.x;
    pointY = p.y;

    if(pointX == 0 && pointY == 0) {
       return; 
    }

    error1 = setpoint1 - (pointX / 1000.0f);
    integral1 = integral1 + error1*sampling;
    derivative1 = (error1 - previous_error1)/0.015f;
    output1 = Kp*error1 + Ki*integral1 + Kd*derivative1;
    previous_error1 = error1;

    output1 = output1 + 90;
    
    angle1 = output1;
    
    if (angle1 < 30) {
      angle1 = 30;
    }
    if (angle1 > 150) {
      angle1 = 150;
    } 
    
  //  angle1 = map(output, -1800, 1800, 30, 150);
    angle1 = 180.0f - angle1;


    error2 = setpoint2 - (pointY / 1000.0f);
    integral2 = integral2 + error2*0.015f;
    derivative2 = (error2 - previous_error2)/0.015f;
    output2 = Kp*error2 + Ki*integral2 + Kd*derivative2;
    previous_error2 = error2;

    output2 = output2 + 90;
    
    angle2 = output2;
    
    if (angle2 < 30) {
      angle2 = 30;
    }
    if (angle2 > 150) {
      angle2 = 150;
    } 
    
  //  angle1 = map(output, -1800, 1800, 30, 150);
    angle2 = 180.0f - angle2;



//    Serial.println("integral");     
//    Serial.println(integral); 

    // set the servo position  
    
//    if (angle1 > 85 && angle1 < 95) {
//      angle1 = 90;
//    }
//
//    if (angle2 > 85 && angle2 < 95) {
//      angle2 = 90;
//    }

    
    Serial.println("angle");     
    Serial.print(angle1);     
    Serial.print("   ");     
    Serial.print(angle2); 

    Serial.print("derivative1: ");
    Serial.print(derivative1);
    Serial.print("   ");     

    servo1.write((int)angle1);
    servo2.write((int)angle2);
    // servo2.write(angle2);
  
    // wait for the servo to get there 
    delay(sampling);
   
 } else {
   integral1 = 0;
   integral2 = 0;
 }
 
 
}
