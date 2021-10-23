#include <Servo.h> // Servo library

// These pins are for the pump motor
#define PWM 3
#define IN1 4
#define IN2 5

// These are the servo pins on the valves
#define SERVO1 9
#define SERVO2 11
#define SERVO3 10

// The three valve servo objects
Servo servo1;  
Servo servo2;  
Servo servo3;  

void setup() {
  // Attach the servo objects
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
}

void loop() {
  // This code is simply the demo used in the video
  // You would want to modify this code to suit your particular purposes
  // For example, add a moisture sensor and some conditional statements.
  
  // Turn on the pump
  int pwm = 255;
  int dir = -1;
  setMotor(dir,pwm,PWM,IN1,IN2);

  // Switch valves on and off by rotating the servos between 0 and 180
  int td =1000; // how long to wait between switching the valves

  servo1.write(0);
  servo2.write(180);
  servo3.write(0);
  delay(td);   

  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  delay(td);   
 
  servo1.write(180);
  servo2.write(0);
  servo3.write(0);
  delay(td); 

  servo1.write(180);
  servo2.write(0);
  servo3.write(180);
  delay(td*2); 

  servo1.write(180);
  servo2.write(0);
  servo3.write(0);
  delay(td); 
  
  servo1.write(180);
  servo2.write(180);
  servo3.write(0);
  delay(td);   

  servo1.write(0);
  servo2.write(180);
  servo3.write(0);
  delay(td);   

}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}
