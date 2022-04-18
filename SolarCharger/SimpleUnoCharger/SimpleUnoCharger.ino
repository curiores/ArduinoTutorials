#include <TimerOne.h>
#include <Adafruit_INA219.h>
#include <Wire.h>
#include "ChargerClasses.hpp"

// Current sensor
Adafruit_INA219 ina219;

// Charge control
#define NPN_PIN 9 
SimplePID voltagePID;
bool charging = true;

void setup() {

  // Basic setup
  Serial.begin(115200);
  pinMode(A0,INPUT);

  // Current sensor
  ina219.begin();

  // Timer 1 pin
  Timer1.initialize(100); // 100 us => 10 kHz
  Timer1.pwm(NPN_PIN, 0); // Start with 0 duty 

  // Charging control
  voltagePID.setParams(0,0.2e-3,0,0,1); // PID parameters
  
}

void loop() {

  // Current reading
  float iBatt = ina219.getCurrent_mA();

  // Voltage measurement
  float vBatt = ina219.getBusVoltage_V() - ina219.getShuntVoltage_mV()/1000;
  float vA0 = analogRead(A0);
  float vSrc = vA0*5.0/1023.0*(5+1)/1;

  // Charge control
  // PID current control
  float targetCurrent = 500;
  float u = voltagePID.evalu(iBatt, targetCurrent);
  float duty = u; // The duty cycle equals the control signal
  duty = 0.5;

  // Set the duty cycle
  Timer1.pwm(NPN_PIN, duty*1023); 

  // Print the result to the serial port
  Serial.print(targetCurrent);
  Serial.print(" ");
  Serial.print(iBatt);
  Serial.print(" ");
  Serial.print(vSrc);
  Serial.print(" ");
  Serial.print(vBatt);
  Serial.print(" ");
  Serial.print(duty*100);
  Serial.println();
  
} 
