#include <TimerOne.h>
#include <Adafruit_INA219.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ChargerClasses.hpp"

// Current sensor
Adafruit_INA219 ina219;

// Charge control
#define PFET_PIN 11 // TimerOne Uno pin: 9, TimerOne Mega pin: 11
SimplePID voltagePID;
bool charging = true;

// SD card data recording
#define CHIP_SELECT 53 // 53 for mega, 10 for uno/nano
#define RECINTERVAL 10 // Seconds between recordings
File dataFile;
String fName = "data0.txt";
float t = 0;
float tn1 = 0;
float t_record = 0;

// Display
Adafruit_SSD1306 display(128,64,&Wire,-1);

// Temperature sensors
#define ONE_WIRE_BUS 22 // Battery temp pin
#define ONE_WIRE_BUS_AMB 23 // Ambient temp pin
OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWireAmb(ONE_WIRE_BUS_AMB);
DallasTemperature tempSensor(&oneWire);
DallasTemperature tempSensorAmb(&oneWireAmb);

// Voltage reading/filtering using interrupts
#define INTERRUPT_PIN 18
#define PWM_INTERRUPT 6
volatile float vA0Filt = 0;
volatile float vA1Filt = 0;
volatile LowPass<2> lpVA0(1,490,false);
volatile LowPass<2> lpVA1(1,490,false);

void voltageReadings(){
  // This function is called by the interrupt
  // Take voltage readings and filter
  float vA0 = analogRead(A0);
  vA0Filt = lpVA0.filt(vA0);
  float vA1 = analogRead(A1);
  vA1Filt = lpVA1.filt(vA1); 
}

void setup() {

  // Basic setup
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  Serial.begin(9600);
  SPI.begin();

  // Current sensor
  ina219.begin();

  // Timer 1 pin
  Timer1.initialize(100); // 100 us => 10 kHz
  Timer1.pwm(PFET_PIN, 0); // Start with 0 duty 

  // Charging control
  voltagePID.setParams(0,0.2e-3,0,0,1); // PID parameters
  charging = true; // Charging flag

  // SD card
  if(!SD.begin(CHIP_SELECT)){
    Serial.println("SD card init failed.");
  }

  // Temp sensor
  tempSensor.begin();
  tempSensorAmb.begin();

  // SD card
  dataFile = SD.open(fName,FILE_WRITE);
  if(dataFile){
    dataFile.println("t,iTarget,iBatt,vSrc,vBatt,tempF,tempAF,duty");
    Serial.println("Wrote header to data file: " + fName);
  }
  
  // Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("Display SSD1306 FAILED to initialize");
  }
  display.display();
  display.clearDisplay();

  // Voltage readings/filtering using interrupts
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN),voltageReadings,RISING);
  analogWrite(PWM_INTERRUPT,100);
  
  delay(1000);
}

void loop() {

  // New time stamp
  float tn = millis()/1.0e3;
  t = t + (tn-tn1);
  tn1 = tn;

  // Current reading
  float iBatt = ina219.getCurrent_mA();

  // Voltage measurement
  noInterrupts();
  float vSrc = vA0Filt*5.0/1023.0*(5+1)/1;
  float vBatt = vA1Filt*5.0/1023.0*(5+1)/1;
  interrupts();
  
  // Pull last temperature reading
  float tempC = tempSensor.getTempCByIndex(0);
  float tempF = tempC*9.0/5.0 + 32.0;

  float tempAC = tempSensorAmb.getTempCByIndex(0);
  float tempAF = tempAC*9.0/5.0 + 32.0;
  
  // Charge control
  // PID current control
  float targetCurrent = 500;
  float u = voltagePID.evaluuff(iBatt, targetCurrent, vSrc);
  float duty = u; // The duty cycle equals the control signal

  // Stop charging if battery temp is over 105 F
  if(tempF > 105){
    charging = false;
  }
  // <add additional stop conditions here>  
  if(!charging){
    duty = 0;
  }
  // Set the duty cycle
  Timer1.pwm(PFET_PIN, duty*1023); 

  // Print the result to the serial port
  Serial.print(t);
  Serial.print(" ");
  Serial.print(targetCurrent);
  Serial.print(" ");
  Serial.print(iBatt);
  Serial.print(" ");
  Serial.print(vSrc);
  Serial.print(" ");
  Serial.print(vBatt);
  Serial.print(" ");
  Serial.print(tempF);
  Serial.print(" ");
  Serial.print(tempAF);
  Serial.print(" ");
  Serial.print(duty*100);
  Serial.println();

  // Record the data on the SD card
  if(fabs(t-t_record)> RECINTERVAL){

    // Only pull the temperature readings here
    // because they take a long time
    tempSensor.requestTemperatures();
    tempSensorAmb.requestTemperatures();

    dataFile = SD.open(fName,FILE_WRITE);
    if(dataFile){

      // Write to the data file
      dataFile.print(t);
      dataFile.print(",");
      dataFile.print(targetCurrent);
      dataFile.print(",");
      dataFile.print(iBatt);
      dataFile.print(",");
      dataFile.print(vSrc);
      dataFile.print(",");
      dataFile.print(vBatt);
      dataFile.print(",");
      dataFile.print(tempF);
      dataFile.print(",");
      dataFile.print(tempAF);
      dataFile.print(",");
      dataFile.print(duty*100);
      dataFile.println(" ");
      Serial.println("--------------Printed data--------------");
    }
    else{
      Serial.println("66666666 Failed to write data 666666666666");
    }
    t_record = t;
    dataFile.close();
  }


  // Write to the display 
  display.clearDisplay(); // Clear the display
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  String current = "i: ";
  current = current + round(iBatt) + "mA";
  display.println(current);
  String voltage = "v: ";
  voltage = voltage + vBatt + " V";
  display.println(voltage);
  String tf = "T: ";
  tf = tf + round(tempF) + " F";
  display.println(tf);
  String af = "AT: ";
  af = af + round(tempAF) + " F";
  display.println(af);
  display.display(); // Display the text  
  
  
} 
