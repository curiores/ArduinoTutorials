// Libraries
#include <Servo.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <HCSR04.h> // Include the HCSR04 header

// Define pins
#define PAN_SERVO 6
#define TILT_SERVO 5
#define TRIG 3
#define ECHO 2

// Globals
int panLim[] = {0,180};
int tiltLim[] = {0,90};

int pan = panLim[0];
int tilt = tiltLim[0];

int panIncrement = 1;
int tiltIncrement = 1;

Servo panServo;
Servo tiltServo;
VL53L1X tofSensor;
UltraSonicDistanceSensor ultrasonicSensor(TRIG, ECHO);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  panServo.attach(PAN_SERVO); 
  tiltServo.attach(TILT_SERVO); 
  panServo.write(pan);
  tiltServo.write(tilt);
  delay(1000);

  tofSensor.setTimeout(500);
  if (!tofSensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
  }
  tofSensor.setDistanceMode(VL53L1X::Long);
  tofSensor.setMeasurementTimingBudget(50000);
  tofSensor.startContinuous(50);

  // Write header
  Serial.print("Time(s), ");
  Serial.print("Pan(degree), ");
  Serial.print("Tilt(degree), ");
  Serial.print("Ultrasonic(mm), ");
  Serial.print("ToF(mm)");
  Serial.println();

}

void loop() {
  
  // Move to new position
  pan = pan + panIncrement;
  panServo.write(pan);  
  if(pan > panLim[1] || pan < panLim[0]){
    panIncrement = -panIncrement;
    tilt = tilt + tiltIncrement;
    tiltServo.write(tilt); 
    if(tilt < tiltLim[0] || tilt > tiltLim[1] ){
      tiltIncrement = -tiltIncrement;
    }
  }
  delay(200); // give a little time to move

  // Current time
  float t = ((float) millis())/1000.0;

  // Ultrasonic
  float usDist = ultrasonicSensor.measureDistanceCm()*10.0; 

  // TOF
  tofSensor.read();
  float tofDist = tofSensor.ranging_data.range_mm;

  // Print result
  Serial.print(t);
  Serial.print(" ");
  Serial.print(pan);
  Serial.print(" ");
  Serial.print(tilt);
  Serial.print(" ");
  Serial.print(usDist);
  Serial.print(" ");
  Serial.print(tofDist);
  Serial.print(" ");
  Serial.println();

}
