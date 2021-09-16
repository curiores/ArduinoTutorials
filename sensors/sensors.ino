// Libraries
#include <DHT.h>
#include <Ultrasonic.h>
#include <MPU6050_tockn.h>
#include <Wire.h> 

// Sensor pins
#define DHTPIN 13
#define SR04_TRIGGER_PIN 4
#define SR04_ECHO_PIN 3
#define SR501_PIN 7

// Sensor definitions
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE);
Ultrasonic ultrasonic(SR04_TRIGGER_PIN, SR04_ECHO_PIN);
MPU6050 mpu6050(Wire);

void setup() {
  // Set up serial communication
  Serial.begin(9600);

  // Set up sensors
  dht.begin();
  pinMode(SR501_PIN,INPUT);
  Wire.begin();
  mpu6050.begin();
  
  // Legend
  Serial.println(" Humidity(%) temp(C) distance(cm) motion100 anglex angley anglez ");

}

void loop() {
  delay(100); // Take a reading every 0.1 seconds

  // DHT sensor
  float humidity = dht.readHumidity();
  float tempC = dht.readTemperature();
  Serial.print(humidity);
  Serial.print(" ");
  Serial.print(tempC);
  Serial.print(" ");

  // HC-SR04
  float distance = ultrasonic.read();
  Serial.print(distance);
  Serial.print(" ");

  // HC-SR501
  int motionValue = digitalRead(SR501_PIN);
  Serial.print(motionValue*100);
  Serial.print(" ");

  // MPU 6050
  mpu6050.update();
  Serial.print(mpu6050.getAngleX());
  Serial.print(" ");
  Serial.print(mpu6050.getAngleY());
  Serial.print(" ");
  Serial.print(mpu6050.getAngleZ());
  Serial.print(" ");
    
  Serial.print("\n");

}
