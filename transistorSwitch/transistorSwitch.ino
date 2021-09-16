// Define the pin you connected to the base of the transistor
#define BASE_PIN 12 

void setup(){
  pinMode(BASE_PIN, OUTPUT);
}

void loop(){
  // Activate the transistor applying 5V to the pin (HIGH)
  digitalWrite(BASE_PIN,HIGH);
  delay(1000);

  // Deactive the transitor by setting the pin to 0V (LOW)
  digitalWrite(BASE_PIN,LOW);
  delay(1000);
}
