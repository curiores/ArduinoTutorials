#define led_pin 8

void setup(){
  pinMode(led_pin,OUTPUT);
  
}

void loop(){
  digitalWrite(led_pin,HIGH); // turn the LED on
  delay(1000); // wait a second

  digitalWrite(led_pin,LOW)); // turn the LED off
  delay(1000); // wait a second  
  
}
