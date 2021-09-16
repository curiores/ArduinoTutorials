// This file shows an example implementation of a low-pass filter on an Arduino.
// Note that there are many potential improvements to this code.

float xn1 = 0;
float yn1 = 0;
int k = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Test signal
  float t = micros()/1.0e6;
  float xn = sin(2*PI*2*t) + 0.2*sin(2*PI*50*t);

  // Compute the filtered signal
  float yn = 0.969*yn1 + 0.0155*xn + 0.0155*xn1;

  delay(1);
  xn1 = xn;
  yn1 = yn;

  if(k % 3 == 0){
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency
  
    // Output
    Serial.print(2*xn);
    Serial.print(" ");
    Serial.println(2*yn);
  }
  k = k+1;
}
