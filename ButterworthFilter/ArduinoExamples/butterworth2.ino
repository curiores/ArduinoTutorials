// This file shows an example implementation of a 
// second order low-pass Butterworth filter on an Arduino. 
// Note that there are many possible improvements to this code.

float x[] = {0,0,0};
float y[] = {0,0,0};
int k = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Test signal
  float t = micros()/1.0e6;
  x[0] = sin(2*PI*2*t) + 0.5*sin(2*PI*25*t);

  // Compute the filtered signal
  // (second order Butterworth example)
  float b[] = {0.00024132, 0.00048264, 0.00024132};
  float a[] = {1.95558189, -0.95654717};
  y[0] = a[0]*y[1] + a[1]*y[2] +
               b[0]*x[0] + b[1]*x[1] + b[2]*x[2];

  if(k % 3 ==0)
  {
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency
    
    // For the serial monitor
    Serial.print(2*x[0]);
    Serial.print(" ");
    Serial.println(2*y[0]);
  }

  delay(1); // Wait 1ms
  for(int i = 1; i >= 0; i--){
    x[i+1] = x[i]; // store xi
    y[i+1] = y[i]; // store yi
  }
  
  k = k+1;
}
