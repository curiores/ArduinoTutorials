template <int order> // order is 2 or...
class BandPass
{
  private:
    float a[order+1];
    float b[order+1];
    float omega0;
    float Q;
    float domega;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    BandPass(float f0, float fw, float fs, bool adaptive){
      // f0: central frequency (Hz)
      // fw: bandpass width (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      omega0 = 6.28318530718*f0;
      domega = 6.28318530718*fw;
      Q = omega0/domega;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;      
        a[k] = 0;
        b[k] = 0;  
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      float alpha = omega0*dt;
      if(order==2){
        float D = pow(alpha,2) + 2*alpha/Q + 4;
        b[0] = 2*alpha/(Q*D);
        b[1] = 0;
        b[2] = -b[0];
        a[0] = 0;
        a[1] = -(2*pow(alpha,2) - 8)/D;
        a[2] = -(pow(alpha,2) - 2*alpha/Q + 4)/D;   
      }
      else if(order==4){

      }
    }
    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order+1; k++){
        y[0] += a[k]*y[k] + b[k]*x[k];
      }

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
BandPass<2> lp(30,10,1e3,true);

void setup() {
  Serial.begin(115200);

}

void loop() {
  // Read pin A0 and compute current in mA
  // -- replace these two lines with your sensor readings --
  float t = millis()/1000.0;
  float xn = sin(2*PI*2*t) + cos(2*PI*30*t) + cos(2*PI*120*t);

  // Compute the filtered signal
  float yn = lp.filt(xn);

  // Output
  Serial.print(xn);
  Serial.print(" ");
  Serial.print(yn);
  Serial.println();
 
}
