template <int order> // order is 1 or 2
class HighPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    HighPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
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
      if(order==1){
        float alphaFactor = 1/(1 + alpha/2.0);
        a[0] = -(alpha/2.0 - 1)*alphaFactor;
        b[0] = alphaFactor;
        b[1] = -alphaFactor;      
      }
      if(order==2){
        float alpha = omega0*dt;
        float dtSq = dt*dt;
        float c[] = {omega0*omega0, sqrt(2)*omega0, 1};
        float D = c[0]*dtSq + 2*c[1]*dt + 4*c[2];
        b[0] = 4.0/D;
        b[1] = -8.0/D;
        b[2] = 4.0/D;
        a[0] = -(2*c[0]*dtSq - 8*c[2])/D;
        a[1] = -(c[0]*dtSq - 2*c[1]*dt + 4*c[2])/D;   
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
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

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
HighPass<2> lp(20,1e3,true);

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read pin A0 and compute current in mA
  // -- replace these two lines with your sensor readings --
  float t = millis()/1000.0;
  float xn = sin(2*PI*2*t) + 0.5*cos(2*PI*50*t);

  // Compute the filtered signal
  float yn = lp.filt(xn);

  // Output
  Serial.print(xn);
  Serial.print(" ");
  Serial.print(yn);
  Serial.println();
}
