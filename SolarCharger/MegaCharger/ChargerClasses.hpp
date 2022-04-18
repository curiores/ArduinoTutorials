
// A class to compute the control signal
class SimplePID{
  private:
    float kp, ki, kd; // Parameters
    float eprev, eintegral, tn1; // Storage
    float umin, umax; // Anti-wind up clamp limits

  public:
  // Constructor
  SimplePID() : kp(1), ki(0), kd(0), eprev(0.0),
                eintegral(0.0), tn1(0.0), umin(-1), umax(1){}

  // A function to set the parameters
  void setParams(float kpIn, float kiIn, float kdIn, float uminIn, float umaxIn){
    kp = kpIn;
    ki = kiIn;
    kd = kdIn;
    umin = uminIn;
    umax = umaxIn;
  }

  // A function to compute the control signal
  float evalu(float value, float target){
    // Compute time step
    float t = micros()/1.0e6;
    float dt = t - tn1;
    tn1 = t; // Store previous time

    // Compute error
    float e = target - value;
    float dedt = (e-eprev)/(dt);
    eprev = e; // store previous error

    float eintegralUpdate = eintegral + e*dt;
    // Evaluate the control signal
    float u = kp*e + ki*eintegralUpdate+ kd*dedt;

    // Anti-wind up clamping
    // If either of the first two conditions are true
    // The integral is not updated
    if( u < umin ){
      u = umin;
    }
    else if( u > umax ){
      u = umax;
    }
    else{
      // Otherwise update the integral
      eintegral = eintegralUpdate;
    }

    return u;
  }

 float evaluuff(float value, float target, float vSrc){
    // This is the same as evalu
    // but with feedforward control
    // The feedforward values should be converted to inputs
  
    // Compute time step
    float t = micros()/1.0e6;
    float dt = t - tn1;
    tn1 = t; // Store previous time

    // Compute error
    float e = target - value;
    float dedt = (e-eprev)/(dt);
    eprev = e; // store previous error

    float eintegralUpdate = eintegral + e*dt;
    // Evaluate the control signal
    float u = kp*e + ki*eintegralUpdate+ kd*dedt;

    // Feedforward controller
    float targetVoltage = 13;
    float uff = (-0.5/targetVoltage)*vSrc + 1.5;
    if( uff < 0){
      uff = 0;
    }

    u = u + uff;
  
    // Anti-wind up clamping
    // If either of the first two conditions are true
    // The integral is not updated
    if( u < umin ){
      u = umin;
    }
    else if( u > umax ){
      u = umax;
    }
    else{
      // Otherwise update the integral
      eintegral = eintegralUpdate;
    }

    return u;
  }
};



// Low pass filter class
template <int order> // order is 1 or 2
class LowPass
{
  private:
    volatile float a[order];
    volatile float b[order+1];
    volatile float omega0;
    volatile float dt;
    volatile bool adapt;
    volatile float tn1 = 0;
    volatile float x[order+1]; // Raw values
    volatile float y[order+1]; // Filtered values

  public:
    LowPass(float f0, float fs, bool adaptive){
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
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);
      }
      if(order==2){
        float c1 = 2*sqrt(2)/alpha;
        float c2 = 4/(alpha*alpha);
        float denom = 1.0+c1+c2;
        b[0] = 1.0/denom;
        b[1] = 2.0/denom;
        b[2] = b[0];
        a[0] = -(2.0-2.0*c2)/denom;
        a[1] = -(1.0-c1+c2)/(1.0+c1+c2);
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
