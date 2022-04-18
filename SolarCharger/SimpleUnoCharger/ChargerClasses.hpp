
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
};
