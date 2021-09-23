// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

#include <Wire.h>
#include <math.h>

/*------------ CLASS ------------*/
class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;
    
  public:
    // Default initialization list
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    
    // Set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Evaluate the signal
    void evalu(int value, int target, float deltaT,int &pwr, int &dir){
        
      // error
      int e = target - value;
      
      float dedt = (e-eprev)/(deltaT);
      eintegral = eintegral + e*deltaT;
      float u = kp*e + kd*dedt + ki*eintegral;
    
      // motor power
      pwr = (int) fabs(u);
      if( pwr > umax ){
        pwr = umax;
      }
           
      // motor direction
      dir = 1;
      if(u<0){
        dir = -1;
      }
            
      // store previous error
      eprev = e;
    }
    
};

/*------------ GLOBALS AND DEFINITIONS ------------*/
// Define the motors
#define NMOTORS 2
#define M0 0
#define M1 1
const int enca[] = {0,1};
const int encb[]= {4,5};
const int pwm[] = {9,13};
const int in1[] = {8,11};
const int in2[] = {10,12};

// Global variables
long prevT = 0;
int posPrev[] = {0,0};

// positions
volatile int posi[] = {0,0};

// PID classes
SimplePID pid[NMOTORS];


/*------------ FUNCTIONS ------------*/
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}

void sendLong(long value){
  for(int k=0; k<4; k++){
    byte out = (value >> 8*(3-k)) & 0xFF;
    Wire.write(out);
  }
}

long receiveLong(){
  long outValue;
  for(int k=0; k<4; k++){
    byte nextByte = Wire.read();
    outValue = (outValue << 8) | nextByte;
  }
  return outValue;
}

// targets
float target_f[] = {0,0,0,0};
long target[] = {0,0,0,0};

void setTarget(float t, float deltat){

  float positionChange[4] = {0.0,0.0,0.0,0.0};
  float pulsesPerTurn = 16*18.75; 
  float pulsesPerMeter = pulsesPerTurn*3.5368;

  t = fmod(t,12); // time is in seconds
  float velocity = 0.25; // m/s

  if(t < 4){
  }
  else if(t < 8){
    for(int k = 0; k < 4; k++){ 
      positionChange[k] = velocity*deltat*pulsesPerMeter;
    }
  }
  else{
    for(int k = 0; k < 4; k++){ 
      positionChange[k] = -velocity*deltat*pulsesPerMeter; 
    } 
  }  

  for(int k = 0; k < 4; k++){
    target_f[k] = target_f[k] + positionChange[k];
  }
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
  target[2] = (long) -target_f[2];
  target[3] = (long) -target_f[3];
}

/*------------ SETUP ------------*/
void setup() {
  Wire.begin();        // join i2c bus
  Serial.begin(9600);
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pid[k].setParams(1,0.1,0,255);
  }
  attachInterrupt(digitalPinToInterrupt(enca[M0]),readEncoder<M0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]),readEncoder<M1>,RISING);
  
}

/*------------ LOOP ------------*/
void loop() {

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
    
  // set target position
  setTarget(currT/1.0e6,deltaT);
  
  // Send the requested position to the follower
  Wire.beginTransmission(1); 
  sendLong(target[2]);
  sendLong(target[3]);
  Wire.endTransmission();

  // Get the current position from the follower
  long pos[4];
  Wire.requestFrom(1, 8);    
  pos[2] = receiveLong();
  pos[3] = receiveLong();

  // Read the position in an atomic block to avoid a potential misread 
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
    pos[k] = posi[k];
  }
  interrupts(); // turn interrupts back on

  // Loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir); // compute the position
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]); // signal the motor
  }

  for(int i = 0; i<4; i++){
    Serial.print(target[i]);
    Serial.print(" ");
  }
  for(int i = 0; i<4; i++){
    Serial.print(pos[i]);
    Serial.print(" ");
  }
  Serial.println();
  
}
