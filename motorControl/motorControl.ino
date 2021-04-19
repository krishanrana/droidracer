//This provides a library to set pin change interrupts which we will use for the encoders
#include "PinChangeInt.h"

//Use for PID control of motors
#include <PID_v2.h>


// Encoders
#define M1_ENCODER_A A0
#define M1_ENCODER_B A1
#define M2_ENCODER_A 11
#define M2_ENCODER_B 13
#define M3_ENCODER_A 6
#define M3_ENCODER_B 7

//Motors
#define TICKS_PER_REV 980 // Using Pololu 20.4:1 25D gearmotor 48 tick/rev
#define WHEEL_DIAMETER 0.058

// L9958 Direction pins
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4

// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10    // Timer1
#define PWM_M3 5

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

// PID
#define PIDSAMPLERATE 20
#define LOOPTIME 30000

// Serial communication definitions
#define INMSGSIZE 7 // Length of message in values (eg. [1.3 100] = 2)
#define INVARBYTES 4 // Bytes per value (eg. float = 4)

// States and signals
#define PARKED 0
#define RUNNING 1
#define STANDBY 3
#define TIMEOUT 5
#define NO_COMMS 100


// Serial communication variables
int byteLength = INMSGSIZE * INVARBYTES;
double msgIn[INMSGSIZE];

// Signals and state machine
int state = NO_COMMS;

// Timers
unsigned long delayTimer = 0;
unsigned long safetyClock = 0;
unsigned long parkClock = 0;
unsigned long safetyTimeOut = LOOPTIME * 50;
unsigned long parkTimeOut = LOOPTIME * 20;

// Define union for variable datatype conversion
union Data{ 
  double d;
  byte b[INVARBYTES];
};

// Motor PWM variables
int pwm1 = 0;
int dir1 = 0;
int pwm2 = 0;
int dir2 = 0;
int pwm3 = 0;
int dir3 = 0;

// PID variable setup
volatile double speed_M1 = 0;
volatile double speed_M2 = 0;
volatile double speed_M3 = 0;

// Output from PID to power motors
double out_M1, out_M2, out_M3;                        

// Variables to store the number of encoder pulses
volatile signed long M1_Count = 0;
volatile signed long M2_Count = 0;
volatile signed long M3_Count = 0;

// Input Variables 
double setspeed_M1 = 0;
double setspeed_M2 = 0;
double setspeed_M3 = 0;
float Kprop = 0;
float Kint = 0;
float Kder = 0;

// PID Constructor 
PID PID_M1(&speed_M1, &out_M1, &setspeed_M1, Kprop, Kint, Kder,P_ON_M, DIRECT);
PID PID_M2(&speed_M2, &out_M2, &setspeed_M2, Kprop, Kint, Kder,P_ON_M, DIRECT);
PID PID_M3(&speed_M3, &out_M3, &setspeed_M3, Kprop, Kint, Kder,P_ON_M, DIRECT);

/////////////////////////////////////////////////////////////////////////////////////////////
// SETUP ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  //initialize Timer2
  noInterrupts(); // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 255; // compare match register 16MHz/256/2Hz
  TCCR2A |= (1 << WGM21); // CTC mode
  TCCR2B |= (1 << CS22); // 1024 prescaler
  TCCR2B |= (1 << CS21); // 1024 prescaler
  TCCR2B |= (1 << CS20); // 1024 prescaler
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
  interrupts(); // enable all interrupts

  pinMode(M1_ENCODER_A, INPUT);
  pinMode(M1_ENCODER_B, INPUT);
  pinMode(M2_ENCODER_A, INPUT);
  pinMode(M2_ENCODER_B, INPUT);
  pinMode(M3_ENCODER_A, INPUT);
  pinMode(M3_ENCODER_B, INPUT);

  // initialize hardware interrupts
  attachPinChangeInterrupt(M1_ENCODER_A, M1EncoderEvent, CHANGE);
  attachPinChangeInterrupt(M2_ENCODER_A, M2EncoderEvent, CHANGE);
  attachPinChangeInterrupt(M3_ENCODER_A, M3EncoderEvent, CHANGE);

  //Set PID compute rate in milliseconds (default = 100)
  
  PID_M1.SetSampleTime(PIDSAMPLERATE);
  PID_M2.SetSampleTime(PIDSAMPLERATE);
  PID_M3.SetSampleTime(PIDSAMPLERATE);

  // Configure for backwards values too
  PID_M1.SetOutputLimits(-255, 255);
  PID_M2.SetOutputLimits(-255, 255);
  PID_M3.SetOutputLimits(-255, 255);
  // Turn the PIDs on
  PID_M1.SetMode(AUTOMATIC);
  PID_M2.SetMode(AUTOMATIC);
  PID_M3.SetMode(AUTOMATIC);

  // L9958 Direction pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);

  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);

  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, LOW);   // HIGH = disabled

  // Open serial port
  Serial.begin(38400);
  delayTimer = micros();

  delay(1000);
}

/////////////////////////////////////////////////////////////////////////////////////////////
// LOOP /////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // State transitions
  // Ensure constant loop speed
  timeSync(LOOPTIME);
  
  // Read: setspeed_M1, setspeed_M2, setspeed_M3, Kprop,Kint,Kder
  readSerialInput();
  int command  = int(msgIn[0]);
  
  // Check that a valid message is received, set operation state and reset safety timer
  if (command != NO_COMMS){
    safetyClock = 0;
   
    switch (command){
      case PARKED:
        state = PARKED;
        break;       
      case RUNNING:
        state = RUNNING;
        break;
      case STANDBY:
        state = STANDBY;
      default:
        state = NO_COMMS;
        break; 
    }         
  }
  else{
    state = NO_COMMS;
    Serial.print("line 200");
  }
  
  if (safetyClock > safetyTimeOut){
    state = TIMEOUT;
  }
  // State actions
  switch (state){
      case PARKED:
        parkClock = 0;
        
        PID_M1.SetOutputLimits(0, 0);
        PID_M1.SetMode(MANUAL);
        PID_M1.SetOutputLimits(-255, 255);
        analogWrite(PWM_M1, 0);
        PID_M2.SetOutputLimits(0, 0);
        PID_M2.SetMode(MANUAL);
        PID_M2.SetOutputLimits(-255, 255);
        analogWrite(PWM_M1, 0);
        PID_M3.SetOutputLimits(0, 0);
        PID_M3.SetMode(MANUAL);
        PID_M3.SetOutputLimits(-255, 255);
        analogWrite(PWM_M1, 0);
        
      case RUNNING:
        parkClock = 0;
        if (PID_M1.GetMode() == MANUAL){
          PID_M1.SetMode(AUTOMATIC);
        }     
        setspeed_M1 = msgIn[1];// Target velocity in ms^-1
        setspeed_M2 = msgIn[2];// Target velocity in ms^-1
        setspeed_M3 = msgIn[3];// Target velocity in ms^-1
        Kprop = msgIn[4]*255;// Match PWM scale
        Kint = msgIn[5]*255;
        Kder = msgIn[6]*255;
        
      case STANDBY:
        parkClock = 0;
        setspeed_M1 = 0;// Target velocity in ms^-1
        setspeed_M2 = 0;// Target velocity in ms^-1
        setspeed_M3 = 0;// Target velocity in ms^-1
        
      case TIMEOUT:
        setspeed_M1 = 0;// Target velocity in ms^-1
        setspeed_M2 = 0;// Target velocity in ms^-1
        setspeed_M3 = 0;// Target velocity in ms^-1
        Kprop = 380;
        Kint = 7800;
        Kder = 0.255;
        parkClock += LOOPTIME;
        if (parkClock > parkTimeOut){
          state = PARKED;
        }
              
      case NO_COMMS:
        break; 
    }    

  // Set Kp, Ki, Kd
  PID_M1.SetTunings(Kprop, Kint, Kder);
  PID_M2.SetTunings(Kprop, Kint, Kder);
  PID_M3.SetTunings(Kprop, Kint, Kder);
  
  // Update PID controller, Calculate output  
  PID_M1.Compute();
  PID_M2.Compute();
  PID_M3.Compute();
    
  // Set output (u)
  setMotorSpeeds(out_M1,out_M2,out_M3);
  safetyClock += LOOPTIME;
 
  //Send data + state:  
  double mess = double(state);
  writeSerial(&mess,&speed_M1,&out_M1,&speed_M2,&out_M2,&speed_M3,&out_M3);
}
    
/////////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPTS ///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) // timer compare interrupt service routine - fires every 0.01632 seconds
{
  // Ticks per second
  
  speed_M1 = ticks2metres(-M1_Count / 0.01632);
  speed_M2 = ticks2metres(M2_Count / 0.01632);
  speed_M3 = ticks2metres(M3_Count / 0.01632);

  M1_Count = 0;
  M2_Count = 0;
  M3_Count = 0;
}


// encoder event for the interrupt call
void M1EncoderEvent() {
  if (digitalRead(M1_ENCODER_A) == HIGH) {
    if (digitalRead(M1_ENCODER_B) == LOW) {
      M1_Count++;
    } else {
      M1_Count--;
    }
  } else {
    if (digitalRead(M1_ENCODER_B) == LOW) {
      M1_Count--;
    } else {
      M1_Count++;
    }
  }
}


// encoder event for the interrupt call
void M2EncoderEvent() {
  if (digitalRead(M2_ENCODER_A) == HIGH) {
    if (digitalRead(M2_ENCODER_B) == LOW) {
      M2_Count++;
    } else {
      M2_Count--;
    }
  } else {
    if (digitalRead(M2_ENCODER_B) == LOW) {
      M2_Count--;
    } else {
      M2_Count++;
    }
  }
}

// encoder event for the interrupt call
void M3EncoderEvent() {
  if (digitalRead(M3_ENCODER_A) == HIGH) {
    if (digitalRead(M3_ENCODER_B) == LOW) {
      M3_Count++;
    } else {
      M3_Count--;
    }
  } else {
    if (digitalRead(M3_ENCODER_B) == LOW) {
      M3_Count--;
    } else {
      M3_Count++;
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////
// OTHER FUNCTIONS //////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - delayTimer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  delayTimer = currTime + timeToDelay;
}

void writeSerial(double* data1, double* data2, double* data3, double* data4, double* data5, double* data6, double* data7)
{ // TODO: Rewrite using loops, Unions etc
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);
  byte* byteData7 = (byte*)(data7);
  
  byte buf[28] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                 byteData5[0], byteData5[1], byteData5[2], byteData5[3],
                 byteData6[0], byteData6[1], byteData6[2], byteData6[3],
                 byteData7[0], byteData7[1], byteData7[2], byteData7[3]};

  Serial.write(buf, 28);
}

void readSerialInput(){
  
  int bytelength = INMSGSIZE * INVARBYTES;
  byte bufIn[byteLength];
  double t0 = millis();
  // Function polls serial line for complete message
  if (Serial.available() >= byteLength){

    int byteIn = Serial.readBytes((byte*)&bufIn,byteLength);
    
    // Check that message is complete
    if (byteIn == byteLength){
      // if complete, decode from bytes to floats     
      for (int var = 0;var<INMSGSIZE;var++){
        int writebit = 0;
        union Data dataIn;
        for (int readbit = (var*INVARBYTES);readbit<(var*INVARBYTES+INVARBYTES);readbit++){       
          // Read each value bytes into union container 
          dataIn.b[writebit] = bufIn[readbit];
          writebit+=1;
        }
        msgIn[var] = dataIn.d;       
      }
    }
    else{
      msgIn[0] = NO_COMMS;
    }
  }
  else{
    msgIn[0] = NO_COMMS;
  }
}

void setMotorSpeeds(double out_M1,double out_M2,double out_M3){
    
      if (out_M1 < 0) {
        digitalWrite(DIR_M1, LOW);
      } else {
        digitalWrite(DIR_M1, HIGH);
      }
      analogWrite(PWM_M1, int(abs(out_M1)));
      return;
      
      if (out_M2 < 0) {
        digitalWrite(DIR_M2, LOW);
      } else {
        digitalWrite(DIR_M2, HIGH);
      }
      analogWrite(PWM_M2, int(abs(out_M2)));
      return;
      
      if (out_M3 < 0) {
        digitalWrite(DIR_M3, LOW);
      } else {
        digitalWrite(DIR_M3, HIGH);
      }
      analogWrite(PWM_M3, int(abs(out_M3)));
      return;
    }
    
double ticks2metres(int ticks) {
  return double(ticks) / TICKS_PER_REV * PI * WHEEL_DIAMETER;
}
