//This provides a library to set pin change interrupts which we will use for the encoders
#include "PinChangeInt.h"

//Use for PID control of motors
#include <PID_v2.h>


// Encoders
#define M1_ENCODER_A A0
#define M1_ENCODER_B A1

//Motors
#define TICKS_PER_REV 980 // Using Pololu 20.4:1 25D gearmotor 48 tick/rev
#define WHEEL_DIAMETER 0.058
#define DROIDRADIUS 0.15
// L9958 DIRection pins
#define DIR_M1 2

// L9958 PWM pins
#define PWM_M1 9

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

// PID
#define PIDSAMPLERATE 20
#define LOOPTIME 100000

// Serial communication definitions
#define INMSGSIZE 6 // Length of message in values (eg. [1.3 100] = 2)
#define INVARBYTES 4 // Bytes per value (eg. float = 4)


// Serial communication definitions
int byteLength = INMSGSIZE * INVARBYTES;
unsigned long delayTimer = 0;
double msgIn[INMSGSIZE];
// Signals sent in last position of message
double m0 = 0.0; // Test completed
double m1 = 1.0; // Test running
double m5 = 5.0; // Waiting for input
double m100 = 100.00;// No serial communication received

// Define union for variable datatype conversion
union Data{ 
  double d;
  byte b[INVARBYTES];
};

// Motor PWM variables
int pwm1 = 0;
int dir1 = 0;
volatile double speed_M1 = 0;         // Used for input measurement to PID
double out_M1 = 0;                        // Output from PID to power motors
double setspeed_M1 = 0;         // Target speed for motors

// Variables to store the number of encoder pulses
volatile signed long M1_Count = 0;

// Input Variables 
int testType = 100;
float testMag = 0;
float testPeriod = 0;
float Kprop = 0;
float Kint = 0;
float Kder = 0;

// Variables for testing
double driveTime = 0;
int testNumber = 0;
int testDuration = 2;
int completeFlag = 1;
double timeStamp = 0;
double t0 = 0;
double t1 = 0;


// PID Constructor 
PID PID_M1(&speed_M1, &out_M1, &setspeed_M1, 50, 100, 0, P_ON_M, DIRECT);


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
  
  // initialize hardware interrupts
  attachPinChangeInterrupt(M1_ENCODER_A, M1EncoderEvent, CHANGE);
  
  //Set PID compute rate in milliseconds (default = 100)
  PID_M1.SetSampleTime(PIDSAMPLERATE);
  // Configure for backwards values too
  PID_M1.SetOutputLimits(-255, 255);
  
  // Turn the PIDs on
  PID_M1.SetMode(AUTOMATIC);
  
  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  
  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  

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
  //timeSync(LOOPTIME);
  // Check to see if new message arrives
  if(completeFlag == 1){ 
    // Send signal: Ready and waiting for input
    writeSerial(&m0, &m0, &m0, &m0, &m100);
    timeSync(LOOPTIME);
    // Read: testType, testMag, TestPeriod, Kprop,Kint,Kder
    readSerialInput();
    // Test does not commence until proper data read (100 is no-read flag)
    if (msgIn[0] != 100){
      completeFlag = 0;
      // Send signal: Input received, starting test
      writeSerial(&m0, &m0, &m0, &m0, &m5);   
    }
  }
  else{
  // Process commands and start test
    testType = int(msgIn[0]);
    testMag = msgIn[1];// Target velocity in ms^-1
    testPeriod = (msgIn[2] * 1000);// convert to milliseconds
    Kprop = msgIn[3]*255;
    Kint = msgIn[4]*255;
    Kder = msgIn[5]*255;
    
    msgIn[0] = 100;
    
    timeSync(30000);
    PID_M1.SetTunings(Kprop, Kint, Kder);
    testNumber = 1;
    t0 = millis();
    t1 = millis();
    // Check if test is finished
    while (testNumber <= testDuration){
      if (PID_M1.GetMode() == MANUAL){
        PID_M1.SetMode(AUTOMATIC);
      }      
      // Update test input parameters
      setspeed_M1 = getTargetVelocity(testType, testMag, testPeriod);
      PID_M1.Compute();
      setMotorSpeed(out_M1);
      timeStamp = millis() - t0;
      //Send data + signal:  Test running
      writeSerial(&setspeed_M1, &speed_M1, &out_M1, &timeStamp, &m1);
      }

    // Test is ended, slow motor to zero and turn off before getting new input 
    setspeed_M1 = 0.0;
    PID_M1.SetTunings(380, 7800, 0.255);
    
  // Park system slowly to avoid integral windup on restart
  for(int t = 0; t<50;t+=1){
    PID_M1.Compute();
    setMotorSpeed(out_M1);
    timeStamp = millis() - t0;
    // Send signal: Test running
    writeSerial(&setspeed_M1, &speed_M1, &out_M1, &timeStamp, &m1); 
    timeSync(20000);
    }
  
  // Set output to 0 and park
  completeFlag = 1;  
  PID_M1.SetOutputLimits(0, 0);
  PID_M1.SetMode(MANUAL);
  PID_M1.SetOutputLimits(-255, 255);
  analogWrite(PWM_M1, 0);
  timeStamp = millis() - t0;
  // Send signal: All stopped, test over
  writeSerial(&setspeed_M1, &speed_M1, &out_M1, &timeStamp, &m0);
  delay(500);
    
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPTS ///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) // timer compare interrupt service routine - fires every 0.01632 seconds
{
  // Ticks per second
  speed_M1 = ticks2metres(-M1_Count / 0.01632);
  
  M1_Count = 0;
  
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

void writeSerial(double* data1, double* data2, double* data3, double* data4, double* data5)
{ // TODO: Rewrite using loops, Unions etc
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);

  byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                 byteData5[0], byteData5[1], byteData5[2], byteData5[3]};

  Serial.write(buf, 20);
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
      msgIn[0] = 100;
    }
  }
  else{
    msgIn[0] = 100;
  }
}


double getTargetVelocity(int Type, float testMag, float testPeriod) {
 
  driveTime = millis()-t1;
  
  double setVelocity = 0;
  
  switch (Type){
    case 0:
      return stepInput(driveTime,testMag,testPeriod);
    
//    case 1:
//      setVelocity = rampInput(driveTime,testMag,testPeriod);
//    break;
//
//    case 2:
//      setVelocity = sinInput(driveTime,testMag,testPeriod);
//    break;

    default:
      return 0;
  }
}

double stepInput(double t, float mag, float T){
  double setVelocity = 0;
  if (t < T/2){
    setVelocity = mag;
  }
  else if (t < T){
    setVelocity = -mag;
    }
  else if (t >= T){
    driveTime = 0;
    t1 = millis();
    testNumber += 1;
    }
  return setVelocity;
}

double rampInput(double t, float mag, float T){
  double setVelocity = 0;
  double accel = 4*mag/T;
  if (t < T/4){
    setVelocity = accel * t;
  }
  else if (t < 3*T/4){
    setVelocity = mag - (t-T/4)*accel;
    }
  else if (t >= T){
    setVelocity = 0;
    driveTime = 0;
    t1 = millis();
    testNumber += 1;
    }
  return setVelocity;
}



void inputWaveform(float testType, float testMag, float testPeriod) {
 
  driveTime = millis()-t1;
  // NOTE: Change to acceleration / jerk model
  // Test PID using synthetic input
  if (driveTime < testPeriod/2){
    // Step input to testMag
    if (testType == 0){
      setspeed_M1 = testMag;
    }
    else if (testType == 1){
      setspeed_M1 += testMag/testPeriod;
    }
    else{setspeed_M1 = 0;}     
  }
  
  else if (driveTime < testPeriod){
    // Step input to -testMag
    if (testType == 0){
      setspeed_M1 = -testMag;
    }
    else if (testType == 1){
      setspeed_M1 -= testMag/testPeriod;
    }
    else{setspeed_M1 = 0;}     
  }

  else if (driveTime >= testPeriod){
    setspeed_M1 = 0;
    driveTime = 0;
    t1 = millis();
    testNumber += 1;
  }
}

void setMotorSpeed(double out_M1){
    
      if (out_M1 < 0) {
        digitalWrite(DIR_M1, LOW);
      } else {
        digitalWrite(DIR_M1, HIGH);
      }
      analogWrite(PWM_M1, int(abs(out_M1)));
      return;
    }

double ticks2metres(int ticks) {
  return double(ticks) / TICKS_PER_REV * PI * WHEEL_DIAMETER;
}
