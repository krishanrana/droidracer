//This provides a library to set pin change interrupts which we will use for the encoders
#include "PinChangeInt.h"

//Use for PID control of motors
#include <PID_v1.h>

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

// Serial communication definitions
#define INMSGSIZE 6 // Length of message in values (eg. [1.3 100] = 2)
#define INVARBYTES 4 // Bytes per value (eg. float = 4)


// Serial communication definitions
int byteLength = INMSGSIZE * INVARBYTES;
unsigned long timer = 0;
double msgIn[INMSGSIZE];

union Data{ 
  double d;
  byte b[INVARBYTES];
};

// Motor PWM variables
int pwm1, pwm2, pwm3;
int dir1, dir2, dir3;

volatile double speed_M1;         // Used for input measurement to PID
double out_M1;                        // Output from PID to power motors
double setspeed_M1;         // Target speed for motors
float pidSampleRate;

// Variables to store the number of encoder pulses
volatile signed long M1_Count = 0;

// Input Variables 
int testType = 0;
float testMag = 0;
float testPeriod = 0;
float Kprop = 0;
float Kint = 0;
float Kder = 0;

// Variables for testing
long loopTime = 100000;   // microseconds
float driveTime = 0;
int testNumber = 0;
int testDuration = 2;
int completeFlag = 0;
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
  
  // PID variable setup
  speed_M1 = 0;
  setspeed_M1 = 0; 
  
  //Set PID compute rate in milliseconds (default = 100)
  pidSampleRate = 20;
  PID_M1.SetSampleTime(pidSampleRate);
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
  timer = micros();

  delay(1000);
 
}

/////////////////////////////////////////////////////////////////////////////////////////////
// LOOP /////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  timeSync(loopTime);
  // Read: testType, testMag, TestPeriod, Kp,Ki,Kd
  readSerialInput();
  
  // Process commands and start test
  // Placeholder echo some input
  double val1 = msgIn[0];
  double val2 = msgIn[1];
  double val3 = msgIn[2];
  double val4 = msgIn[3];
  double val5 = msgIn[4];

//  Serial.println(val1);
  // Write current test values: SetPoint, motorspeed, PWM, time, Complete
  writeSerial(&val1, &val2, &val3, &val4, &val5);

  
//  if (testNumber <= testDuration){
//    if (PID_M1.GetMode() == MANUAL){
//      PID_M1.SetMode(AUTOMATIC);
//    }
//    
//    // Update test input parameters
//    inputWaveform(testType,testMag,testPeriod);
//  
//    // Compute PID values
//    PID_M1.SetTunings(Kprop, Kint, Kder); 
//    PID_M1.Compute();
//   
//    // Write to the motor directions and pwm power   
//    // Allow for negative (Reverse) velocity
// 
//    setMotorSpeed(out_M1);
//    timeStamp = millis() - t0;
////    Serial.print("Running no.");
////    Serial.println(testNumber);
//    sendMsg();
//    }
//  else{
//    // Test is ended, slow motor to zero and turn off while waiting for new input 
//    setspeed_M1 = 0;
//    complete = 1;  
//    if(abs(out_M1) > 3){
//      PID_M1.SetTunings(5, 1, 0);
//      PID_M1.Compute();
//      setMotorSpeed(out_M1);
////      Serial.print("Slowing no.");
////      Serial.println(testNumber);
//      sendMsg(); 
//    }
//    // When motor is not running, this state may not be entered into.. FIX
//    else{
//      // Set output to 0 and park
//      PID_M1.SetMode(MANUAL);
//      analogWrite(PWM_M1, 0);
////      Serial.print("Stopped no.");
////      Serial.println(testNumber);
//      sendMsg();
//    }
//  }
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
  long timeToDelay = deltaT - (currTime - timer);
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
  timer = currTime + timeToDelay;
}

// void writeSerial(int* data1, int* data2, int* data3)
// {
//   byte* byteData1 = (byte*)(data1);
//   byte* byteData2 = (byte*)(data2);
//   byte* byteData3 = (byte*)(data3);
//   byte buf[6] = {byteData1[0], byteData1[1],
//                  byteData2[0], byteData2[1],
//                  byteData3[0], byteData3[1]};
//   Serial.write(buf, 6);
// }

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
      msgIn[0] = 999;
    }
  }
  else{
    msgIn[0] = -999;
  }
}


//     // Get test waveform and PID parameters
//     testType = int(msgIn[0]);
//     testMag = msgIn[1];
//     testPeriod = (msgIn[2] * 1000);
//     Kprop = msgIn[3];
//     Kint = msgIn[4];
//     Kder = msgIn[5];
    
//     testNumber = 0;
//     complete = 0;
//     t0 = millis();
//     t1 = millis();
//   }
// return;
// }

void inputWaveform(float testType, float testMag, float testPeriod) {
 
  driveTime = millis()-t1;
  
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
