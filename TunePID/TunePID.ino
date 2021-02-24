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

// Motor PWM variables
int pwm1, pwm2, pwm3;
int dir1, dir2, dir3;
char dataString[50] = {0};

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

volatile float val = 0;
volatile float data[] = {0,0,0,0,0,0};
char delimiters[] = ",";
char* valPosition;
char charData[60];

// Variables for testing
float driveTime = 0;
int testNumber = 0;
int testDuration = 2;
int complete = 0;
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

  Serial.begin(9600);

  delay(2000);
 
}

/////////////////////////////////////////////////////////////////////////////////////////////
// LOOP /////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  // Check serial comms for new message
  //readMSG(data);

  
  
  sendMsg();
  
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
  // Don't know why but M2 and M3 counts are backwards.
  // Possible swapped wiring of encoders or swapped pin defs?
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
void sendMsg(){
//  Serial.print(setspeed_M1);
//  Serial.print("\t");
//  Serial.print(speed_M1);
//  Serial.print("\t");
//  Serial.print(out_M1);
//  Serial.print("\t");
//  Serial.print(timeStamp);
//  Serial.print("\t");
//  Serial.println(complete);

 
  Serial.print(t0);
  Serial.print("\t");
  Serial.print(t1);
  Serial.print("\t");
  Serial.print(7);
  Serial.print("\t");
  Serial.print(" data");
  Serial.print("\t");
  Serial.println("in");

}

void readMSG(float *data){
  if (Serial.available() > 0){

    //Store the data in to the variable data
    String str = Serial.readStringUntil('\n');
    str.toCharArray(charData, 50);
    valPosition = strtok(charData, delimiters);
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    

    for (int i = 0; i < 6; i++) {
      data[i] = atof(valPosition);
      //Serial.println(data[i]);
      valPosition = strtok(NULL, delimiters);
    }

    // Get test waveform and PID parameters
    testType = int(data[0]);
    testMag = data[1];
    testPeriod = (data[2] * 1000);
    Kprop = data[3];
    Kint = data[4];
    Kder = data[5];
    
    testNumber = 0;
    complete = 0;
    t0 = millis();
    t1 = millis();
  }
return;
}

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
