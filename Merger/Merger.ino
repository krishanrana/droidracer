//This provides a library to set pin change interrupts which we will use for the encoders
#include "PinChangeInt.h"

//Use for PID control of motors
#include <PID_v1.h>


// Encoders
#define M1_ENCODER_A A0
#define M1_ENCODER_B A1
#define M2_ENCODER_A 11
#define M2_ENCODER_B 13
#define M3_ENCODER_A 6
#define M3_ENCODER_B 7

//Motors
#define TICKS_PER_REV 108
#define WHEEL_DIAMETER 0.058
// L9958 DIRection pins
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10    // Timer1
#define PWM_M3 5
// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

int pwm1, pwm2, pwm3;
int dir1, dir2, dir3;


// PID variables
#define KP 10
#define KI 30
#define KD 0
volatile double speed_M1, speed_M2, speed_M3;         // Used for input measurement to PID
double out_M1, out_M2, out_M3;                        // Output from PID to power motors
double setspeed_M1, setspeed_M2, setspeed_M3;         // Target speed for motors
PID PID_M1(&speed_M1, &out_M1, &setspeed_M1, KP, KI, KD, DIRECT);
PID PID_M2(&speed_M2, &out_M2, &setspeed_M2, KP, KI, KD, DIRECT);
PID PID_M3(&speed_M3, &out_M3, &setspeed_M3, KP, KI, KD, DIRECT);


// variables to store the number of encoder pulses
// for each motor
volatile signed long M1_Count = 0;
volatile signed long M2_Count = 0;
volatile signed long M3_Count = 0;

volatile float heading_angle = 0;


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

  //Motor stuff

  // PID variable setup
  speed_M1 = 0;
  speed_M2 = 0;
  speed_M3 = 0;
  setspeed_M1 = 0;
  setspeed_M2 = 0;
  setspeed_M3 = 0;
  // Configure for backwards values too
  PID_M1.SetOutputLimits(-255, 255);
  PID_M2.SetOutputLimits(-255, 255);
  PID_M3.SetOutputLimits(-255, 255);
  // Turn the PIDs on
  PID_M1.SetMode(AUTOMATIC);
  PID_M2.SetMode(AUTOMATIC);
  PID_M3.SetMode(AUTOMATIC);
    
  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  
  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
 
  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, LOW);   // HIGH = disabled

  Serial.begin(9600);

  delay(2000);
}


/////////////////////////////////////////////////////////////////////////////////////////////
// LOOP /////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
 
  // Check serial comms for new vel/heading/angular_vel
  //TODO:
  

  // Update the measured motor speeds
  //computeVelocities(2, PI, 0);

  // Just using these for testing instead of using computeVelocities()
  setspeed_M1 = 2;
  setspeed_M2 = -2;
  

  // Process the PIDs
  Serial.print("Motor 1: ");
  Serial.print(speed_M1);
  Serial.print(", ");
  Serial.print(out_M1);
  Serial.print(", ");
  Serial.println(setspeed_M1);
  Serial.print("Motor 2: ");
  Serial.print(speed_M2);
  Serial.print(", ");
  Serial.print(out_M2);
  Serial.print(", ");
  Serial.println(setspeed_M2);
  Serial.print("Motor 3: ");
  Serial.print(speed_M3);
  Serial.print(", ");
  Serial.print(out_M3);
  Serial.print(", ");
  Serial.println(setspeed_M3);
  Serial.println("");
  PID_M1.Compute();
  PID_M2.Compute();
  PID_M3.Compute();

  // Write to the motor directions and pwm power

  if (out_M1 < 0){
    digitalWrite(DIR_M1, LOW);
  } else {
    digitalWrite(DIR_M1, HIGH);
  }
  analogWrite(PWM_M1, int(abs(out_M1)));

  if (out_M2 < 0){
    digitalWrite(DIR_M2, LOW);
  } else {
    digitalWrite(DIR_M2, HIGH);
  }
  analogWrite(PWM_M2, int(abs(out_M2)));

  if (out_M3 < 0){
    digitalWrite(DIR_M3, LOW);
  } else {
    digitalWrite(DIR_M3, HIGH);
  }
  analogWrite(PWM_M3, int(abs(out_M3)));
}


/////////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPTS ///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) // timer compare interrupt service routine - fires every 0.01632 seconds
{
  // Ticks per second
  // Don't know why but M2 and M3 counts are backwards. 
  // Possible swapped wiring of encoders or swapped pin defs?
  speed_M1 = ticks2metres(M1_Count / 0.01632);
  speed_M2 = ticks2metres(-M2_Count / 0.01632);
  speed_M3 = ticks2metres(-M3_Count / 0.01632);

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

void computeVelocities(float vel, float heading, float angular_vel) {
  // 3 wheel omniwheel kinematics
  // Transforms from velocity/heading/angular velocity to motor speeds
  int radius = 12;
  setspeed_M1 = vel * (-0.5 * cos(heading) - sqrt(3) / 2 * sin(heading)) + radius * angular_vel;
  setspeed_M2 = vel * (-0.5 * cos(heading) + sqrt(3) / 2 * sin(heading)) + radius * angular_vel;
  setspeed_M3 = vel * cos(heading) + radius * angular_vel;
}

double ticks2metres(int ticks){
  return double(ticks)/TICKS_PER_REV * PI * WHEEL_DIAMETER;
}