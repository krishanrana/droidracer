/*
   Records encoder ticks for each wheel
   and prints the number of ticks for
   each encoder every 500ms
*/
#include "PinChangeInt.h"
//This provides a library to set pin change interrupts which we will use for the encoders


// Encoders
#define RH_ENCODER_A A0
#define RH_ENCODER_B A1
#define LH_ENCODER_A 6
#define LH_ENCODER_B 7
#define M_ENCODER_A 11
#define M_ENCODER_B 13

//Motors
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

// variables to store the number of encoder pulses
// for each motor
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
volatile unsigned long middleCount = 0;

void setup() {
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(M_ENCODER_A, INPUT);
  pinMode(M_ENCODER_B, INPUT);

  // initialize hardware interrupts
  attachPinChangeInterrupt(A0, rightEncoderEvent, CHANGE);
  attachPinChangeInterrupt(6, leftEncoderEvent, CHANGE);
  attachPinChangeInterrupt(11, middleEncoderEvent, CHANGE);

  //Motor stuff

    
  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);

  
  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
 
  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, HIGH);   // HIGH = disabled

  Serial.begin(9600);
}

void loop() {
 // put your main code here, to run repeatedly:
 
  // toggle direction 
  dir1 = !dir1;
  
  // ramp speed up for all motors
  for (pwm1 = 0; pwm1 < 256; pwm1 +=5) {
    analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M2, pwm1);  digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M3, pwm1);  digitalWrite(DIR_M3, dir1);
    
    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
    delay(10);
  }
  delay(1000); 
  
  // ramp speed down for all motors
  for (pwm1 = 255; pwm1 >= 0; pwm1 -=5) {
    analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M2, pwm1);  digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M3, pwm1);  digitalWrite(DIR_M3, dir1);
    
    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
    delay(10);
  } 
  delay(1000);  

}

// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
      //Serial.print("Left Count1: ");
      //Serial.println(leftCount);raspberry
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}

// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++; 
    }
  }
}


// encoder event for the interrupt call
void middleEncoderEvent() {
  if (digitalRead(M_ENCODER_A) == HIGH) {
    if (digitalRead(M_ENCODER_B) == LOW) {
      middleCount++;
    } else {
      middleCount--;
    }
  } else {
    if (digitalRead(M_ENCODER_B) == LOW) {
      middleCount--;
    } else {
      middleCount++; 
    }
  }
}





