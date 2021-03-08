/******************************************************************************
   AutoTune RC Filter Example
   Use Serial Monitor and Serial Plotter to view results.
   Reference: https://github.com/Dlloydev/QuickPID/wiki/AutoTune
   Circuit: https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter

   TUNING RULE             RECOMMENED FOR
   0 ZIEGLER_NICHOLS_PI    Good noise and disturbance rejection
   1 ZIEGLER_NICHOLS_PID   Good noise and disturbance rejection
   2 TYREUS_LUYBEN_PI      Time-constant (lag) dominant processes (conservative)
   3 TYREUS_LUYBEN_PID     Time-constant (lag) dominant processes (conservative)
   4 CIANCONE_MARLIN_PI    Delay (dead-time) dominant processes
   5 CIANCONE_MARLIN_PID   Delay (dead-time) dominant processes
   6 AMIGO_PID             More universal than ZN_PID (uses a dead time dependancy)
   7 PESSEN_INTEGRAL_PID   Similar to ZN_PID but with better dynamic response
   8 SOME_OVERSHOOT_PID    ZN_PID with lower proportional and integral gain
   9 NO_OVERSHOOT_PID      ZN_PID with much lower P,I,D gain settings
 ******************************************************************************/
#include "PinChangeInt.h"
#include "QuickPID.h"

// Encoders
#define M1_ENCODER_A A0
#define M1_ENCODER_B A1

//Motors
#define TICKS_PER_REV 980 // Using Pololu 20.4:1 25D gearmotor 48 tick/rev
#define WHEEL_DIAMETER 0.058

// L9958 DIRection pins
#define DIR_M1 2

// L9958 PWM pins
#define PWM_M1 9

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

//const byte inputPin = 0;
//const byte outputPin = 3;

// Motor PWM variables
int pwm1 = 0;
int dir1 = 0;
volatile double speed_M1 = 0;         // Used for input measurement to PID
double out_M1 = 0;                        // Output from PID to power motors
double setspeed_M1 = 0;         // Target speed for motors

// Variables to store the number of encoder pulses
volatile signed long M1_Count = 0;

int Print = 0;                // on(1) monitor, off(0) plotter
int tuningRule = 1;           // see above table
float POn = 1.0;              // Mix of PonE to PonM (0.0-1.0)
unsigned long timeout = 120;  // AutoTune timeout (sec)

//int Input, Output, Setpoint;
float Kp = 0, Ki = 0, Kd = 0;

QuickPID myQuickPID(&speed_M1, &out_M1, &setspeed_M1, Kp, Ki, Kd, POn, DIRECT);

void setup()
{
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
  
  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  
  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);

  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, LOW);   // HIGH = disabled

  Serial.begin(115200);
  myQuickPID.AutoTune(speed_M1, out_M1, tuningRule, Print, timeout);
  myQuickPID.SetTunings(myQuickPID.GetKp(), myQuickPID.GetKi(), myQuickPID.GetKd());
  myQuickPID.SetSampleTimeUs(5000); // recommend 5000µs (5ms) minimum
  myQuickPID.SetMode(AUTOMATIC);
  Setpoint = 700;

  if (Print == 1) {
    // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png
    if (float(myQuickPID.GetTu() / myQuickPID.GetTd() + 0.0001) > 0.75) Serial.println("This process is easy to control.");
    else if (float(myQuickPID.GetTu() / myQuickPID.GetTd() + 0.0001) > 0.25) Serial.println("This process has average controllability.");
    else Serial.println("This process is difficult to control.");
    Serial.print("Tu: "); Serial.print(myQuickPID.GetTu());    // Ultimate Period (sec)
    Serial.print("  td: "); Serial.print(myQuickPID.GetTd());  // Dead Time (sec)
    Serial.print("  Ku: "); Serial.print(myQuickPID.GetKu());  // Ultimate Gain
    Serial.print("  Kp: "); Serial.print(myQuickPID.GetKp());
    Serial.print("  Ki: "); Serial.print(myQuickPID.GetKi());
    Serial.print("  Kd: "); Serial.println(myQuickPID.GetKd());
    delay(6000);
  }
}

void loop()
{ // plotter for stuff
  Serial.print("Setpoint:");  Serial.print(Setpoint);  Serial.print(",");
  Serial.print("Input:");     Serial.print(Input);     Serial.print(",");
  Serial.print("Output:");    Serial.print(Output);    Serial.print(",");
  Serial.println(",");

  //Input = myQuickPID.analogReadFast(inputPin);
  myQuickPID.Compute();
  setMotorSpeed(out_M1);
}

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
