// Test code from https://thepoorengineer.com/en/arduino-python-plot/

unsigned long timer = 0;
long loopTime = 5000;   // microseconds
double loops = 0;

void setup() {
  Serial.begin(38400);
  timer = micros();
}

void loop() {
  timeSync(loopTime);
  loops += 1;
  //int val = analogRead(0) - 512;
  //double val = (analogRead(0) -512) / 512.0;
  double val1 = loops;
  double val2 = random(-10,10);
  double val3 = random(-5,5);
  //Serial.println(val2);
  sendToPC(&val1, &val2, &val3);
  if (loops > 10){
    loops = 0; 
  }
}

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

void sendToPC(int* data1, int* data2, int* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[6] = {byteData1[0], byteData1[1],
                 byteData2[0], byteData2[1],
                 byteData3[0], byteData3[1]};
  Serial.write(buf, 6);
}

void sendToPC(double* data1, double* data2, double* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}
