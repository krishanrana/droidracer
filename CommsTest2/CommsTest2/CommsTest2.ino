// Test code from https://thepoorengineer.com/en/arduino-python-plot/

#define INMSGSIZE 3 // Length of message in values (eg. [1.3 100] = 2)
#define INVARBYTES 4 // Bytes per value (eg. float = 4)

int byteLength = INMSGSIZE * INVARBYTES;

unsigned long timer = 0;
long loopTime = 100000;   // microseconds
double msgIn[INMSGSIZE];

union Data{ 
  double d;
  byte b[INVARBYTES];
};

void setup() {
  Serial.begin(38400);
  timer = micros();
}

void loop() {
  //Serial.reset_input_line();
  // Check serial input for message
  timeSync(loopTime);
  readSerialInput();
  
  // Process and send back
  double val1 = msgIn[0] + 1;
  double val2 = msgIn[1] + 1;
  double val3 = msgIn[2] + 1;
//  Serial.println(val1);
  
  sendToPC(&val1, &val2, &val3);

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

//void getSerialData()
//{
//  while (Serial.available())
//  {
//    char input = Serial.read();
//    g_recvString += input;
//    if (input == '%')   // this is the end of message marker
//    {
//      int index = g_recvString.indexOf('%');
//      String tmp = g_recvString.substring(0, index);  // remove the '%' sign
//      g_scaleFactor = tmp.toFloat();
//      g_recvString = "";
//      break;
//    }
//  }
//}
