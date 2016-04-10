#include <Servo.h>

#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0
//#define CHIPSELECT1 8 // CS_n 1
//#define CHIPSELECT2 7 // CS_n 2
//#define CHIPSELECT3 6 // CS_n 3


#define GYCONTOLWORD 0x0300
#define GYAD 10
#define GYCD 4

// #define PRINTGYCONTROLWORD

struct GyroReading {
  int value;
  byte addr;
  // unsigned int numDel0;
  // unsigned int numDel1;
} gyroReading;

struct ADCStats {
  unsigned long n;
  float val;
  float rAv;
  float sum;
  float sumSq;
} adcStat[4];

Servo servo;

void setup()
{
  byte clr;

  Serial.begin(115200);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(CHIPSELECT0, OUTPUT);
  //pinMode(CHIPSELECT1, OUTPUT);
  //pinMode(CHIPSELECT2, OUTPUT);
  //pinMode(CHIPSELECT3, OUTPUT);
  //delay(10);
  //digitalWrite(CHIPSELECT0, HIGH); // disable device
  //digitalWrite(CHIPSELECT1, HIGH); // disable device
  //digitalWrite(CHIPSELECT2, HIGH); // disable device
  //digitalWrite(CHIPSELECT3, HIGH); // disable device
  digitalWrite(CHIPSELECT0, LOW); // disable device
  //digitalWrite(CHIPSELECT1, LOW); // disable device
  //digitalWrite(CHIPSELECT2, LOW); // disable device
  //digitalWrite(CHIPSELECT3, LOW); // disable device

  servo.attach(9, 585, 2527);
  servo.write(90);

  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA)|(0);
  clr = SPSR;
  clr = SPDR;
  Serial.println("Init complete.");
  readGyro(GYCONTOLWORD | (0x8000) | (1 << GYCD) | (1 << GYAD));
}

void loop()
{
  static boolean selstate = false;
  unsigned int control = GYCONTOLWORD | (1 << GYCD);
  static float val = 0;
  static byte chan = 0;

  if(Serial.available()) {
    int val = Serial.read() - '0';
    switch(val) {
      case 0:
      case 1:
      case 2:
      case 3:
        //control |= (0x8000) | (val << GYAD);
        if(selstate) {
          //digitalWrite(CHIPSELECT0, HIGH);
        } else {
          //digitalWrite(CHIPSELECT0, LOW);
        }
        Serial.println((int)chan);
        selstate = !selstate;
        break;
    }
  } else {
    control |= (0x8000) | ((chan & 0x03) << GYAD);
    chan += (chan & 0x03)?1:2;
  }

  readGyro(control);

  switch(gyroReading.addr) {
    case 0:
      val = (gyroReading.value - 2048) * 0.09766;  // ADIS16080
      // val = (gyroReading.value - 2048) * 0.2439;  // ADIS16100
      break;
    case 1:
      val = (gyroReading.value - 2048) * 0.1453 + 25; // ADIS16100/ADIS16080
      break;
    case 2:
    case 3:
      val = 5*gyroReading.value/(float)4096;
      break;
  }

  adcStat[gyroReading.addr].n++;
  adcStat[gyroReading.addr].val = val;
  adcStat[gyroReading.addr].sum += val;
  adcStat[gyroReading.addr].sumSq += val*val;
  adcStat[gyroReading.addr].rAv = 0.95*adcStat[gyroReading.addr].rAv + 0.05*adcStat[gyroReading.addr].val;

  servo.write((int)(1*adcStat[0].rAv+0.5)+90);

  if(!chan) {
  //if(false) {
    // Serial.print("Address :");
    // Serial.print(gyroReading.addr, DEC);
    // Serial.print(": Value :");
    // Serial.print(gyroReading.value, DEC);
    // Serial.print(": Delay(0) :");
    // Serial.print(gyroReading.numDel0, DEC);
    // Serial.print(": Delay(1) :");
    // Serial.print(gyroReading.numDel1, DEC);
    Serial.print("Gyro :");
    Serial.print((int)(1*adcStat[0].val+0.5));
    Serial.print(": Temp :");
    Serial.print((int)(10*adcStat[1].rAv+0.5));
    Serial.print(": ADC0 :");
    Serial.print((int)(1000*adcStat[2].rAv+0.5));
    Serial.print(": ADC1 :");
    Serial.print((int)(1000*adcStat[3].rAv+0.5));
    Serial.print(": Angle :");
    Serial.print((int)(angle(adcStat[2].val, adcStat[3].val)*180.0/M_PI + 0.5));
    for(byte i = 0; i < 4; i++) {
      float mu = adcStat[i].sum/(float)adcStat[i].n;
      float sig = sqrt(adcStat[i].sumSq/(float)adcStat[i].n - mu*mu);
      Serial.print(": M");
      Serial.print((int)i);
      Serial.print(" :");
      Serial.print((int)(1000*mu + 0.5));
      Serial.print(": S");
      Serial.print((int)i);
      Serial.print(" :");
      Serial.print((int)(1000*sig + 0.5));
    }
    Serial.println(":");
  }
  delay(5);
}

float angle(float v1, float v2)
{
  return atan2(v1 - 1.628, v2 - 2.01);
}

void PrintByte(byte bin)
{
  for(byte a = 0; a < 8; a++) {
    Serial.print((bin & 0x80)==0x80, BIN);
    bin <<= 1;
  }
}

void readGyro(unsigned int control)
{
  #ifdef PRINTGYCONTROLWORD
    Serial.print("Gyro control word :");
    PrintByte(control >> 8);
    Serial.print(",");
    PrintByte(control & 0x00FF);
    Serial.println(":");
  #endif

  //gyroReading.numDel0 = 0;
  //gyroReading.numDel1 = 0;

  digitalWrite(CHIPSELECT0, HIGH); // enable

  SPDR = control >> 8;
  control &= 0x00FF;
  while (!(SPSR & (1<<SPIF))) { // Wait for the end of the transmission
    //gyroReading.numDel0++;
  }
  gyroReading.addr = SPDR;
 
  SPDR = control;
  gyroReading.value = (int)(gyroReading.addr & 0x0F) << 8;
  gyroReading.addr = gyroReading.addr >> 4;
  while (!(SPSR & (1<<SPIF))) { // Wait for the end of the transmission
    //gyroReading.numDel1++;
  }
  gyroReading.value |= SPDR;
 
  digitalWrite(CHIPSELECT0, LOW); // disable
}

byte spi_transfer(volatile byte data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF))) {   // Wait for the end of the transmission
  }
  return SPDR;                    // return the received byte
}
