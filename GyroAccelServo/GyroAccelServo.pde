#include <Servo.h>

#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0

#define SERVOPIN 9

#define SPISPEED (0x03 & (0))
#define SPISETTINGS ((1<<SPE)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA)|SPISPEED)

#define GYCD 4
#define GYAD 10
#define GYCONTOLBASE (0x0300)
#define GYCONTOLWORD (GYCONTOLBASE | (0x8000) | (1<<GYCD))

#define GYROCHAN 0
#define ACCELCHAN 3

#define SWAPBYTES(a) ((0x00 | ((a) << 8)) | ((a) >> 8))
#define SQR(a) ((a)*(a))

struct GyISRData {
  volatile byte iState;
  volatile int currReading;
  unsigned int currGyControlWord;
  int gyroReading[8];
} gyISRData;

Servo servo;

void setup()
{
  byte clr;

  Serial.begin(115200);

  pinMode(DATAIN, INPUT); // SPI
  pinMode(DATAOUT, OUTPUT);
  pinMode(SPICLOCK, OUTPUT);

  pinMode(CHIPSELECT0, OUTPUT); // SPI gyro chip select signal
  digitalWrite(CHIPSELECT0, LOW);

  SPSR = (1<<SPI2X);
  SPCR = (0<<SPIE)|SPISETTINGS;
  clr = SPSR;
  clr = SPDR;

  findGyroZero();

  Serial.println("Gyro init complete.");

  servo.attach(SERVOPIN, 585, 2527);
  servo.write(90);

  Serial.println("Servo init complete.");
}

void loop()
{
  static unsigned int nCalls = 0;
  static float sensorNulls[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  static float sensorReadings[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  static byte chanNo = 0;

  startReadingGyros();
  getGyroVals();

  // Serial.print("After(");
  // Serial.print(nWait);
  // Serial.println("):");

  //sensorReadings[0] = 0.95 * sensorReadings[0] + 0.05 * ((5*(gyISRData.gyroReading[0] & 0x0FFF)/(float)4096)-1.65)*1.25;
  //sensorReadings[1] = 0.95 * sensorReadings[1] + 0.05 * ((5*(gyISRData.gyroReading[1] & 0x0FFF)/(float)4096)-1.65)*1.25;
  //sensorReadings[2] = 0.95 * sensorReadings[2] + 0.05 * ((5*(gyISRData.gyroReading[2] & 0x0FFF)/(float)4096)-1.65)*1.25;
  //sensorReadings[3] = 0.95 * sensorReadings[3] + 0.05 * ((5*(gyISRData.gyroReading[3] & 0x0FFF)/(float)4096)-1.65)*1.25;

  sensorReadings[0] = 0.99 * sensorReadings[0] + 0.01 * (5*(gyISRData.gyroReading[0] & 0x0FFF)/(float)4096-1.934)*1.25;
  sensorReadings[1] = 0.99 * sensorReadings[1] + 0.01 * (5*(gyISRData.gyroReading[1] & 0x0FFF)/(float)4096-1.651)*1.25;
  sensorReadings[2] = 0.99 * sensorReadings[2] + 0.01 * (5*(gyISRData.gyroReading[2] & 0x0FFF)/(float)4096-1.763)*1.25;
  sensorReadings[3] = 0.99 * sensorReadings[3] + 0.01 * (5*(gyISRData.gyroReading[3] & 0x0FFF)/(float)4096);

  //sensorReadings[0] = 0.95 * sensorReadings[0] + 0.05 * (5*(gyISRData.gyroReading[0] & 0x0FFF)/(float)4096);
  //sensorReadings[1] = 0.95 * sensorReadings[1] + 0.05 * (5*(gyISRData.gyroReading[1] & 0x0FFF)/(float)4096);
  //sensorReadings[2] = 0.95 * sensorReadings[2] + 0.05 * (5*(gyISRData.gyroReading[2] & 0x0FFF)/(float)4096);
  //sensorReadings[3] = 0.95 * sensorReadings[3] + 0.05 * (5*(gyISRData.gyroReading[3] & 0x0FFF)/(float)4096);

  sensorReadings[4] = 0.95 * sensorReadings[4] + 0.05 * (((gyISRData.gyroReading[4] & 0x0FFF) - 2048) * 0.09766);
  sensorReadings[5] = 0.95 * sensorReadings[5] + 0.05 * (((gyISRData.gyroReading[5] & 0x0FFF) - 2048) * 0.09766);
  sensorReadings[6] = 0.95 * sensorReadings[6] + 0.05 * (((gyISRData.gyroReading[6] & 0x0FFF) - 2048) * 0.2439);
  sensorReadings[7] = 0.95 * sensorReadings[7] + 0.05 * (((gyISRData.gyroReading[7] & 0x0FFF) - 2048) * 0.2439);


  if(!(nCalls % 63)) {
    if(Serial.available()) {
      int val = Serial.read() - '0';
      switch(val) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
          chanNo = val;
          break;
        case 9:
          memcpy(sensorNulls, sensorReadings, sizeof(sensorNulls));
          break;
      }
    }
    switch(chanNo) {
        case 0:
        case 1:
        case 2:
        case 3:
          servo.write(sensorReadings[chanNo+4] - sensorNulls[chanNo+4] + 90);
          break;
        case 4:
        case 5:
        case 6:
          servo.write(sensorReadings[chanNo-4]*90+90);
          break;
        case 7:
          servo.write((float)(atan2(sensorReadings[2], sensorReadings[1])*180.0/M_PI));
          break;
        case 8:
          servo.write((float)(atan2(sensorReadings[2], sensorReadings[0])*180.0/M_PI));
          break;
    }
  }

  if(!(nCalls % 2000) || ((gyISRData.gyroReading[3] & 0x0FFF) < 4000)) {
    Serial.println();
    for(int i = 0; i < 8; i++) {
      Serial.print("Address:");
      Serial.print((0xFF00 & gyISRData.gyroReading[i]) >> 12);
      Serial.print(": Index:");
      Serial.print(i);
      Serial.print(": Value:");
      //Serial.print((long int)((sensorReadings[i] - sensorNulls[i])*10 + 0.5));
      PrintFloat(sensorReadings[i] - sensorNulls[i], 3);
      Serial.print(": Reading:");
      Serial.print(gyISRData.gyroReading[i] & 0x0FFF);
      Serial.print(": Angle:");
      PrintFloat(atan2(sensorReadings[1], sensorReadings[2])*180.0/M_PI, 2);
      Serial.println(":");
    }
  }

  nCalls++;
}

void startReadingGyros()
{
  gyISRData.currReading = 0;
  gyISRData.iState = 1; // We start the first send ourselves in order to kick off the interrupts
  gyISRData.currGyControlWord = 0xFF & (GYCONTOLWORD | (GYROCHAN<<GYAD)); // Last byte of the first control word
  SPCR = (1<<SPIE)|SPISETTINGS; // Turn on interrupts
  digitalWrite(CHIPSELECT0, HIGH); // Select a gyro
  SPDR = 0xFF & ((GYCONTOLWORD | (GYROCHAN<<GYAD)) >> 8); // Write the first part of the first control word
}

void getGyroVals()
{
  while(gyISRData.iState != 16) { // Wait for the end of the gyro reading state machine
  }
  while (!(SPSR & (1<<SPIF))) {   // Wait for the end of the last transmission
  }
  gyISRData.gyroReading[7] = gyISRData.currReading | SPDR; // Save the data
  digitalWrite(CHIPSELECT0, LOW); // Deselect the last gyro
}

ISR(SIG_SPI)
{
  if((gyISRData.iState & 0x01) == 0) { // Are we about to read from a new gyro?
    digitalWrite(CHIPSELECT0, LOW);
    digitalWrite(CHIPSELECT0, HIGH);
  }
  SPDR = 0xFF & gyISRData.currGyControlWord; // Write data to gyro
  gyISRData.currReading |= SPDR; // Read buffered data
  if((gyISRData.iState & 0x01) == 0) { // If a read has completed (i.e. we're on a new gyro)
    gyISRData.currGyControlWord >>= 8;
    gyISRData.gyroReading[(gyISRData.iState >> 1)-1] = gyISRData.currReading;  // Then save the data
  } else {
    if(gyISRData.iState == 15) { // End of reading gyros
      SPCR = (0<<SPIE)|SPISETTINGS; // Turn off interrupts
    } else if(gyISRData.iState < 7) {
      gyISRData.currGyControlWord = SWAPBYTES(GYCONTOLWORD | (GYROCHAN<<GYAD)); // Read from gyro channels
    } else {
      gyISRData.currGyControlWord = SWAPBYTES(GYCONTOLWORD | (ACCELCHAN<<GYAD)); // Read from accelerometer channels
    }
  }
  gyISRData.currReading <<= 8;
  gyISRData.iState++;
}

void findGyroZero()
{
  startReadingGyros();
  getGyroVals();
  startReadingGyros();
  getGyroVals();

  for(byte i = 0; i < 4; i++) {
    digitalWrite(CHIPSELECT0, HIGH);
    digitalWrite(CHIPSELECT0, LOW);
    if((gyISRData.gyroReading[i] & 0x0FFF) >= 3500) { // Found fourth gyro
      gyISRData.gyroReading[i] = 0;
      i -= i + 1; // Add another i+1 cycles to push the gyro into position
    }
  }
}

void PrintFloat( float val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  if(val < 0.0) {
    Serial.print('-');
    val = -val;
  }

  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;

    while(precision--)
      mult *=10;

    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;

    unsigned long frac1 = frac;

    while( frac1 /= 10 )
      padding--;

    while(  padding--)
      Serial.print("0");

    Serial.print(frac,DEC) ;
  }
}


/*
struct GyReading {
  int value;
  byte addr;
};

struct GyReading readGyro(unsigned int control)
{
  struct GyReading reading;

  SPDR = control >> 8;
  control &= 0x00FF;
  while (!(SPSR & (1<<SPIF))) { // Wait for the end of the transmission
  }
  reading.addr = SPDR;
 
  SPDR = control;
  reading.value = (int)(reading.addr & 0x0F) << 8;
  reading.addr = reading.addr >> 4;
  while (!(SPSR & (1<<SPIF))) { // Wait for the end of the transmission
  }
  reading.value |= SPDR;

  return reading;
}

void PrintByte(byte bin)
{
  for(byte a = 0; a < 8; a++) {
    Serial.print((bin & 0x80)==0x80, BIN);
    bin <<= 1;
  }
}
*/
