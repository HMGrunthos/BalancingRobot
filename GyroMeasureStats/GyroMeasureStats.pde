// include <Servo.h>

#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0

//#define SERVOPIN 9

#define SPISPEED (0x03 & (0))
#define SPISETTINGS ((1<<SPE)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA)|SPISPEED)

#define GYCD 4
#define GYAD 10
#define GYCONTOLBASE (0x0300)
#define GYCONTOLWORD (GYCONTOLBASE | (0x8000) | (1<<GYCD))

#define GYROCHAN 0
#define ACCELCHAN 3
#define V3V3CHAN 2

#define SWAPBYTES(a) ((0x00 | ((a) << 8)) | ((a) >> 8))
#define SQR(a) ((a)*(a))

struct GyISRData {
  volatile byte iState;
  volatile unsigned int currReading;
  unsigned int currGyControlWord;
  unsigned int gyroReading[8];
} gyISRData;

// Servo servo;

// #define Serial1 Serial

byte vChan;
double sensorReadings[8][2];

void setup()
{
  byte clr;

  Serial1.begin(115200);

  pinMode(DATAIN, INPUT); // SPI
  pinMode(DATAOUT, OUTPUT);
  pinMode(SPICLOCK, OUTPUT);

  pinMode(CHIPSELECT0, OUTPUT); // SPI gyro chip select signal
  digitalWrite(CHIPSELECT0, LOW);

  SPSR = (0<<SPI2X);
  SPCR = (0<<SPIE)|SPISETTINGS;
  clr = SPSR;
  clr = SPDR;

  findGyroZero();
  startReadingGyros();
  getGyroVals();

  Serial1.println("Gyro init complete.");

  //servo.attach(SERVOPIN, 585, 2527);
  //servo.write(90);

  //Serial1.println("Servo init complete.");

  memset(sensorReadings, 0, sizeof(sensorReadings));
}

void loop()
{
  static unsigned long long int nCalls = 1;
  static unsigned long int nAddrErrors = 0;

  startReadingGyros();
  getGyroVals();

  for(byte i = 0; i < 8; i++) {
    if((i <= 2 && ((0xFF00 & gyISRData.gyroReading[i]) >> 12) == 3)
        || (i >= 4 && ((0xFF00 & gyISRData.gyroReading[i]) >> 12) == 0)
        || (i == 3 && ((0xFF00 & gyISRData.gyroReading[i]) >> 12) == 2)
        || (i == 3 && ((0xFF00 & gyISRData.gyroReading[i]) >> 12) == 3)) {
      double val = (gyISRData.gyroReading[i] & 0x0FFF);
      double delta = val - sensorReadings[i][0];
      sensorReadings[i][0] = sensorReadings[i][0] + delta/nCalls;
      sensorReadings[i][1] = sensorReadings[i][1] + delta*(val - sensorReadings[i][0]);
    } else {
      nAddrErrors++;
    }
  }

  //if(!(nCalls % 1000) || ((gyISRData.gyroReading[3] & 0x0FFF) < 4000)) {
  if(!(nCalls % 1000)) {
    Serial1.println();
    Serial1.print("Number of address errors :");
    Serial1.print(nAddrErrors);
    Serial1.println(":");
    for(int i = 0; i < 8; i++) {
      double mu = sensorReadings[i][0];
      double sigma = sqrt(sensorReadings[i][1]/(double)(nCalls-1));
      //double sigma = sensorReadings[i][1];
      Serial1.print("Address:");
      Serial1.print((0xFF00 & gyISRData.gyroReading[i]) >> 12);
      Serial1.print(": Reading mu:");
      Serial1.print(mu);
      //Serial1.print("/");
      // Serial1.print((long int)(mu*1000+0.5));
      Serial1.print(": sigma:");
      //Serial1.print((long int)(sensorReadings[i][1]/(double)nCalls+0.5));
      Serial1.print(sigma);
      //Serial1.print("/");
      //Serial1.print((long int)(mu*mu+0.5));
      //Serial1.print((long int)(sigma*1000+0.5));
      Serial1.println(":");
    }
  }

  delay(5);

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
    if(gyISRData.iState < 7) {
      //Serial.println("Asking Gyros");
      gyISRData.currGyControlWord = SWAPBYTES(GYCONTOLWORD | (GYROCHAN<<GYAD)); // Read from gyro channels
    } else if(gyISRData.iState < 13) {
      //Serial.println("Asking Accels");
      gyISRData.currGyControlWord = SWAPBYTES(GYCONTOLWORD | (ACCELCHAN<<GYAD)); // Read from accelerometer channels
    } else if(gyISRData.iState == 13) {
      //Serial.println("Asking VInput");
      gyISRData.currGyControlWord = SWAPBYTES(GYCONTOLWORD | (vChan<<GYAD)); // Read from accelerometer channels
    } else { // gyISRData.iState == 15
      SPCR = (0<<SPIE)|SPISETTINGS; // Turn off interrupts
    }
  }
  gyISRData.currReading <<= 8;
  gyISRData.iState++;
}

void findGyroZero()
{
  vChan = ACCELCHAN;
  startReadingGyros();
  getGyroVals();
  startReadingGyros();
  getGyroVals();
  vChan = V3V3CHAN;

  for(byte i = 0; i < 4; i++) {
    digitalWrite(CHIPSELECT0, HIGH);
    digitalWrite(CHIPSELECT0, LOW);
    if((gyISRData.gyroReading[i] & 0x0FFF) >= 3500) { // Found fourth gyro
      gyISRData.gyroReading[i] = 0;
      i -= i + 1; // Add another i+1 cycles to push the gyro into position
    }
  }
}

/*
void PrintDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  if(val < 0.0) {
    Serial.print('-');
    val = -val;
  }

  Serial.print((long int)fabs(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;

    while(precision--)
      mult *=10;

    if(val >= 0)
      frac = (val - (long int)val) * mult;
    else
      frac = ((long int)val- val) * mult;

    unsigned long int frac1 = frac;

    while( frac1 /= 10 )
      padding--;

    while(  padding--)
      Serial.print("0");

    Serial.print(frac,DEC) ;
  }
}

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
