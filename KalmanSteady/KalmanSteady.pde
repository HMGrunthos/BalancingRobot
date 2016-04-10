#include <FrequencyTimer2.h>

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

#define SERVOMIN 500
#define SERVOMAX 2000
#define SERVORANGE ((SERVOMAX-SERVOMIN)/2)
#define SERVOMID (SERVORANGE+SERVOMIN)

#define DT  0.005

float kGain[3][2] = {{0.0017615, 0.0027167}, {0.00031415, 0.9112}, {-0.00027004, 1.2451e-6}};
float x_est0 = 0;
float x_est1 = 0;
float x_est2 = 0;

struct GyISRData {
  volatile byte iState;
  volatile int currReading;
  unsigned int currGyControlWord;
  int gyroReading[8];
} gyISRData;

volatile byte tock = 0;

void setup()
{
  byte clr;

  // Serial.begin(115200);

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

  // Serial.println("Gyro init complete.");

  byte oldSREG = SREG;
  cli();
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(CS11);
    ICR1 = 20000;
    OCR1A = SERVOMID;
  SREG = oldSREG;
  pinMode(SERVOPIN, OUTPUT);

  // Serial.println("Servo init complete.");

  FrequencyTimer2::setPeriod(10000);
  FrequencyTimer2::setOnOverflow(tick);
  
  // Serial.println("Heatbeat init complete.");
  
  startReadingGyros();
}

void loop()
{
  static long int lCount = 0;
  float sensorReadings[7];

  do {
    getGyroVals();
    sensorReadings[0] = (gyISRData.gyroReading[0] & 0x0FFF) - 1651;
    sensorReadings[1] = (gyISRData.gyroReading[1] & 0x0FFF) - 1349;
    sensorReadings[2] = (gyISRData.gyroReading[2] & 0x0FFF) - 1455;
    sensorReadings[3] = ((gyISRData.gyroReading[4] & 0x0FFF) - 2048) * 0.0017044885;
    sensorReadings[4] = ((gyISRData.gyroReading[5] & 0x0FFF) - 2048) * 0.0017044885;
    sensorReadings[5] = ((gyISRData.gyroReading[6] & 0x0FFF) - 2048) * 0.004256858;
    sensorReadings[6] = ((gyISRData.gyroReading[7] & 0x0FFF) - 2048) * 0.004256858;
    startReadingGyros();
  } while(tock == 0);
  tock = 0;

  float acc_angle = -atan2(sensorReadings[0], sensorReadings[2]);
  float gyro_rate = sensorReadings[6];
  //float acc_angle = -atan2(sensorReadings[1], sensorReadings[2]);
  //float gyro_rate = sensorReadings[4];
  //float acc_angle = -atan2(sensorReadings[1], sensorReadings[0]);
  //float gyro_rate = sensorReadings[3];
  //float gyro_rate = -sensorReadings[5];

  float a = acc_angle - x_est0 - DT*x_est1;
  float b = gyro_rate - x_est1 - x_est2;

  x_est0 = x_est0 + DT*x_est1 + kGain[0][0]*a + kGain[0][1]*b;
  x_est1 = x_est1 +             kGain[1][0]*a + kGain[1][1]*b;
  x_est2 = x_est2 +             kGain[2][0]*a + kGain[2][1]*b;

  OCR1A = SERVOMID+(SERVORANGE)*(x_est0)/(M_PI/2);
  //OCR1A = SERVOMID+(SERVORANGE)*(x_est0-M_PI/2)/(M_PI/2);
}

void tick()
{
  tock++;
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
