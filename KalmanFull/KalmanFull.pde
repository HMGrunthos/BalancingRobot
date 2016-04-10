// #include <Servo.h>

#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0

// #define SERVOPIN 9

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

#define SERVOMIN 500
#define SERVOMAX 2000
#define SERVORANGE ((SERVOMAX-SERVOMIN)/2)
#define SERVOMID (SERVORANGE+SERVOMIN)

byte vChan;
struct GyISRData {
  volatile byte iState;
  volatile unsigned int currReading;
  unsigned int currGyControlWord;
  int gyroReading[8];
} gyISRData;

#define dt  (60/(float)34667)
#define gyroNoiseVar SQR(6.368*0.09766*M_PI/180)
#define accelNoiseVar (1.0)
#define rateQ (175.0)
#define biasQ (50.0)

float M00 = 0.05;
float M01 = 0;
float M02 = 0;
float M11 = 0.1;
float M12 = 0;
float M22 = 0.1;

float x_est0 = 0;
float x_est1 = 0;
float x_est2 = 0;

unsigned long int mStartTime = 0;
unsigned long int nUpdates = 0;

// Servo servo;

void setup()
{
  byte clr;

  Serial.begin(115200);

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

  Serial.println("Gyro init complete.");
/*
  byte oldSREG = SREG;
  cli();
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(CS11);
    ICR1 = 20000;
    OCR1A = SERVOMID;
  SREG = oldSREG;
  pinMode(SERVOPIN, OUTPUT);
  */
  // servo.attach(SERVOPIN, 585, 2527);
  // servo.write(90);

 // Serial.println("Servo init complete.");

  startReadingGyros();
}

void loop()
{
  getGyroVals();

  float sensorReadings[7];
  sensorReadings[0] = (gyISRData.gyroReading[0] & 0x0FFF) - 1651;
  sensorReadings[1] = (gyISRData.gyroReading[1] & 0x0FFF) - 1349;
  sensorReadings[2] = (gyISRData.gyroReading[2] & 0x0FFF) - 1455;
  sensorReadings[3] = ((gyISRData.gyroReading[4] & 0x0FFF) - 2048) * 0.0017044885;
  sensorReadings[4] = ((gyISRData.gyroReading[5] & 0x0FFF) - 2048) * 0.0017044885;
  sensorReadings[5] = ((gyISRData.gyroReading[6] & 0x0FFF) - 2048) * 0.004256858;
  sensorReadings[6] = ((gyISRData.gyroReading[7] & 0x0FFF) - 2048) * 0.004256858;

  //float acc_angle = -atan2(sensorReadings[0], sensorReadings[2]);
  //float gyro_rate = sensorReadings[6];
  float acc_angle = -atan2(sensorReadings[1], sensorReadings[2]);
  float gyro_rate = sensorReadings[4];
  //float acc_angle = -atan2(sensorReadings[0], sensorReadings[1]);
  //float gyro_rate = sensorReadings[1];

  if(!mStartTime) {
    mStartTime = millis();
  } else if((millis() - mStartTime) > (unsigned long int)60000) {
    Serial.print("Elapsed updates :");
    Serial.print(nUpdates);
    Serial.println(":");
    Serial.println();
    Serial.println((int)(x_est0*180/M_PI));
    Serial.println((int)(x_est1*180/M_PI));
    Serial.println((int)(x_est2*180/M_PI));
    nUpdates = 0;
    mStartTime = millis();
  }
  nUpdates++;

  startReadingGyros();

  float sigma[9];

  float rQ2 = rateQ*dt*dt;
  float rQ3 = 0.5*rQ2*dt;
  float rQ4 = 0.5*rQ3*dt;	
  float bQ4 = 0.25*biasQ*dt*dt*dt*dt;

  float x_pred0 = x_est0 + dt*x_est1;
  float x_pred1 = x_est1;
  float x_pred2 = x_est2;

  sigma[2] = M00 + accelNoiseVar;
  sigma[1] = 2*M12 + M22 + M11 + gyroNoiseVar;
  sigma[0] = 1/(sigma[2]*sigma[1] - M01*M01 - M02*(M02 + 2*M01));
  sigma[5] = M01+M02;
  sigma[7] = -M02*sigma[5] + (M12+M22)*sigma[2];
  sigma[8] =  M02*sigma[1] - (M12+M22)*sigma[5];
  sigma[2] =  M01*sigma[5] - (M12+M11)*sigma[2];
  sigma[4] = (M11+M12)*sigma[5];
  sigma[6] = sigma[5]*sigma[5];

  float angleError = acc_angle - x_pred0;
  float rateError = gyro_rate - x_pred1 - x_pred2;

  x_est0 = x_pred0  + ((M00*sigma[1] - sigma[6])*angleError
                    + accelNoiseVar*sigma[5]*rateError)*sigma[0];
  x_est1 = x_pred1  + ((M01*sigma[1] - sigma[4])*angleError
                    - sigma[2]*rateError)*sigma[0];
  x_est2 = x_pred2  + (sigma[8]*angleError
                    + sigma[7]*rateError)*sigma[0];

  sigma[2] = sigma[2]*sigma[0];
  sigma[3] = 1 + sigma[2];
  sigma[4] = (-M01*sigma[1] + sigma[4])*sigma[0];
  sigma[5] = -accelNoiseVar*sigma[5]*sigma[0];
  sigma[6] = 1 + (sigma[6] - M00*sigma[1])*sigma[0];

  float M12n = sigma[4]*M02 + sigma[3]*M12 + sigma[2]*M22;
  float M11n = sigma[4]*M01 + sigma[3]*M11 + sigma[2]*M12;
  float M01n = sigma[6]*M01 + sigma[5]*(M11 + M12) + dt*M11n;
  float M22n = (-sigma[8]*M02 - sigma[7]*(M12+M22))*sigma[0] + M22 + rQ4;

  M00 = sigma[6]*M00 + sigma[5]*(M01 + M02)+ dt*(sigma[4]*M00 + sigma[3]*M01 + sigma[2]*M02) + dt*M01n + rQ4;
  M01 = M01n + rQ3;
  M02 = sigma[6]*M02 + (M12 + M22)*sigma[5] + dt*M12n;
  M11 = M11n + rQ2;
  M12 = M12n;
  M22 = M22n;

  OCR1A = SERVOMID+(SERVORANGE)*x_est0/(M_PI/2);
  // if(!(nUpdates%30)) {
    // servo.write((float)(x_est0*180/M_PI + 90));
  // }
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
