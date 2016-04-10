#include <avr/eeprom.h>

#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0

#define MOTRORPWM1 6
#define MOTRORPWM2 5
#define MOTRORDIR1 18
#define MOTRORDIR2 17

#define BUTTON 19

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

#define DTCLOCKFACTOR 6

struct GyISRData {
  volatile byte vChan;
  volatile byte iState;
  volatile unsigned int currReading;
  unsigned int currGyControlWord;
  int gyroReading[8];
} gyISRData;

volatile byte tock = 0;

#define dt 0.004896 // (32*510*DTCLOCKFACTOR)/20000000
#define gyroNoiseVar SQR(6.368*0.09766*M_PI/180)
#define accelNoiseVar (0.1)
#define rateQ (175.0)
#define biasQ (5.0)

float M00 = 0.05;
float M01 = 0;
float M02 = 0;
float M11 = 0.1;
float M12 = 0;
float M22 = 0.1;

float x_est0 = 0;
float x_est1 = 0;
float x_est2 = 0;

float pGain;
float iGain;
float dGain;
float eGain;
float accNull0; // = 0.489;
float accNull1; // = 0.513;
float accNull2; // = 0.489;

boolean active = false;

unsigned long int mStartTime = 0;
unsigned long int nUpdates = 0;

void setup()
{
  byte clr;

  Serial1.begin(115200);

  pinMode(DATAIN, INPUT); // SPI
  pinMode(DATAOUT, OUTPUT);
  pinMode(SPICLOCK, OUTPUT);

  pinMode(CHIPSELECT0, OUTPUT); // SPI gyro chip select signal
  digitalWrite(CHIPSELECT0, LOW);

  pinMode(MOTRORDIR1, OUTPUT);
  pinMode(MOTRORDIR2, OUTPUT);
  digitalWrite(MOTRORDIR1, LOW);
  digitalWrite(MOTRORDIR2, LOW);
  pinMode(MOTRORPWM1, OUTPUT);
  pinMode(MOTRORPWM2, OUTPUT);
  digitalWrite(MOTRORPWM1, LOW);
  digitalWrite(MOTRORPWM2, LOW);

  pinMode(BUTTON, INPUT);

  SPSR = (0<<SPI2X);
  SPCR = (0<<SPIE)|SPISETTINGS;
  clr = SPSR;
  clr = SPDR;

  eeprom_read_block(&pGain, (unsigned char *)0, 4);
  eeprom_read_block(&iGain, (unsigned char *)4, 4);
  eeprom_read_block(&dGain, (unsigned char *)8, 4);
  eeprom_read_block(&eGain, (unsigned char *)12, 4);
  eeprom_read_block(&accNull0, (unsigned char *)16, 4);
  eeprom_read_block(&accNull1, (unsigned char *)20, 4);
  eeprom_read_block(&accNull2, (unsigned char *)24, 4);
  eGain = 0.015;
/*
  pGain *= dt;
  iGain *= dt;
  dGain *= dt;
  eGain *= dt;
*/
  findGyroZero();
  startReadingGyros();
  getGyroVals(false);

  Serial1.println("Gyro init complete.");

  byte oldSREG = SREG;
  cli();
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20); // Phase correct non inverting
    TCCR2B = _BV(CS21) | _BV(CS20); // Div 32
    OCR2A = 0; // Motor off
    OCR2B = 0; // Motor off
    TIMSK2 = (1 << TOIE2); // Interrupt on
  SREG = oldSREG;

  startReadingGyros();

  Serial1.println("Waiting for button press...");
}

void loop()
{
  static float pidOutput = 0;
  static float iTerm = 0;
  static float pokeTerm = 0;
  
  float sensorReadings[7];

  if(Serial1.available()) {
    parseSerial();
    return;
  }

  do {
    int fullScale;

    if(getGyroVals(true)) {
      fullScale = (gyISRData.gyroReading[3] & 0x0FFF);
      sensorReadings[0] = (gyISRData.gyroReading[0] & 0x0FFF) - fullScale*accNull0;
      sensorReadings[1] = (gyISRData.gyroReading[1] & 0x0FFF) - fullScale*accNull1;
      sensorReadings[2] = (gyISRData.gyroReading[2] & 0x0FFF) - fullScale*accNull2;
      sensorReadings[3] = ((gyISRData.gyroReading[4] & 0x0FFF) - 2048) * 0.0017044885;
      sensorReadings[4] = ((gyISRData.gyroReading[5] & 0x0FFF) - 2048) * 0.0017044885;
      sensorReadings[5] = ((gyISRData.gyroReading[6] & 0x0FFF) - 2048) * 0.004256858;
      sensorReadings[6] = ((gyISRData.gyroReading[7] & 0x0FFF) - 2048) * 0.004256858;
      startReadingGyros();
    }
  } while(tock < DTCLOCKFACTOR);
  tock = 0;

  float acc_angle = -atan2(sensorReadings[1], sensorReadings[2]);
  float gyro_rate = sensorReadings[4];

  if(!mStartTime) {
    mStartTime = millis();
  } else if((millis() - mStartTime) > (unsigned long int)5000) {
    Serial1.print("Elapsed updates :");
    Serial1.print(nUpdates);
    Serial1.println(":");
    Serial1.print("Angle est :");
    Serial1.print(x_est0*180/M_PI);
    Serial1.print(": Rate est :");
    Serial1.print(x_est1*180/M_PI);
    Serial1.print(": Bias est :");
    Serial1.print(x_est2*180/M_PI);
    Serial1.print(": Drive strength :");
    Serial1.print(255*pidOutput);
    Serial1.print(": Innet I. :");
    Serial1.print(iTerm);
    Serial1.print(": Outer I :");
    Serial1.print(pokeTerm);
    Serial1.print(": Sum Outer :");
    Serial1.print((pokeTerm+(0.15*dt*pidOutput)));
    Serial1.println(":");
    nUpdates = 0;
    mStartTime = millis();
  }
  nUpdates++;

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

  // float pTerm = (eGain*M_PI/180) - x_est0;

  float pTerm = (pokeTerm+(0*dt*pidOutput)) - x_est0;
  float dTerm = -x_est1;
  iTerm += pTerm;
  
  iTerm = iTerm > 25 ? 25 : (iTerm < -25 ? -25 : iTerm);

  pidOutput = pGain*1*pTerm + iGain*1*iTerm + dGain*1*dTerm;
  
  pidOutput = pidOutput > 1 ? 1 : (pidOutput < -1 ? -1 : pidOutput);
  
  // Serial1.println("Bef");
  // Serial1.println(pokeTerm);
  pokeTerm += eGain*dt*pidOutput;
  
  pokeTerm = pokeTerm > 4 ? 4 : (pokeTerm < -4 ? -4 : pokeTerm);
  
  // Serial1.println("Aft");
  // Serial1.println(pokeTerm);

  if(active) {
    OCR2A = min(255, 255*fabs(pidOutput));
    OCR2B = min(255, 255*fabs(pidOutput));

    if(pidOutput > 0) {
      digitalWrite(MOTRORDIR1, HIGH);
      digitalWrite(MOTRORDIR2, HIGH);
    } else {
      digitalWrite(MOTRORDIR1, LOW);
      digitalWrite(MOTRORDIR2, LOW);
    }
  } else {
    if(digitalRead(BUTTON) == LOW) {
      Serial1.println("Got button press.");
      active = true;
      iTerm = 0;
      pokeTerm = 0;
    }
  }
}

void parseSerial()
{
  enum StringState {
    PChar,
    PTermS,
    PTermP1,
    PTerm0,
    PTermN1,
    IChar,
    ITermS,
    ITerm0,
    ITermN1,
    ITermN2,
    DChar,
    DTermS,
    DTerm0,
    DTermN1,
    DTermN2,
    BChar,
    EBiasS,
    EBias0,
    EBiasN1,
    EBiasN2,
    EndChar,
    EndState
  } stringReadState = PChar;
  boolean inputValid = true;
  float signMult;

  int readVal = Serial1.read();
  if(readVal == 'S') {
    Serial1.print("PGain:");
    Serial1.print(pGain);
    Serial1.print(": IGain:");
    Serial1.print(iGain);
    Serial1.print(": DGain:");
    Serial1.print(dGain);
    Serial1.print(": EGain:");
    Serial1.print(eGain);
    Serial1.print(": accNull0:");
    Serial1.print(accNull0);
    Serial1.print(": accNull1:");
    Serial1.print(accNull1);
    Serial1.print(": accNull2:");
    Serial1.print(accNull2);
    Serial1.println(":");
  } else if(readVal == 'p') {
    stringReadState = (StringState)(((int)stringReadState) + 1);
  }

  while(inputValid && stringReadState < EndState) {
    readVal = Serial1.read();
    switch(stringReadState) {
      case PChar:
          if(readVal != 'p') {
            inputValid = false;
          }
          stringReadState = (StringState)(((int)stringReadState) + 1);
        continue;
      case IChar:
          if(readVal != 'i') {
            inputValid = false;
          }
          stringReadState = (StringState)(((int)stringReadState) + 1);
        continue;
      case DChar:
          if(readVal != 'd') {
            inputValid = false;
          }
          stringReadState = (StringState)(((int)stringReadState) + 1);
        continue;
      case BChar:
          if(readVal != 'b') {
            inputValid = false;
          }
          stringReadState = (StringState)(((int)stringReadState) + 1);
        continue;
      case EndChar:
          if(readVal != 'x') {
            inputValid = false;
          }
          stringReadState = (StringState)(((int)stringReadState) + 1);
        continue;
      case PTermS:
      case ITermS:
      case DTermS:
      case EBiasS:
        if(readVal != '+' && readVal != '-') {
          inputValid = false;
          continue;
        }
        break;
      case PTermP1:
      case PTerm0:
      case PTermN1:
      case ITerm0:
      case ITermN1:
      case ITermN2:
      case DTerm0:
      case DTermN1:
      case DTermN2:
      case EBias0:
      case EBiasN1:
      case EBiasN2:
        readVal -= '0';
        if(readVal < 0 || readVal > 9) {
          inputValid = false;
          continue;
        }
        break;
    }
    switch(stringReadState) {
      case PTermS:
      case ITermS:
      case DTermS:
      case EBiasS:
        if(readVal == '-') {
          signMult = -1;
        } else {
          signMult = 1;
        }
        break;
    }
    switch(stringReadState) {
      case PTermP1:
        pGain = readVal*10;
        break;
      case PTerm0:
        pGain += readVal*1;
        break;
      case PTermN1:
        pGain += readVal/(float)10;
        pGain *= signMult;
        break;
      case ITerm0:
        iGain = readVal*1;
        break;
      case ITermN1:
        iGain += readVal/(float)10;
        break;
      case ITermN2:
        iGain += readVal/(float)100;
        iGain *= signMult;
        break;
      case DTerm0:
        dGain = readVal*1;
        break;
      case DTermN1:
        dGain += readVal/(float)10;
        break;
      case DTermN2:
        dGain += readVal/(float)100;
        dGain *= signMult;
        break;
      case EBias0:
        eGain = readVal*1;
        break;
      case EBiasN1:
        eGain += readVal/(float)10;
        break;
      case EBiasN2:
        eGain += readVal/(float)100;
        eGain *= signMult;
        break;
    }
    stringReadState = (StringState)(((int)stringReadState) + 1);
  }
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

boolean getGyroVals(boolean checkTime)
{
  while(gyISRData.iState != 16) { // Wait for the end of the gyro reading state machine
    if(checkTime && (tock >= DTCLOCKFACTOR)) {
      return 0;
    }
  }
  while (!(SPSR & (1<<SPIF))) {   // Wait for the end of the last transmission
  }
  gyISRData.gyroReading[7] = gyISRData.currReading | SPDR; // Save the data
  digitalWrite(CHIPSELECT0, LOW); // Deselect the last gyro
  return 1;
}

void findGyroZero()
{
  gyISRData.vChan = ACCELCHAN;
  startReadingGyros();
  getGyroVals(false);
  startReadingGyros();
  getGyroVals(false);
  gyISRData.vChan = V3V3CHAN;

  for(byte i = 0; i < 4; i++) {
    digitalWrite(CHIPSELECT0, HIGH);
    digitalWrite(CHIPSELECT0, LOW);
    if((gyISRData.gyroReading[i] & 0x0FFF) >= 3500) { // Found fourth gyro
      gyISRData.gyroReading[i] = 0;
      i -= i + 1; // Add another i+1 cycles to push the gyro into position
    }
  }
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
      gyISRData.currGyControlWord = SWAPBYTES(GYCONTOLWORD | (gyISRData.vChan<<GYAD)); // Read from accelerometer channels
    } else { // gyISRData.iState == 15
      SPCR = (0<<SPIE)|SPISETTINGS; // Turn off interrupts
    }
  }
  gyISRData.currReading <<= 8;
  gyISRData.iState++;
}

ISR(SIG_OVERFLOW2)
{
  tock++;
}
