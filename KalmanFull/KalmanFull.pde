#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0

#define SQR(a) ((a)*(a))

#define dt  0.005
#define gyroNoiseVar 0.1
#define accelNoiseVar 1.0
#define rateQ 175
#define biasQ 100

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

void setup()
{
  Serial1.begin(115200);

  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(CHIPSELECT0, OUTPUT);
  digitalWrite(CHIPSELECT0, LOW); // disable device

  delay(500);
  Serial1.println("Init.");
}

void loop()
{
  if(!mStartTime) {
    mStartTime = millis();
  } else if((millis() - mStartTime) > (unsigned long int)10000) {
    Serial1.print("Elapsed updates :");
    Serial1.print(nUpdates);
    Serial1.println(":");
    mStartTime = millis();
    nUpdates = 0;
    // toggle = !toggle;
  }
  nUpdates++;

  float sigma[9];

  float acc_angle;
  float gyro_rate;

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
}
