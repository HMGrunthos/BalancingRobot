#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0

#define SQR(a) ((a)*(a))

#define dt 0.004896
#define gyroNoiseVar 36.0
#define accelNoiseVar 0.2
#define rateQ 935
#define biasQ 10
#define scaleQ 10

float P00 = 0.005;
float P01 = 0;
float P02 = 0;
float P03 = 0;
float P11 = 0.0001;
float P12 = 0;
float P13 = 0;
float P22 = 25;
float P23 = 0;
float P33 = 25;

float x_est0 = 0;
float x_est1 = 0;
float x_est2 = 0;
float x_est3 = 586.69;

unsigned long int mStartTime = 0;
unsigned long int nUpdates = 0;

#define DT2 (dt*dt)

#define DT6SCALE ((dt*dt*dt*dt*dt*dt)*scaleQ)
#define DT5SCALE ((dt*dt*dt*dt*dt)*scaleQ)
#define DT4SCALE ((dt*dt*dt*dt)*scaleQ)
#define DT3SCALE ((dt*dt*dt)*scaleQ)
#define DT2SCALE ((dt*dt)*scaleQ)
#define DT6RATEQ ((dt*dt*dt*dt*dt*dt)*rateQ)
#define DT4RATEQ ((dt*dt*dt*dt)*rateQ)
#define DT3RATEQ ((dt*dt*dt)*rateQ)
#define DT2RATEQ ((dt*dt)*rateQ)
#define DT1RATEQ (dt*rateQ)
#define DT5BIASQ ((dt*dt*dt*dt*dt)*biasQ)
#define DT4BIASQ ((dt*dt*dt*dt)*biasQ)
#define DT3BIASQ ((dt*dt*dt)*biasQ)
#define DT2BIASQ ((dt*dt)*biasQ)

// #define Serial1 Serial

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

  float acc_angle;
  float gyro_rate;
 
  float xest1to2 = (x_est1*x_est1);
  float xest3to2 = (x_est3*x_est3);

  float P01to2 = (P01*P01);
  float P02to2 = (P02*P02);
  float P03to2 = (P03*P03);
  float P12to2 = (P12*P12);
  float P13to2 = (P13*P13);

  float sigma[24];
  
  sigma[1] = x_est3*DT3RATEQ*x_est1*P03;
  sigma[2] = x_est3*dt*P11*x_est1*P03;
  sigma[3] = DT4RATEQ*x_est3*x_est1*P13;
  sigma[4] = x_est3*DT3RATEQ*P02;
  sigma[5] = x_est3*P01*x_est1*P03;
  sigma[6] = P00*xest1to2*DT2SCALE;
  sigma[7] = P00*x_est3*x_est1*P13;
  sigma[8] = P00*xest3to2*DT2RATEQ;
  sigma[9] = DT4RATEQ*x_est1*P23;
  sigma[10] = DT4RATEQ*x_est3*P12;
  sigma[11] = DT4SCALE*P11*xest1to2;
  sigma[12] = DT3SCALE*P01*xest1to2;
  sigma[13] = DT3RATEQ*P01*xest3to2;
  sigma[14] = DT6RATEQ*xest1to2*scaleQ;
  sigma[15] = DT4RATEQ*xest1to2*P33;
  sigma[16] = DT4RATEQ*P11*xest3to2;
  sigma[17] = dt*P01*x_est3*x_est1*P13;
  sigma[18] = accelNoiseVar*x_est1*P23;
  sigma[19] = accelNoiseVar*x_est3*P12;
  sigma[20] = accelNoiseVar*DT2BIASQ;
  sigma[21] = DT4RATEQ*gyroNoiseVar;
  sigma[22] = DT2*P11*gyroNoiseVar;
  sigma[23] = DT4RATEQ*P22+8*P00*x_est3*P12+DT6RATEQ*biasQ+8*dt*P01*P22+4*P00*DT2BIASQ+8*dt*P01*gyroNoiseVar+4*DT4BIASQ*P11+8*DT3BIASQ*P01+4*DT2*P11*P22-8*P02*dt*P12-8*x_est3*P01*P02+4*P00*xest3to2*P11-4*xest3to2*P01to2-4*P02to2+8*dt*P01*x_est3*P12+4*accelNoiseVar*P22-4*DT2*P12to2+4*P00*gyroNoiseVar+4*P00*P22-8*x_est3*dt*P11*P02+4*accelNoiseVar*gyroNoiseVar+(-4*DT2*P13to2-8*P03*dt*P13+4*DT2*P11*P33+DT4RATEQ*P33+8*dt*P01*P33+4*accelNoiseVar*P33+4*P00*DT2SCALE+4*DT4SCALE*P11+8*DT3SCALE*P01+4*P00*P33+DT6RATEQ*scaleQ-4*P03to2+4*accelNoiseVar*DT2SCALE)*xest1to2+(-8*P02*P03-4*P03*x_est3*DT3RATEQ-8*dt*P12*P03+8*dt*P13*x_est3*P01+8*DT2*P11*P23-8*P02*dt*P13-8*DT2*P12*P13+8*accelNoiseVar*P23+8*P00*P23+8*x_est3*P13*accelNoiseVar-8*P03*x_est3*dt*P11-2*DT4RATEQ*P13*x_est3-8*P03*x_est3*P01+2*DT4RATEQ*P23+16*dt*P01*P23+8*x_est3*P13*P00)*x_est1-4*sigma[4]+4*sigma[8]-2*sigma[10]+4*sigma[13]+sigma[16]+8*sigma[19]+4*sigma[20]+sigma[21]+4*sigma[22]+4*accelNoiseVar*xest3to2*P11+4*accelNoiseVar*xest3to2*DT2RATEQ;

  float K00 = (-4*P03to2-4*DT2*P13to2+8*dt*P01*P33+4*P00*P33+4*DT2*P11*P33-8*P03*dt*P13)/sigma[23]*xest1to2+(-8*DT2*P12*P13-8*P02*dt*P13-8*dt*P12*P03+8*DT2*P11*P23+8*P00*P23-8*P02*P03+16*dt*P01*P23)/sigma[23]*x_est1+(DT4RATEQ*P22+8*P00*x_est3*P12+DT6RATEQ*biasQ+8*dt*P01*P22+4*P00*DT2BIASQ+8*dt*P01*gyroNoiseVar+4*DT4BIASQ*P11+8*DT3BIASQ*P01+4*DT2*P11*P22+8*sigma[17]-8*P02*dt*P12-8*x_est3*P01*P02+4*P00*xest3to2*P11-4*xest3to2*P01to2-2*sigma[3]-4*P02to2+8*dt*P01*x_est3*P12-4*DT2*P12to2+4*P00*gyroNoiseVar+4*P00*P22-8*x_est3*dt*P11*P02-2*sigma[10]-8*sigma[2]+sigma[16]+2*sigma[9]+4*sigma[11]+sigma[14]+4*sigma[22]+sigma[21]+sigma[15]+4*sigma[13]+8*sigma[12]+4*sigma[8]+8*sigma[7]+4*sigma[6]-4*sigma[1]-8*sigma[5]-4*sigma[4])/sigma[23];
  float K01 = 2*(2*P03+2*dt*P13)*accelNoiseVar/sigma[23]*x_est1+2*(2*x_est3*P01+2*x_est3*dt*P11+x_est3*DT3RATEQ+2*P02+2*dt*P12)*accelNoiseVar/sigma[23];
  float K10 = 2*(2*P01*P33-2*P13to2*dt-2*P03*P13+2*dt*P11*P33+2*DT3SCALE*P11+DT3RATEQ*P33+rateQ*DT5SCALE+2*P01*DT2SCALE)/sigma[23]*xest1to2+2*(4*dt*P11*P23-2*P12*P03-4*P12*dt*P13-1*DT3RATEQ*x_est3*P13+2*DT3RATEQ*P23-2*P02*P13+2*P01*x_est3*P13-2*x_est3*P11*P03-2*x_est3*DT2RATEQ*P03+4*P01*P23)/sigma[23]*x_est1+2*(2*dt*P11*gyroNoiseVar+2*DT3BIASQ*P11+2*P01*P22-2*dt*P12to2+DT3RATEQ*gyroNoiseVar+DT3RATEQ*P22+rateQ*DT5BIASQ-2*x_est3*P11*P02+2*P01*DT2BIASQ+2*P01*x_est3*P12+2*dt*P11*P22+2*P01*gyroNoiseVar-2*x_est3*DT2RATEQ*P02-1*DT3RATEQ*x_est3*P12-2*P02*P12)/sigma[23];
  float K11 = (4*P13*P00+4*P01*dt*P13+4*P13*accelNoiseVar-1*DT4RATEQ*P13-2*DT3RATEQ*P03-4*P01*P03-4*dt*P11*P03)/sigma[23]*x_est1+(-2*DT3RATEQ*P02+4*P01*x_est3*DT3RATEQ+4*x_est3*P11*accelNoiseVar+4*x_est3*P11*P00-1*DT4RATEQ*P12-4*dt*P11*P02+DT4RATEQ*P11*x_est3+4*P01*dt*P12-4*x_est3*P01to2+4*P12*accelNoiseVar+4*P12*P00-4*P01*P02+4*x_est3*DT2RATEQ*P00+4*x_est3*DT2RATEQ*accelNoiseVar)/sigma[23];
  float K20 = -2*(2*P23*P03+2*P23*dt*P13-2*dt*P12*P33-2*DT3SCALE*P12-2*P02*P33-2*P02*DT2SCALE)/sigma[23]*xest1to2-2*(2*P03*x_est3*P12+2*P03*P22+2*dt*P13*P22+P23*x_est3*DT3RATEQ-2*P23*P02+2*P03*DT2BIASQ+2*DT3BIASQ*P13-2*dt*P13*x_est3*P12-4*x_est3*P13*P02-2*P23*dt*P12+2*P23*x_est3*P01+2*P23*x_est3*dt*P11)/sigma[23]*x_est1-2*(2*P22*x_est3*P01-2*dt*P12*gyroNoiseVar+DT5BIASQ*rateQ*x_est3-2*dt*P12to2*x_est3-2*P02*xest3to2*DT2RATEQ-2*P02*x_est3*P12+2*P22*x_est3*dt*P11+P22*x_est3*DT3RATEQ-2*P02*xest3to2*P11+2*xest3to2*P12*P01+2*DT3BIASQ*x_est3*P11-2*P02*gyroNoiseVar-1*DT3RATEQ*P12*xest3to2+2*DT2BIASQ*x_est3*P01)/sigma[23];
  float K21 = (-4*P02*dt*P13-4*dt*P12*P03-4*DT2*P12*P13+4*P00*P23-4*P02*P03+8*dt*P01*P23+4*DT2*P11*P23)/sigma[23]*x_est1+(4*dt*P01*x_est3*P12+DT4RATEQ*P22+4*P00*x_est3*P12+DT6RATEQ*biasQ+8*dt*P01*P22+4*P00*DT2BIASQ+4*DT4BIASQ*P11+8*DT3BIASQ*P01+4*DT2*P11*P22-1*sigma[10]-8*P02*dt*P12-4*x_est3*P01*P02-4*P02to2-4*DT2*P12to2+sigma[9]-2*sigma[4]+4*accelNoiseVar*P22-4*x_est3*dt*P11*P02+4*P00*P22+4*sigma[20]+4*sigma[19]+4*sigma[18])/sigma[23];
  float K30 = -2*(2*P02*P33+P33*x_est3*DT3RATEQ+2*P02*DT2SCALE+DT5SCALE*x_est3*rateQ+2*dt*P12*P33+2*DT3SCALE*P12-2*P23*P03+2*P33*x_est3*P01-2*P23*dt*P13-2*dt*P13to2*x_est3-2*P03*x_est3*P13+2*DT2SCALE*x_est3*P01+2*DT3SCALE*x_est3*P11+2*P33*x_est3*dt*P11)/sigma[23]*x_est1-2*(2*P23*x_est3*P01-4*P03*x_est3*P12+2*P23*dt*P12-2*dt*P13*P22+2*xest3to2*P13*P01-2*dt*P13*gyroNoiseVar-2*P03*xest3to2*P11+2*x_est3*P13*P02+P23*x_est3*DT3RATEQ-2*P03*DT2BIASQ-2*DT3BIASQ*P13-2*P03*gyroNoiseVar+2*P23*P02-2*P03*P22-1*DT3RATEQ*P13*xest3to2-2*dt*P13*x_est3*P12+2*P23*x_est3*dt*P11-2*P03*xest3to2*DT2RATEQ)/sigma[23];
  float K31 = (DT6SCALE*rateQ+P33*DT4RATEQ+4*P00*P33+4*P33*accelNoiseVar-4*P03to2-4*DT2*P13to2-8*P03*dt*P13+8*dt*P01*P33+4*DT2*P11*P33+8*DT3SCALE*P01+4*DT4SCALE*P11+4*DT2SCALE*P00+4*DT2SCALE*accelNoiseVar)/sigma[23]*x_est1+(DT4RATEQ*P23-4*P02*dt*P13+4*accelNoiseVar*P23-4*DT2*P12*P13-4*P02*P03+4*dt*P13*x_est3*P01+8*dt*P01*P23+4*x_est3*P13*accelNoiseVar+4*x_est3*P13*P00-4*P03*x_est3*P01+4*DT2*P11*P23-4*dt*P12*P03+4*P00*P23-2*P03*x_est3*DT3RATEQ-1*DT4RATEQ*P13*x_est3-4*P03*x_est3*dt*P11)/sigma[23];

  float F0 = P00+2*dt*P01+DT2*P11+1/4*DT4RATEQ-K00*P00-2*K00*dt*P01-K00*DT2*P11-1/4*K00*DT4RATEQ-K01*x_est3*P01-K01*x_est3*dt*P11-1/2*K01*x_est3*DT3RATEQ-K01*P02-K01*dt*P12-K01*x_est1*P03-K01*x_est1*dt*P13;
  float F1 = P01+dt*P11+1/2*DT3RATEQ-K00*P01-K00*dt*P11-1/2*K00*DT3RATEQ-K01*x_est3*P11-K01*x_est3*DT2RATEQ-K01*P12-K01*x_est1*P13;
  float F2 = P02+dt*P12-K00*P02-K00*dt*P12-K01*x_est3*P12-K01*P22-K01*DT2BIASQ-K01*x_est1*P23;
  float F3 = P03+dt*P13-K00*P03-K00*dt*P13-K01*x_est3*P13-K01*P23-K01*x_est1*P33-K01*x_est1*DT2SCALE;
  float F4 = -K10*P01-K10*dt*P11-1/2*K10*DT3RATEQ+P11+DT2RATEQ-K11*x_est3*P11-K11*x_est3*DT2RATEQ-K11*P12-K11*x_est1*P13;
  float F5 = -K10*P02-K10*dt*P12+P12-P12*K11*x_est3-K11*P22-K11*DT2BIASQ-K11*x_est1*P23;
  float F6 = -K10*P03-K10*dt*P13+P13-P13*K11*x_est3-K11*P23-K11*x_est1*P33-K11*x_est1*DT2SCALE;
  float F7 = -K20*P02-K20*dt*P12-K21*x_est3*P12+P22+DT2BIASQ-K21*P22-K21*DT2BIASQ-K21*x_est1*P23;
  float F8 = -K20*P03-K20*dt*P13-K21*x_est3*P13+P23-P23*K21-K21*x_est1*P33-K21*x_est1*DT2SCALE;
  float F9 = -K30*P03-K30*dt*P13-K31*x_est3*P13-K31*P23+P33+DT2SCALE-K31*x_est1*P33-K31*x_est1*DT2SCALE;
  float F10 = x_est0+dt*x_est1+K00*acc_angle-K00*x_est0-K00*dt*x_est1+K01*gyro_rate-K01*x_est3*x_est1-K01*x_est2;
  float F11 = x_est1+K10*acc_angle-K10*x_est0-K10*dt*x_est1+K11*gyro_rate-K11*x_est3*x_est1-K11*x_est2;
  float F12 = x_est2+K20*acc_angle-K20*x_est0-K20*dt*x_est1+K21*gyro_rate-K21*x_est3*x_est1-K21*x_est2;
  float F13 = x_est3+K30*acc_angle-K30*x_est0-K30*dt*x_est1+K31*gyro_rate-K31*x_est3*x_est1-K31*x_est2;

  P00 = F0;
  P01 = F1;
  P02 = F2;
  P03 = F3;
  P11 = F4;
  P12 = F5;
  P13 = F6;
  P22 = F7;
  P23 = F8;
  P33 = F9;
  x_est0 = F10;
  x_est1 = F11;
  x_est2 = F12;
  x_est3 = F13;
}
