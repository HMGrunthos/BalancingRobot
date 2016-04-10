#define DATAOUT 11  // DIN
#define DATAIN 12   // DOUT
#define SPICLOCK 13 // SCLK
#define CHIPSELECT0 10 // CS_n 0

// #define CHIPSELECT1 8 // CS_n 1
// #define CHIPSELECT2 7 // CS_n 2
// #define CHIPSELECT3 6 // CS_n 3

#define DT  0.005

float kGain[3][2] = {{0.0017615, 0.0027167}, {0.00031415, 0.9112}, {-0.00027004, 1.2451e-6}};

float x_est0 = 0;
float x_est1 = 0;
float x_est2 = 0;

volatile float acc_ang_meas;
volatile float x_dot_meas;

unsigned long int mStartTime = 0;
unsigned long int nUpdates = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(CHIPSELECT0, OUTPUT);
//  pinMode(CHIPSELECT1, OUTPUT);
//  pinMode(CHIPSELECT2, OUTPUT);
//  pinMode(CHIPSELECT3, OUTPUT);
  digitalWrite(CHIPSELECT0, LOW); // disable device
//  digitalWrite(CHIPSELECT1, HIGH); // disable device
//  digitalWrite(CHIPSELECT2, HIGH); // disable device
//  digitalWrite(CHIPSELECT3, HIGH); // disable device

  delay(500);
  Serial.println("Init.");
}

void loop()
{
  if(!mStartTime) {
    mStartTime = millis();
  } else if((millis() - mStartTime) > (unsigned long int)60000) {
    Serial.print("Elapsed updates :");
    Serial.print(nUpdates);
    Serial.println(":");
    mStartTime = millis();
    nUpdates = 0;
  }
  nUpdates++;

  float a = acc_ang_meas - x_est0 - DT*x_est1;
  float b = x_dot_meas - x_est1 - x_est2;

  x_est0 = x_est0 + DT*x_est1 + kGain[0][0]*a + kGain[0][1]*b;
  x_est1 = x_est1 +             kGain[1][0]*a + kGain[1][1]*b;
  x_est2 = x_est2 +             kGain[2][0]*a + kGain[2][1]*b;
}
