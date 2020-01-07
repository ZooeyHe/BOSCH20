#include <SPI.h>
/*
   H1_Controller takes a joystick's input
   ==========================================
   Project: JHU BOSCH20 Senior Design
   Description: Control a H1-design hexapod using joysticks to
      to input desired positions
   Author: Zhuohong (Zooey) He
   Date: 11.21.2019
   ==========================================
   NOTE: For use on an Arduino Mega, LS7633 Shield, Datalogger Shield
   make sure to do "Sketch->Include Library-> Add .ZIP Library" and
   find the MatrixMath.zip file
*/

// Control Loop Characteristics
float kpMax = 10.0;
float kiMax = 10.0;
float kdMax = 10.0;

float kp = kpMax / 2.0; // [V/rad]
float ki = kiMax / 2.0; // [V/(rad*s)]
float kd = kdMax / 2.0; // [V/(rad/s)]
int stepSize = 100; // [counts]

int kpReadPin = A0;
int kiReadPin =  A1;
int kdReadPin =  A2;
int stepButton = 21;
int endButton =  20;

// Pins to Control BTS7960 Motor Driver
const byte RPWM_OUTPUT = 4; //
const byte LPWM_OUTPUT = 5; //
const byte REN_OUTPUT  = 22; // PortA
const byte LEN_OUTPUT  = 23; // PortB

// Encoder Specifications
const int countsPerRev = 600; // if 4x multiply this number by 4

// Variables that we update while running
long desiredCounts;
long currentCounts;
long lastFinish;
long lastCounts;
long error;
float intError;
long lastError;
long lastStart;

// Platform Pin Positions wrt Platform Coordinates {x, y, z, thz}
float platformSetup[6][4];

// Base Motor Positions wrt Base Coordinates {x, y, z, thz}
float baseSetup[6][4];

#define ENABLE    1
#define DISABLE   0

#define NQUAD     0x00 //non-quadrature mode
#define QUADRX1   0x01 //X1 quadrature mode
#define QUADRX2   0x02 //X2 quadrature mode
#define QUADRX4   0x03 //X4 quadrature mode
//Running modes
#define FREE_RUN  0x00
#define SINGE_CYCLE 0x04
#define RANGE_LIMIT 0x08
#define MODULO_N  0x0C
//Index modes
#define DISABLE_INDX  0x00  //index_disabled
#define INDX_LOADC    0x10  //index_load_CNTR
#define INDX_RESETC   0x20  //index_rest_CNTR
#define INDX_LOADO    0x30  //index_load_OL
#define ASYNCH_INDX   0x00  //asynchronous index
#define SYNCH_INDX    0x80  //synchronous index
//Clock filter modes
#define FILTER_1    0x00  //filter clock frequncy division factor 1
#define FILTER_2    0x80  //filter clock frequncy division factor 2
/* *MDR1 configuration data; any of these**
***data segments can be ORed together***/
//Flag modes
#define NO_FLAGS    0x00  //all flags disabled
#define IDX_FLAG    0x10  //IDX flag
#define CMP_FLAG    0x20  //CMP flag
#define BW_FLAG     0x40  //BW flag
#define CY_FLAG     0x80  //CY flag
//1 to 4 bytes data-width
#define BYTE_4      0x00  //four byte mode
#define BYTE_3      0x01  //three byte mode
#define BYTE_2      0x02  //two byte mode
#define BYTE_1      0x03  //one byte mode
//Enable/disable counter
#define EN_CNTR     0x00  //counting enabled
#define DIS_CNTR    0x04  //counting disabled
/* LS7366R op-code list */
#define CLR_MDR0      0x08
#define CLR_MDR1      0x10
#define CLR_CNTR      0x20
#define CLR_STR       0x30
#define READ_MDR0     0x48
#define READ_MDR1     0x50
#define READ_CNTR     0x60
#define READ_OTR      0x68
#define READ_STR      0x70
#define WRITE_MDR1    0x90
#define WRITE_MDR0    0x88
#define WRITE_DTR     0x98
#define LOAD_CNTR     0xE0
#define LOAD_OTR      0xE4

//the lines are used by 74HC138 chip to select the cable select lines
int nSS_ENC_A2_pin = 42;//C  A2
int nSS_ENC_A1_pin = 41; //B  A1
int nSS_ENC_A0_pin = 40; //A  A0

//CLK Select DFLAG DF-F
int CLK_SEL_DFAG_pin = 38;

//Enable ENC_SS
int EN_ENC_SS_pin = 39;

//Blue LED
int LED_ACT_pin = 45;

//DFLAG
int DFLAG_pin = 3;

//LFLAG
int LFLAG_pin = 2;

#define Slave_Select_Low PORTB &= ~(1 << PB4)
#define Slave_Select_High PORTB |= (1 << PB4)

//function prototypes
void blinkActLed(void);
long getChanEncoderValue(int encoder);
unsigned int getChanEncoderReg(int opcode, int encoder);
void rstEncCnt(int encoder);
void setSSEnc(bool enable, int encoder);
void clearStrReg(int encoder);
void setSSEncCtrlBits(int value);
void Init_LS7366Rs(void);

void setDFlagChMux(int encoder);

void loadRstReg(unsigned char op_code);
void writeSingleByte(unsigned char op_code, unsigned char data);
unsigned char readSingleByte(unsigned char op_code);

//Global Variables
int IsrDFlag;
int DFlagCh;
int IsrLFlag;
int LFlagCnt[6];


void setup() {
  // Setting Up Pins
  Serial.begin(9600);
  Serial.println("... Setting Up ...");

  pinMode(stepButton, INPUT_PULLUP);
  pinMode(endButton, INPUT_PULLUP);
  pinMode(kpReadPin, INPUT);
  pinMode(kiReadPin, INPUT);
  pinMode(kdReadPin, INPUT);
  
  pinMode(RPWM_OUTPUT, OUTPUT);
  pinMode(LPWM_OUTPUT, OUTPUT);
  pinMode(REN_OUTPUT,  OUTPUT);
  pinMode(LEN_OUTPUT,  OUTPUT);
  enableMotor();
  Serial.println("... Finished Setting Up Motor Control ...");
  pinMode(LED_ACT_pin, OUTPUT);
  pinMode(CLK_SEL_DFAG_pin, OUTPUT);
  pinMode(EN_ENC_SS_pin, OUTPUT);
  pinMode(nSS_ENC_A2_pin, OUTPUT);
  pinMode(nSS_ENC_A1_pin, OUTPUT);
  pinMode(nSS_ENC_A0_pin, OUTPUT);
  digitalWrite(LED_ACT_pin, LOW);
  digitalWrite(CLK_SEL_DFAG_pin, LOW);
  digitalWrite(EN_ENC_SS_pin, LOW);
  digitalWrite(nSS_ENC_A2_pin, HIGH);
  digitalWrite(nSS_ENC_A1_pin, HIGH);
  digitalWrite(nSS_ENC_A0_pin, HIGH);
  pinMode(DFLAG_pin, INPUT);
  pinMode(LFLAG_pin, INPUT_PULLUP);
  Init_LS7366Rs();
  Serial.println("... Finished Setting Up Encoder Shield ...");
  Serial.println("... Finished Performing Initial Calculations ...");
  lastStart = micros();
  long temp;
  temp = getCount(1);
  desiredCounts = 0;
  error = desiredCounts - temp;
  Serial.println("... Finished Setup, Beginning Loop Now ...");
}


void loop() {

  Serial.print(desiredCounts);
  Serial.print(" ");

  long start = micros();
  long dt = start - lastStart;
  lastStart = start;

  currentCounts = getCount(1);
  Serial.print(currentCounts);
  Serial.println();
  error = desiredCounts - currentCounts;
  long derror = error - lastError;

  float de_dt = derror * 1.0 / dt;

  intError = intError + error * dt;

  kp = (analogRead(kpReadPin) - 512) / 512.0 * kpMax;
  ki = (analogRead(kiReadPin) - 512) / 512.0 * kiMax;
  kd = (analogRead(kdReadPin) - 512) / 512.0 * kdMax;
  
  long ctrlSig = kp * counts2rad(error, countsPerRev) + kd * counts2rad(de_dt, countsPerRev) + ki * counts2rad(intError, countsPerRev);
  unsigned int PWMvalue = (int) constrain(abs(ctrlSig / 5 * 255), 0, 200); // Constrained for safety, usually goes to 255

  if (ctrlSig < 0) {
    analogWrite(LPWM_OUTPUT, 0);
    analogWrite(RPWM_OUTPUT, PWMvalue);
  } else if (ctrlSig > 0) {
    analogWrite(LPWM_OUTPUT, PWMvalue);
    analogWrite(RPWM_OUTPUT, 0);
  }

  if (digitalRead(stepButton) == LOW) {
    if (desiredCounts == stepSize) {
      desiredCounts = 0;
    } else {
      desiredCounts == stepSize;
    }
  } else if (digitalRead(endButton) == LOW) {
    while(true){}
  }
}

long getCount(int encoder) {
  // TODO: Implement this method to read i2c from LS7366 Shield
  long result = getChanEncoderValue(encoder + 1);
  return result;
}


// HELPER FUNCTIONS

void Init_LS7366Rs(void)
//*************************************************
{
  int a = 1;

  // SPI initialization
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);      // SPI at 125Khz (on 16Mhz clock)
  setSSEnc(DISABLE, 0);
  delay(100);

  //Serial.print("\r\n");
  // Serial.print("\r\n");

  //initialize the 6
  for (a = 1; a <= 6; a++)
  {
    //********
    setSSEnc(ENABLE, a);
    SPI.transfer(WRITE_MDR0);// Select MDR0 | WR register
    SPI.transfer(FILTER_2 | DISABLE_INDX | FREE_RUN | QUADRX1); // Filter clock division factor = 1 || Asynchronous Index ||
    // disable index || free-running count mode || x4 quadrature count mode
    setSSEnc(DISABLE, 0);

    //Serial.print(" TX MDR0=");
    //Serial.print(FILTER_2 | DISABLE_INDX | FREE_RUN | QUADRX1, HEX);
    //Serial.print(";");
    // Serial.print(" RX MDR0=");
    //Serial.print(getChanEncoderReg(READ_MDR0, a), HEX);
    //Serial.print(";");
    //********
    //********
    setSSEnc(ENABLE, a);
    SPI.transfer(WRITE_MDR1);// Select MDR1 | WR register
    SPI.transfer(CMP_FLAG | BYTE_4 | EN_CNTR); //4-byte counter mode || Enable counting || FLAG on CMP (B5 of STR)
    setSSEnc(DISABLE, 0);

    //Serial.print(" TX MDR1=");
    //Serial.print(CMP_FLAG | BYTE_4 | EN_CNTR, HEX);
    //Serial.print(";");
    //Serial.print(" RX MDR1=");
    //Serial.print(getChanEncoderReg(READ_MDR1, a), HEX);
    //Serial.print(";");
    //********
    //********
    setSSEnc(ENABLE, a);
    SPI.transfer(WRITE_DTR);// Select DTR | WR register
    SPI.transfer(0x00);// DTR MSB
    SPI.transfer(0x00);// DTR
    SPI.transfer(0x00);// DTR
    SPI.transfer(0x0A);// DTR LS
    setSSEnc(DISABLE, 0);
    //********
    //********
    setSSEnc(ENABLE, a);
    SPI.transfer(LOAD_CNTR);
    setSSEnc(DISABLE, 0);

    //Serial.print(" Ch");
    //Serial.print(a);
    //Serial.print("=");
    //Serial.print(getChanEncoderValue(a), HEX);
    //Serial.print(";");
    //********
    //********
    setSSEnc(ENABLE, a);
    SPI.transfer(CLR_CNTR);// Select CNTR || CLEAR register
    setSSEnc(DISABLE, 0);
    //********
    //********
    clearStrReg(a);   //reseting the counter value inside the encoder chips to 0
    rstEncCnt(a);
    //********
    //********
    //Serial.print(" STR=");
    //Serial.print(getChanEncoderReg(READ_STR, a), BIN);
    //Serial.print(";");
    //Serial.print("\t");
    //********
    //********
    //Serial.print("\r\n");
  }
} //end func

void clearStrReg(int encoder)
//*************************************************
{
  setSSEnc(ENABLE, encoder);
  SPI.transfer(CLR_STR);// Select STR || CLEAR register
  setSSEnc(DISABLE, 0);
} //end func

void rstEncCnt(int encoder)
//*****************************************************
{
  setSSEnc(DISABLE, encoder);
  SPI.transfer(CLR_CNTR);
  setSSEnc(DISABLE, 0);
} //end func

/**
   Channel Encoder Slave Select Control
*/
void setSSEnc(bool enable, int encoder) {
  if (encoder > 0)
    setSSEncCtrlBits(encoder - 1);

  if (enable)
    digitalWrite(EN_ENC_SS_pin, HIGH);
  else
    digitalWrite(EN_ENC_SS_pin, LOW);

}

void setSSEncCtrlBits(int value) {
  switch (value)
  {
    case 0:
      digitalWrite(nSS_ENC_A2_pin, LOW);
      digitalWrite(nSS_ENC_A1_pin, LOW);
      digitalWrite(nSS_ENC_A0_pin, LOW);
      break;

    case 1:
      digitalWrite(nSS_ENC_A2_pin, LOW);
      digitalWrite(nSS_ENC_A1_pin, LOW);
      digitalWrite(nSS_ENC_A0_pin, HIGH);
      break;

    case 2:
      digitalWrite(nSS_ENC_A2_pin, LOW);
      digitalWrite(nSS_ENC_A1_pin, HIGH);
      digitalWrite(nSS_ENC_A0_pin, LOW);
      break;

    case 3:
      digitalWrite(nSS_ENC_A2_pin, LOW);
      digitalWrite(nSS_ENC_A1_pin, HIGH);
      digitalWrite(nSS_ENC_A0_pin, HIGH);
      break;

    case 4:
      digitalWrite(nSS_ENC_A2_pin, HIGH);
      digitalWrite(nSS_ENC_A1_pin, LOW);
      digitalWrite(nSS_ENC_A0_pin, LOW);
      break;

    case 5:
      digitalWrite(nSS_ENC_A2_pin, HIGH);
      digitalWrite(nSS_ENC_A1_pin, LOW);
      digitalWrite(nSS_ENC_A0_pin, HIGH);
      break;

    case 6:
      digitalWrite(nSS_ENC_A2_pin, HIGH);
      digitalWrite(nSS_ENC_A1_pin, HIGH);
      digitalWrite(nSS_ENC_A0_pin, LOW);
      break;

    case 7:
      digitalWrite(nSS_ENC_A2_pin, HIGH);
      digitalWrite(nSS_ENC_A1_pin, HIGH);
      digitalWrite(nSS_ENC_A0_pin, HIGH);
      break;

    default:
      digitalWrite(nSS_ENC_A2_pin, HIGH);
      digitalWrite(nSS_ENC_A1_pin, HIGH);
      digitalWrite(nSS_ENC_A0_pin, HIGH);
      break;
  }

}

long getChanEncoderValue(int encoder)
//*****************************************************
{
  unsigned int cnt1Value, cnt2Value, cnt3Value, cnt4Value;
  long result;

  setSSEnc(ENABLE, encoder);

  SPI.transfer(READ_CNTR); // Request count
  cnt1Value = SPI.transfer(0x00); // Read highest order byte
  cnt2Value = SPI.transfer(0x00);
  cnt3Value = SPI.transfer(0x00);
  cnt4Value = SPI.transfer(0x00); // Read lowest order byte

  setSSEnc(DISABLE, 0);

  result = ((long) cnt1Value << 24) + ((long) cnt2Value << 16) + ((long) cnt3Value << 8) + (long) cnt4Value;
  //Serial.println(result);
  return result;
} //end func

unsigned int getChanEncoderReg(int opcode, int encoder)
{
  unsigned int Value;

  setSSEnc(ENABLE, encoder);
  SPI.transfer(opcode);
  Value = SPI.transfer(0x00); // Read byte
  setSSEnc(DISABLE, 0);
  return Value;
}

void calculateRotationMatrix(float rotMat[3][3], float * desiredPos) {
  float thx = desiredPos[3];
  float thy = desiredPos[4];
  float thz = desiredPos[5];

  rotMat[0][0] =  cos(thz) * cos(thy);
  rotMat[0][1] = -sin(thz) * cos(thx) + cos(thz) * sin(thy) * sin(thx);
  rotMat[0][2] =  sin(thz) * sin(thx) + cos(thz) * sin(thy) * cos(thx);
  rotMat[1][0] =  sin(thz) * cos(thy);
  rotMat[1][1] =  cos(thz) * cos(thx) + sin(thz) * sin(thy) * sin(thx);
  rotMat[1][2] = -cos(thz) * sin(thx) + sin(thz) * sin(thy) * cos(thx);
  rotMat[2][0] = -sin(thy);
  rotMat[2][1] =  cos(thy) * sin(thx);
  rotMat[2][2] =  cos(thy) * cos(thx);
}

void matmulvec(float * result, float mat[3][3], float * vec) {
  for (int r = 0; r < 3; r++) {
    result[r] = 0.0;
    for (int c = 0; c < 3; c++) {
      result[r] = result[r] + mat[r][c] * vec[c];
    }
  }
}

void vecAdd(float * result, float * vec1, float * vec2, int len) {
  for (int i = 0; i < len; i++) {
    result[i] = vec1[i] + vec2[i];
  }
}

void vecSub(float * result, float * vec1, float * vec2, int len) {
  for (int i = 0; i < len; i++) {
    result[i] = vec1[i] - vec2[i];
  }
}

void vecSub(long * result, long * vec1, long * vec2, int len) {
  for (int i = 0; i < len; i++) {
    result[i] = vec1[i] - vec2[i];
  }
}

float mag(float * vector) {
  return sqrt(sq(vector[0]) + sq(vector[1]) + sq(vector[2]));
}

float deg2rad(float deg) {
  return deg * PI / 180.0;
}

float cm2m(float cm) {
  return cm / 100.0;
}

float counts2rad(float count, float cntsPerRev) {
  return count / cntsPerRev * 2 * PI;
}

void printVector(int * vec, int len) {
  for (int i = 0; i < len; i++) {
    Serial.print(vec[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void printVector(float * vec, int len) {
  for (int i = 0; i < len; i++) {
    Serial.print(vec[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void printVector(long * vec, int len) {
  for (int i = 0; i < len; i++) {
    Serial.print(vec[i]);
    Serial.print("\t");
  }
  Serial.println();
}

// MOTOR CONTROL FUNCTIONS
void enableMotor() {
  digitalWrite(REN_OUTPUT, HIGH);
  digitalWrite(LEN_OUTPUT, HIGH);
}

void disableMotor(int i) {
  digitalWrite(REN_OUTPUT, LOW);
  digitalWrite(LEN_OUTPUT, LOW);
}
