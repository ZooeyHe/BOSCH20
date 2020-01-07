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

// Hexapod Dimensions [cm, degrees]
const float Rp =   15;
float       minp = 50;
const float Rb =   18;
float       minb = 50;
const float a =     6;
const float s =    18;

// Hexapod Motion Envelope Limitations {x, y, z, thx, thy, thz} [cm, degrees]
float platformLimits[] = {20, 20, 20, 5, 5, 5};
const float zorigin = 22;

// Hexapod Limitations [degrees]
float motorAngleLimits[] =  { -30, 90};
int evenEncLowLim = -50;
int oddEncLowLim = -120;


// Control Loop Characteristics
const float kp = 2.0; // [V/rad]
const float kd = 0.1; // [V/(rad/s)]
const float runsPerRead = 5; // number of control iterations per joystick reading

// Pins to Control BTS7960 Motor Driver
const byte RPWM_OUTPUT[] = {     2,     4,     6,     8,    10,    12}; // Change first num to 2
const byte LPWM_OUTPUT[] = {     3,     5,     7,     9,    11,    13}; // Change first num to 3
const byte REN_OUTPUT[]  = {    22,    23,    24,    25,    26,    27}; // PortA
const byte LEN_OUTPUT[]  = {    30,    31,    32,    33,    34,    35}; // PortB
const byte MOTOR_ON[]    = {  true,  true, false, false, false, false}; // Switch motors on to run

// Pins to Read Potentiometers on Joystick
const byte JS_INPUT[] = {A0, A1, A2, A3, A4, A5};

// Encoder Specifications
const int countsPerRev = 600; // 4x counting!

// Variables what we update while running
float desiredCounts[6];
float currentCounts[6];
long lastFinish;
float lastCounts[6];

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
/* **MDR1 configuration data; any of these***
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
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  //Serial.println("Setting Up");
  for (int i = 0; i < 6; i++) {
    pinMode(RPWM_OUTPUT[i], OUTPUT);
    pinMode(LPWM_OUTPUT[i], OUTPUT);
    pinMode(REN_OUTPUT[i],  OUTPUT);
    pinMode(LEN_OUTPUT[i],  OUTPUT);
    enableMotor(i);
  }

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

  // Converting input units to computationally efficient units
  platformLimits[3] = deg2rad(platformLimits[3]);
  platformLimits[4] = deg2rad(platformLimits[4]);
  platformLimits[5] = deg2rad(platformLimits[5]);
  motorAngleLimits[0] = deg2rad(motorAngleLimits[0]);
  motorAngleLimits[1] = deg2rad(motorAngleLimits[1]);
  minp = deg2rad(minp);
  minb = deg2rad(minb);

  // Setting up variables
  getPinsWrtPlatform();
  getMotorsWrtBase();
  evenEncLowLim = (int)(-evenEncLowLim / 360.0 * countsPerRev);
  oddEncLowLim = (int)(-oddEncLowLim / 360.0 * countsPerRev);

  //Serial.println("Finished Set Up, beginning loop now...");

  Init_LS7366Rs();

  lastFinish = micros();
  getCurrentCounts(lastCounts);
}




void loop() {
  getDesiredEncoderCounts(desiredCounts);
  // printVector(desiredCounts, 6);
  controlled_move(desiredCounts, lastFinish, lastCounts, currentCounts, MOTOR_ON, runsPerRead);
}

boolean controlled_move(float desired[6], long lastEnd[6], float lastCnts[6], float current[6], byte motorOn[6], unsigned int iterations) {
  float error[6]; // The error in counts
  long start;
  int elapsedTime;
  float elapsedCounts;
  float ctrlSig = 0;

  for (int n = 0; n < iterations; n++) {
    //printVector(current, 6);
    //printVector(error, 6);
    //Serial.println();
    for (int m = 0; m < 6; m++) {
      if (motorOn[m] == false) {
        continue;
      }

      start = micros();
      elapsedTime = start - lastEnd[m];
      lastEnd[m] = start;

      //Serial.println(getChanEncoderValue(m+1));
      current[m] = getCount(m);
      elapsedCounts = current[m] - lastCnts[m];
      lastCnts[m] = current[m];

      error[m] = desired[m] - current[m];
      

      ctrlSig = kp * 2 * PI * error[m] / countsPerRev + kd * 2 * PI * elapsedCounts / countsPerRev / elapsedTime * 1000000;
      unsigned int PWMvalue = (int)constrain(abs(ctrlSig / 5 * 255), 0, 200); // Heavily constrained for safety, usually goes to 255

      
      //Serial.print(ctrlSig);
      //Serial.print(", ");
      
      if (ctrlSig < 0) {
        analogWrite(LPWM_OUTPUT[m], 0);
        analogWrite(RPWM_OUTPUT[m], PWMvalue);
      } else if (ctrlSig > 0) {
        analogWrite(LPWM_OUTPUT[m], PWMvalue);
        analogWrite(RPWM_OUTPUT[m], 0);
      }
    }
    //Serial.println();
    printVector(error,6);
  }
  return true;
}

void getCurrentCounts(float counts[6]) {
  for (int m = 0; m < 6; m++) {
    counts[m] = getCount(m);
    if (m == 1) {
    }
  }
}

long getCount(int encoder) {
  // TODO: Implement this method to read i2c from LS7366 Shield
  long result = getChanEncoderValue(encoder + 1);
  return result;
}

float getCountTESTING(float count, int motor) {
  // return count + (desiredCounts[motor] - count) / (10.0 - kp);
  return 0;
}

/*
   Calculate the desired encoder counts from desired platform position
*/
void getDesiredEncoderCounts(float * counts) {
  float desiredPlatformPosition[6];
  float rot[3][3];
  float p[3];             //NOTE: p is the platform pin in the base frame
  getDesiredPlatformPosition(desiredPlatformPosition);
  calculateRotationMatrix(rot, desiredPlatformPosition);

  for (int m = 0; m < 6; m++) {
    calculatePlatformPinWrtBase(p, desiredPlatformPosition, rot, platformSetup[m]);

    float linkageVec[3];
    float effPistonLen = mag(linkageVec);
    float L = sq(effPistonLen) - (sq(s) - sq(a));
    float M = 2 * a * (p[2] - baseSetup[m][2]);
    float N = 2 * a * ((p[0] - baseSetup[m][0]) * cos(baseSetup[m][3]) + (p[1] - baseSetup[m][1]) * sin(baseSetup[m][3]));

    float alpha = asin(L / sqrt(sq(M) + sq(N)) - atan(N / M));


    if (m % 2 == 0) {  // If we have an odd numbered motor, invert angle. TODO: Test this, may need to swap 0 for 1
      counts[m] = -alpha / 2 / PI * countsPerRev; // + evenEncLowLim;
    } else {
      counts[m] = alpha / 2 / PI * countsPerRev; // + oddEncLowLim;
    }
  }

}


/*
   calculate the desired position from joystick input
*/
void getDesiredPlatformPosition(float * pos) {
  int joystickAnalogReadings[6];
  for (int d = 0; d < 6; d++) {
    float JSreading = readJoystick(JS_INPUT[d]);
    pos[d] = platformLimits[d] * JSreading;

    if (d == 2) {
      pos[d] += zorigin;
    }
  }
  // Serial.println();
}


/*
   Read's a joystick's value and returns a value from -1.0 to 1.0
*/
float readJoystick(int pin) {
  float JSvalue = analogRead(pin);
  // Serial.print(JSvalue);
  // Serial.print(" ");
  JSvalue = (JSvalue - 512) / 512.0;

  return JSvalue;
}


void calculatePlatformPinWrtBase(float * pins, float * platformPos, float rotMat[3][3], float * pinWrtPlatform) {
  float rotatedVec[3];
  matmulvec(rotatedVec, rotMat, pinWrtPlatform);
  vecAdd(pins, platformPos, rotatedVec, 3);
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


float mag(float * vector) {
  return sqrt(sq(vector[0]) + sq(vector[1]) + sq(vector[2]));
}

float deg2rad(float deg) {
  return deg * PI / 180.0;
}

float cm2m(float cm) {
  return cm / 100.0;
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

// MOTOR CONTROL FUNCTIONS
void enableMotor(int i) {
  digitalWrite(REN_OUTPUT[i], HIGH);
  digitalWrite(LEN_OUTPUT[i], HIGH);
}

void disableMotor(int i) {
  digitalWrite(REN_OUTPUT[i], LOW);
  digitalWrite(LEN_OUTPUT[i], LOW);
}


// FUNCTIONS USED TO SET UP THE SYSTEM
void getPinsWrtPlatform() {
  float majp = 2.0 * PI / 3.0 - minp;
  float theta = majp / 2.0;
  for (int i = 0; i < 6; i++) {
    platformSetup[i][0] = Rp * cos(theta);
    platformSetup[i][1] = Rp * sin(theta);
    platformSetup[i][2] = 0;
    if (i % 2 == 0) {
      theta += minp;
    } else {
      theta += majp;
    }
  }
  platformSetup[0][3] = deg2rad( -30);
  platformSetup[1][3] = deg2rad( 150);
  platformSetup[2][3] = deg2rad(  90);
  platformSetup[3][3] = deg2rad( -90);
  platformSetup[4][3] = deg2rad(-150);
  platformSetup[5][3] = deg2rad(  30);
}

void getMotorsWrtBase() {
  float majb = 2.0 * PI / 3.0 - minb;
  float theta = majb / 2.0;
  for (int i = 0; i < 6; i++) {
    baseSetup[i][0] = Rb * cos(theta);
    baseSetup[i][1] = Rb * sin(theta);
    baseSetup[i][2] = 0;
    if (i % 2 == 0) {
      theta += minb;
    } else {
      theta += majb;
    }
  }
  baseSetup[0][3] = deg2rad( -30);
  baseSetup[1][3] = deg2rad( 150);
  baseSetup[2][3] = deg2rad(  90);
  baseSetup[3][3] = deg2rad( -90);
  baseSetup[4][3] = deg2rad(-150);
  baseSetup[5][3] = deg2rad(  30);
}
