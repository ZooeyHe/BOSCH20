#include <SPI.h>
/*
   H1_StepMove takes a platform position command from the serial window.
   ==========================================
   Project: JHU BOSCH20 Senior Design
   Description: Control a H1-design hexapod using the serial
      to input desired positions
   Author: Zhuohong (Zooey) He
   Date: 02.20.2020
   ==========================================
   NOTE: For use on an Arduino Mega, LS7633 Shield, Datalogger Shield, and Pololu G2 24v21 Drivers.
   Use {a,s,d,f,g,h} to increment in the 6DOF and {z,x,c,v,b,n} to decrement
*/

// For Debugging
int verbosity = 0; // Do not print trace
//int verbosity = 1; // Prints some:
//int verbosity = 2; // Prints some:
//int verbosity = 3; // Print all trace for debugging

// Hexapod Dimensions [cm, degrees]
const float Rp =   14.9;
float       minp = 50;
const float Rb =   17.6;
float       minb = 50;
const float a =    6.5;
const float s =    20.3;

// Hexapod Motion Envelope Limitations {x, y, z, thx, thy, thz} [cm, degrees]
float platformLimits[] = {10, 10, 5, 10, 10, 5};
float desiredPlatformPosition[] = {0, 0, 25, 0, 0 , 0};
const float zorigin = 25;

// Hexapod Limitations [degrees]
float motorAngleLimits[] =  { -30, 90};

// Control Loop Characteristics
const float kp = 9.5; // [V/rad]
const float ki = 10.0; // [V/(rad*s)]
const float kd = 0.08; // [V/(rad/s)] // change back to 0.08 if needed
const float runsPerRead = 8; // number of control iterations per joystick reading
const int   speedLimit = 255; // set limit between 0 and 255
const byte  startpin = 49;

// Pins to Control BTS7960 Motor Driver
// MOTOR NUMBER:            1    2    3    4    5    6
//                       ==============================
const byte MOTOR_ON[] = {   1,   1,   1,   1,   1,   1}; // Switch motors on to run
const byte PWM[]      = {   6,   7,   8,   9,  10,  11}; // Change first num to 2
const byte DIR[]      = {  22,  23,  24,  25,  26,  27}; // Direction PIN
const byte CS[]       = {  A0,  A1,  A2,  A3,  A4,  A5}; // Current Sense Pin
const byte SWITCHES[] = {  30,  31,  32,  33,  34,  35}; // Microswitches used for startup calibrations

// Pins to Read Potentiometers on Joystick
const byte JS_INPUT[] =   { A8,  A9, A10, A11, A12, A13};
// This is the one direction analogRead range for each of the joysticks, 512 if full
const byte JS_MAPPING[] = {120, 120, 180, 120, 120, 180};

// Step Sizes for
float STEP_SIZE[] =  {  1,  1,  1,  1,  1,  1}; // cm or degree depending on direction
const char POS_STEP_CMDS[] =  {'a', 's', 'd', 'f', 'g', 'h'};
const char NEG_STEP_CMDS[] =  {'z', 'x', 'c', 'v', 'b', 'n'};


// Encoder Specifications
const int quadCountMode = 4; // 1x, 2x, or 4x
const int countsPerRev = 600 * quadCountMode;
const float microswitchAngle = 16.8; //degrees

// Variables that we update while running
long desiredCounts[6], currentCounts[6], lastCounts[6], error[6], lastError[6], lastStart[6];
float intError[6];

float platformSetup[6][4];  // Platform Pin Positions wrt Platform Coordinates {x, y, z, thz}
float baseSetup[6][4];      // Base Motor Positions wrt Base Coordinates {x, y, z, thz}

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
int nSS_ENC_A2_pin = 42;    //C  A2
int nSS_ENC_A1_pin = 41;    //B  A1
int nSS_ENC_A0_pin = 40;    //A  A0
int CLK_SEL_DFAG_pin = 38;  //CLK Select DFLAG DF-F
int EN_ENC_SS_pin = 39;     //Enable ENC_SS
int LED_ACT_pin = 45;       //Blue LED
int DFLAG_pin = 3;          //DFLAG
int LFLAG_pin = 2;          //LFLAG

#define Slave_Select_Low PORTB &= ~(1 << PB4)
#define Slave_Select_High PORTB |= (1 << PB4)

//Global Variables
int IsrDFlag, DFlagCh, IsrLFlag;
int LFlagCnt[6];

//=============================SETUP==================================
void setup() {
  // Setting Up Pins
  Serial.begin(38400);
  if (verbosity > 0) {
    Serial.println("... Setting Up ...");
  }
  initJoystickPins();
  modifyPWMfrequency(1);

  if (verbosity > 0) {
    Serial.println("... Finished Setting Up Joystick Input ...");
  }
  if (verbosity > 1) {
    updateDesiredEncoderCounts(desiredCounts, true);
    Serial.println("... Current Desired Position (Counts) is ...");
    printVector(desiredCounts, 6);
  }

  initMotorPins();
  if (verbosity > 1) {
    Serial.println("... Finished Setting Up Motor Pins ...");
  }

  initCounterPins();
  Serial.println("Press the start button to continue");
  pinMode(startpin, INPUT_PULLUP);
  while (digitalRead(startpin) == HIGH) {
    delay(10);
  }
  Serial.println("... Starting ...");
  delay(500);

  Init_LS7366Rs();
  if (verbosity > 1) {
    Serial.println("... Finished Setting Up Encoder Shield ...");
  }

  // Converting input units to computationally efficient units
  platformLimits[3] = deg2rad(platformLimits[3]);
  platformLimits[4] = deg2rad(platformLimits[4]);
  platformLimits[5] = deg2rad(platformLimits[5]);
  STEP_SIZE[3] = deg2rad(STEP_SIZE[3]);
  STEP_SIZE[4] = deg2rad(STEP_SIZE[4]);
  STEP_SIZE[5] = deg2rad(STEP_SIZE[5]);
  motorAngleLimits[0] = deg2rad(motorAngleLimits[0]);
  motorAngleLimits[1] = deg2rad(motorAngleLimits[1]);
  minp = deg2rad(minp);
  minb = deg2rad(minb);

  // Setting up variables
  getPinsWrtPlatform();
  getMotorsWrtBase();

  if (verbosity > 1) {
    Serial.println("... Finished Performing Initial Calculations ...");
  }

  Serial.println("Press the start button to start calibration");
  pinMode(startpin, INPUT_PULLUP);
  while (digitalRead(startpin) == HIGH) {
    delay(10);
  }
  delay(500);

  Serial.println("... Peforming Calibration ...");

  calibration_procedure();

  Serial.println("... Calibration finished, handing control to user ...");

  for (int m = 0; m < 6; m++) {
    lastStart[m] = micros();
    error[m] = 0;
  }

  if (verbosity > 0) {
    Serial.println("... Finished Setup, Beginning Loop Now ...");
  }
  updateDesiredEncoderCounts(desiredCounts, true);
}

//=============================LOOP==================================
void loop() {
  int res = updateDesiredEncoderCounts(desiredCounts, false);
  if (res == 0) {
    printVector(desiredCounts, 6);
  } else if (res == 2) {
    Serial.println("... IMPOSSIBLE PLATFORM POSTITION ...");
  }
  if (verbosity > 2) {
    //printVector(currentCounts, 6);
    //printVector(desiredCounts, 6);
  }

  //float curr[6];
  //currentSenseAll(curr);
  //printVector(curr, 6);
  //Serial.print(curr[0] + curr[1] + curr[2]);
  //Serial.print(",");
  //Serial.println(curr[3] + curr[4] + curr[5]);

  // Runs the Loop Iterations
  for (int n = 0; n < runsPerRead; n++) {
    // Handle all 6 motors
    for (int m = 0; m < 6; m++) {
      if (MOTOR_ON[m] == false) {
        continue;
      }
      if (digitalRead(startpin) == LOW) {
        SystemShutDown();
      }

      long start = micros();
      float dt = start - lastStart[m];
      dt = dt / 1000000.0;
      lastStart[m] = start;

      currentCounts[m] = getCount(m);
      if (abs(currentCounts[m]) > 90 / 360.0 * countsPerRev) {
        SystemShutDown();
      }

      error[m] = desiredCounts[m] - currentCounts[m];
      long derror = error[m] - lastError[m];
      lastError[m] = error[m];
      float de_dt = derror * 1.0 / dt;
      intError[m] = intError[m] + error[m] * dt;

      float ctrlSig = kp * counts2rad(error[m], countsPerRev) + kd * counts2rad(de_dt, countsPerRev) + ki * counts2rad(intError[m], countsPerRev);
      unsigned int PWMvalue = (int) constrain(abs(ctrlSig / 5 * 255), 0, speedLimit); // Constrained for safety, usually goes to 255
      if (ctrlSig < 0) {
        digitalWrite(DIR[m], HIGH);
      } else if (ctrlSig > 0) {
        digitalWrite(DIR[m], LOW);
      }
      if (isnan(ctrlSig)) {
        analogWrite(PWM[m], 0);
      } else {
        analogWrite(PWM[m], PWMvalue);
      }
    }
  }
}

void getCurrentCounts(long counts[6]) {
  for (int m = 0; m < 6; m++) {
    counts[m] = getCount(m);
    if (m == 1) {
    }
  }
}

long getCount(int encoder) {
  // TODO: Implement this method to read i2c from LS7366 Shield
  long result = getChanEncoderValue(encoder + 1);
  if (encoder % 2 == 1) {
    result = result; // - 90 / 360.0 * countsPerRev;
  } else {
    result = result; // + 90 / 360.0 * countsPerRev;
  }
  return result;
}

/*
   Calculate the desired encoder counts from desired platform position
*/
int updateDesiredEncoderCounts(long * counts, bool forceUpdate) {
  if (Serial.available() || forceUpdate) {
    char command = Serial.read();
    boolean validChar = false;
    for (int i = 0; i < 6; i++) {
      if (command == POS_STEP_CMDS[i]) {
        desiredPlatformPosition[i] += STEP_SIZE[i];
        validChar = true;
        break;
      } else if (command == NEG_STEP_CMDS[i]) {
        desiredPlatformPosition[i] -= STEP_SIZE[i];
        validChar = true;
        break;
      } else if (forceUpdate) {
        validChar = true;
        break;
      }
    }

    if (!validChar) {
      return 1;
    }

    float rot[3][3];
    float p[3];
    calculateRotationMatrix(rot, desiredPlatformPosition);
    for (int m = 0; m < 6; m++) {
      calculatePlatformPinWrtBase(p, desiredPlatformPosition, rot, platformSetup[m]);
      float linkageVec[3];
      float effPistonLen = mag(linkageVec);
      float L = sq(effPistonLen) - (sq(s) - sq(a));
      float M = 2 * a * (p[2] - baseSetup[m][2]);
      float N = 2 * a * ((p[0] - baseSetup[m][0]) * cos(baseSetup[m][3]) + (p[1] - baseSetup[m][1]) * sin(baseSetup[m][3]));
      float alpha = asin(L / sqrt(sq(M) + sq(N)) - atan(N / M));
      if (isnan(alpha)) {
        return 2;
      } else {
        if (m % 2 == 0) // if odd
          counts[m] = alpha / 2 / PI * countsPerRev;
        else
          counts[m] = -alpha / 2 / PI * countsPerRev;
      }
    }
    return 0;
  }
  return 1;
}

/*
   calculate the desired position from joystick input
*/
void getDesiredPlatformPosition(float * pos) {
  int joystickAnalogReadings[6];
  for (int d = 0; d < 6; d++) {
    float JSreading = readJoystick(JS_INPUT[d], JS_MAPPING[d]);
    pos[d] = platformLimits[d] * JSreading;
    if (d == 2) {
      pos[d] += zorigin;
    }
  }
}


/*
   Read's a joystick's value and returns a value from -1.0 to 1.0
*/
float readJoystick(int pin, float mapValue) {
  float JSvalue = analogRead(pin);
  JSvalue = (JSvalue - 512) / mapValue;
  return JSvalue;
}


void calculatePlatformPinWrtBase(float * pins, float * platformPos, float rotMat[3][3], float * pinWrtPlatform) {
  float rotatedVec[3];
  matmulvec(rotatedVec, rotMat, pinWrtPlatform);
  vecAdd(pins, platformPos, rotatedVec, 3);
}


void currentSenseAll(float curr[6]) {
  for (int i = 0; i < 6; i++) {
    curr[i] = currentSense(CS[i]);
  }
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
    SPI.transfer(FILTER_2 | DISABLE_INDX | FREE_RUN | QUADRX4); // Filter clock division factor = 1 || Asynchronous Index ||
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

float counts2rad(float count, int cntsPerRev) {
  return count * 1.0 / cntsPerRev * 2 * PI;
}

float counts2rad(long count, int cntsPerRev) {
  return count * 1.0 / cntsPerRev * 2 * PI;
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

void SystemShutDown() {
  Serial.println("... INITIATING SHUTDOWN SEQUENCE ...");
  for (int i = 0; i < 6; i++) {
    disableMotor(i);
  }
  Serial.println("... SYSTEM POWER OFF ...");
  Serial.println("Click reset button to restart program.");
  while (true) {
    delay(10);
  }
}

void disableMotor(int i) {
  digitalWrite(PWM[i], LOW);
  digitalWrite(DIR[i], LOW);
}

float currentSense(int pin) {
  return (analogRead(pin) / 1023.0 * 5000.0 + 50.0) / 20.0;
}

void modifyPWMfrequency(int myPrescaler) {
  int myEraser = 7;
  TCCR4B &= ~myEraser; //Pins 8, 7, 6
  TCCR2B &= ~myEraser; //Pins 10, 9
  TCCR1B &= ~myEraser; //Pins 11, 12
  TCCR4B |= myPrescaler;
  TCCR2B |= myPrescaler;
  TCCR1B |= myPrescaler;
}

// FUNCTIONS USED TO SET UP THE SYSTEM
void initJoystickPins() {
  for (int i = 0; i < 6; i++) {
    pinMode(JS_INPUT[i], INPUT);
  }
}

void initMotorPins() {
  for (int i = 0; i < 6; i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
    pinMode(CS[i],  INPUT);
  }
}

void initCounterPins() {
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
}

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
