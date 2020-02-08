/**
   Encoder Test 1.0
   This is a simple movement test to determine if the encoder and LS7366R chip works as planned.
   The wiring diagram for this can be found in Zooey's notebook. This test uses the BTS7960 driver.
*/

#include <SPI.h>

// Control Loop Characteristics
const float kp = 1.0; // [V/rad]
const float ki = 0.0; // [V/(rad*s)]
const float kd = 0.0; // [V/(rad/s)]

// Pins to Control BTS7960 Motor Driver
const byte mn = 6; //Use this to select which motor setup to use
const int  speedLimit = 200; //Select a value from 0 to 255

const byte RPWM[] = {     2,     4,     6,     8,    10,    12}; 
const byte LPWM[] = {     3,     5,     7,     9,    11,    13};
const byte REN[]  = {    22,    24,    26,    28,    30,    32};
const byte LEN[]  = {    23,    25,    27,    29,    31,    33};
const byte RPWM_OUTPUT = RPWM[mn-1];
const byte LPWM_OUTPUT = LPWM[mn-1];
const byte REN_OUTPUT = REN[mn-1];
const byte LEN_OUTPUT = LEN[mn-1];

// Encoder Specifications
const int countsPerRev = 600; // if 4x multiply this number by 4

// Control Specs
const byte revButton = 51;
int spinDirection = 1;
const byte stepButton = 53;
float stepAngleDeg = 30;

// Variables that we update while running
long desiredCount;
long currentCount;
long lastFinish;
long lastCount;
long error;
float intError;
long lastError;
long lastStart;
long lastButtonPress;

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
int DFLAG_pin = 47;

//LFLAG
int LFLAG_pin = 48;

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
  Serial.begin(38400);
  Serial.println("... Setting Up ...");
  pinMode(revButton, INPUT_PULLUP);
  pinMode(stepButton, INPUT_PULLUP);
  pinMode(RPWM_OUTPUT, OUTPUT);
  pinMode(LPWM_OUTPUT, OUTPUT);
  pinMode(REN_OUTPUT,  OUTPUT);
  pinMode(LEN_OUTPUT,  OUTPUT);
  digitalWrite(REN_OUTPUT, HIGH);
  digitalWrite(LEN_OUTPUT, HIGH);
  
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
  delay(1000);
  
  Init_LS7366Rs();
  Serial.println("... Finished Setting Up Encoder Shield ...");
  lastStart = micros();
  lastButtonPress = micros();
  error = 0;
  Serial.println("... Finished Setup, Beginning Loop Now ...");
}


void loop() {
  if (micros() - lastButtonPress > 500000) {
    if (digitalRead(revButton) == LOW) {
      //Serial.println("revButton pressed");
      spinDirection = spinDirection * -1;
      lastButtonPress = micros();
    } else if (digitalRead(stepButton) == LOW) {
      //Serial.println("stepButton Pressed");
      desiredCount = desiredCount + spinDirection*rad2counts(deg2rad(stepAngleDeg), countsPerRev);
      lastButtonPress = micros();
    }
  }

  Serial.print(currentCount);
  Serial.print(" ");
  Serial.println(desiredCount);
  long start = micros();
  long dt = start - lastStart;
  lastStart = start;

  currentCount = getCount();
  error = desiredCount - currentCount;
  long derror = error - lastError;
  lastError = error;

  float de_dt = derror * 1.0/dt;

  intError = intError + error*dt;

  float ctrlSig = kp * counts2rad(error, countsPerRev) + kd * counts2rad(de_dt, countsPerRev) + ki * counts2rad(intError, countsPerRev);
  unsigned int PWMvalue = (int) constrain(abs(ctrlSig / 5 * 255), 0, speedLimit);
  
  if (ctrlSig < 0) {
    analogWrite(RPWM_OUTPUT, PWMvalue);
    analogWrite(LPWM_OUTPUT, 0);
  } else if (ctrlSig >= 0) {
    analogWrite(RPWM_OUTPUT, 0);
    analogWrite(LPWM_OUTPUT, PWMvalue);
  }
}

long getCount() {
  // TODO: Implement this method to read i2c from LS7366 Shield
  long result = getChanEncoderValue(mn);
  if (mn % 2 == 1) {
    result = result - 90 / 360.0 * countsPerRev;
  } else {
    result = result + 90 / 360.0 * countsPerRev;
  }
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
  //Serial.print("\r\n");

  //initialize the 6
  
  for (a = 1; a <= 6; a++)
  {
    //********
    setSSEnc(ENABLE, a);
    SPI.transfer(WRITE_MDR0);// Select MDR0 | WR register
    SPI.transfer(FILTER_2 | DISABLE_INDX | FREE_RUN | QUADRX1); // Filter clock division factor = 1 || Asynchronous Index ||
    // disable index || free-running count mode || x4 quadrature count mode
    setSSEnc(DISABLE, 0);

    Serial.print(" TX MDR0=");
    Serial.print(FILTER_2 | DISABLE_INDX | FREE_RUN | QUADRX1, HEX);
    Serial.print(";");
    Serial.print(" RX MDR0=");
    Serial.print(getChanEncoderReg(READ_MDR0, a), HEX);
    Serial.print(";");
    //********
    //********
    setSSEnc(ENABLE, a);
    SPI.transfer(WRITE_MDR1);// Select MDR1 | WR register
    SPI.transfer(CMP_FLAG | BYTE_4 | EN_CNTR); //4-byte counter mode || Enable counting || FLAG on CMP (B5 of STR)
    setSSEnc(DISABLE, 0);

    Serial.print(" TX MDR1=");
    Serial.print(CMP_FLAG | BYTE_4 | EN_CNTR, HEX);
    Serial.print(";");
    Serial.print(" RX MDR1=");
    Serial.print(getChanEncoderReg(READ_MDR1, a), HEX);
    Serial.print(";");
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
    Serial.println("... LS7366 initialized ...");
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

float deg2rad(float deg) {
  return deg * PI / 180.0;
}

float counts2rad(float count, int cntsPerRev) {
  return count * 1.0 / cntsPerRev * 2 * PI;
}

float counts2rad(long count, int cntsPerRev) {
  return count * 1.0 / cntsPerRev * 2 * PI;
}

float rad2counts(float rad, int cntsPerRev) {
  return rad / 2.0 / PI * cntsPerRev;
}

float rad2counts(long rad, int cntsPerRev) {
  return rad / 2.0 / PI * cntsPerRev;
}
