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


// Control Loop Characteristics
const float kp = 4.0; // [V/rad]
const float kd = 0.01; // [V/(rad/s)]
const float runsPerRead = 5; // number of control iterations per joystick reading

// Pins to Control BTS7960 Motor Driver
const byte RPWM_OUTPUT[] = {     0,     4,     6,     8,    10,    12}; // Change first num to 2
const byte LPWM_OUTPUT[] = {     1,     5,     7,     9,    11,    13}; // Change first num to 3
const byte REN_OUTPUT[]  = {    22,    23,    24,    25,    26,    27}; // PortA
const byte LEN_OUTPUT[]  = {    30,    31,    32,    33,    34,    35}; // PortB
const byte MOTOR_ON[]    = { false,  true, false, false, false, false}; // Switch motors on to run

// Pins to Read Potentiometers on Joystick
const byte JS_INPUT[] = {A0, A1, A2, A3, A4, A5};

// Encoder Specifications
// const int countsPerRev = 600;

// Variables what we update while running
float desiredCounts[6];
float currentCounts[6];
long lastFinish;
float lastCounts[6];

// Platform Pin Positions wrt Platform Coordinates {x, y, z, thz}
float platformSetup[6][4];

// Base Motor Positions wrt Base Coordinates {x, y, z, thz}
float baseSetup[6][4];

// TESTING VARIABLES: DELETE SOON
volatile int pololuCount = 0;
const int countsPerRev = 34*12*4;

void setup() {
  // Setting Up Pins
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  Serial.println("Setting Up");
  for (int i = 0; i < 6; i++) {
    pinMode(RPWM_OUTPUT[i], OUTPUT);
    pinMode(LPWM_OUTPUT[i], OUTPUT);
    pinMode(REN_OUTPUT[i],  OUTPUT);
    pinMode(LEN_OUTPUT[i],  OUTPUT);
    enableMotor(i);
  }

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
  Serial.println("Finished Set Up, beginning loop now...");

  // SETTING UP TESTING VARIABLES
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(0, chA, CHANGE);
  attachInterrupt(1, chB, CHANGE);

  lastFinish = micros();
  getCurrentCounts(lastCounts);
}




void loop() {
  getDesiredEncoderCounts(desiredCounts);
  controlled_move(desiredCounts, lastFinish, lastCounts, currentCounts, MOTOR_ON, runsPerRead);
}

boolean controlled_move(float desired[6], long lastEnd[6], float lastCnts[6], float current[6], byte motorOn[6], unsigned int iterations) {
  float error[6]; // The error in counts
  long start;
  int elapsedTime;
  float elapsedCounts;
  float ctrlSig = 0;

  for (int n = 0; n < iterations; n++) {
    for (int m = 0; m < 6; m++) {
      
      if (motorOn[m] == false) {
        continue;
      }
      
      start = micros();
      elapsedTime = start - lastEnd[m];
      lastEnd[m] = start;

      current[m] = getCount(m);
      elapsedCounts = current[m] - lastCnts[m];
      lastCnts[m] = current[m];

      error[m] = desired[m] - current[m];
      
      ctrlSig = kp * 2 * PI * error[m] / countsPerRev + kd * 2 * PI * elapsedCounts / countsPerRev / elapsedTime * 1000000;
      unsigned int PWMvalue = (int)constrain(abs(ctrlSig / 5 * 255), 0, 255);

      if (ctrlSig > 0) {
        analogWrite(LPWM_OUTPUT[m], 0);
        analogWrite(RPWM_OUTPUT[m], PWMvalue);
      } else if (ctrlSig < 0) {
        analogWrite(LPWM_OUTPUT[m], PWMvalue);
        analogWrite(RPWM_OUTPUT[m], 0);
      }
    }
  }
  return true;
}

void getCurrentCounts(float counts[6]) {
  for (int m = 0; m < 6; m++) {
    counts[m] = getCount(m); 
  }
}


float getCount(int motor) {
  // TODO: Implement this method to read i2c from LS7366 Shield
  return getCountTESTING(currentCounts[motor], motor);
}

float getCountTESTING(float count, int motor) {
  // return count + (desiredCounts[motor] - count) / (10.0 - kp);
  return pololuCount;
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
      counts[m] = -alpha / 2 / PI * countsPerRev;
    } else {
      counts[m] = alpha / 2 / PI * countsPerRev;
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
  Serial.println();
}


/*
   Read's a joystick's value and returns a value from -1.0 to 1.0
*/
float readJoystick(int pin) {
  float JSvalue = analogRead(pin);
  Serial.print(JSvalue);
  Serial.print(" ");
  JSvalue = (JSvalue - 512) / 512.0;
  
  return JSvalue;
}


void calculatePlatformPinWrtBase(float * pins, float * platformPos, float rotMat[3][3], float * pinWrtPlatform) {
  float rotatedVec[3];
  matmulvec(rotatedVec, rotMat, pinWrtPlatform);
  vecAdd(pins, platformPos, rotatedVec, 3);
}


// HELPER FUNCTIONS

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

// Interrupt functions for keeping track of the encoder
void chA () {
  if (digitalRead(2) != digitalRead(3)) {
    pololuCount++;
  }
  else {
    pololuCount--;
  }
}

void chB () {
  if (digitalRead(2) == digitalRead(3)) {
    pololuCount++;
  }
  else {
    pololuCount--;
  }
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
