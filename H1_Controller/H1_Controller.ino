/*
 * H1_Controller takes a joystick's input
 */

// Hexapod Dimensions [cm, degrees]
const byte Rp = 15;
const byte minp = 50;
const byte Rb = 18;
const byte minb = 50;
const byte a =6;
const byte s = 18;

// Hexapod Motion Envelope Limitations {x, y, z, thx, thy, thz} [cm, degrees]
const byte platformLimits[] = {3, 3, 3, 5, 5, 5};

// Hexapod Limitations [degrees]
const byte motorAngleLim[] = {-30, 90};

// Pins to Control BTS7960 Motor Driver
const byte RPWM_OUTPUT[] = {  2,  4,  6,  8, 10, 12};
const byte LPWM_OUTPUT[] = {  3,  5,  7,  9, 11, 13};
const byte REN_OUTPUT[]  = { 22, 23, 24, 25, 26, 27}; // PortA
const byte LEN_OUTPUT[]  = { 30, 31, 32, 33, 34, 35}; // PortB

// Pins to Read Potentiometers on Joystick
const byte JS_INPUT[] = {A0, A1, A2, A3, A4, A5};

// Encoder Specifications
const int countsPerRev = 600;



void setup() {
  // 1. Setting Up Pins
  for (int i = 0; i < 6; i++) {
    pinMode(RPWM_OUTPUT[i], OUTPUT);
    pinMode(LPWM_OUTPUT[i], OUTPUT);
    pinMode(REN_OUTPUT[i],  OUTPUT);
    pinMode(LEN_OUTPUT[i],  OUTPUT);
  }
  
}

void loop() {
  int desiredCounts[6];
  getDesiredEncoderCounts(desiredCounts);
}


void getDesiredEncoderCounts(int * counts) {
  float desiredPlatformPosition[6];
  getDesiredPlatformPosition(desiredPlatformPosition);
}


void getDesiredPlatformPosition(float * pos) {
  int joystickAnalogReadings[6];
  for (int d = 0; d < 6; d++) {
    float JSreading = readJoystick(JS_INPUT[d]);
    pos[d] = platformLimits[d] * JSreading;
  }
}

/*
 * Read's a joystick's value and returns a value from -1.0 to 1.0
 */
int readJoystick(int pin) {
  float JSvalue = analogRead(pin);
  JSvalue = (JSvalue - 512) / 512.0;
  return JSvalue
}
