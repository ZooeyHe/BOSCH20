/* Prelab 08 - Servomotor Position Control
   written by Zooey He and Randall Elkind
   11.06.2019

   The purpose of this program is to track the motor position,
   and use a proportionality constant to move the motor to the
   desired position.
*/


// Declaring Important Pins
const byte A = 2;  //initialize low interrupt pin
const byte B = 3;  //initialize high interrupt pin
const byte enPWMpin = 5;
const byte in1pin = 7;
const byte in2pin = 8;
const byte potReadPin = A0;

// Other Variables
volatile int count = 0; // encoder position           [counts]
float desiredCount = 0; // desired position           [counts]
float error = 0;        // error                      [rad]
float ctrlSig = 0;      // control signal             [V]
const float Kprop = 3;  // Proporationality Constant  [V/rad]
const float KD = 0.5;   // Derivative Constant        [V/(rad/s)]
const int deltaT = 10;  // loop delay                 [ms]

unsigned long previousTime = millis();
float prevError = 0;

void setup() {
  Serial.begin(9600);
  //declare pins for input and output
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(enPWMpin, OUTPUT);
  pinMode(in1pin, OUTPUT);
  pinMode(in2pin, OUTPUT);
  pinMode(potReadPin, INPUT);

  //Initialize the direction of motor spin
  digitalWrite(in1pin, HIGH);
  digitalWrite(in2pin, LOW);

  //Set interrupts calling switchLED function and to trigger when falling
  attachInterrupt(0, chB, CHANGE);
  attachInterrupt(1, chA, CHANGE);
}

void loop() {
  // Gets the desired count from potentiometer and calculates error and derivative error
  desiredCount = analogRead(potReadPin) / 1023.0 * 1632.0;
  error = (count - desiredCount) / 1632.0 * 2 * PI;
  float errDeriv = (error-prevError)/deltaT * 1000.0;
  
  // Creates the control signal from Kprop and KD, then writes the direction
  ctrlSig = Kprop * error + KD * errDeriv;
  if (ctrlSig > 0) {
    digitalWrite(in1pin, HIGH);
    digitalWrite(in2pin, LOW);
  } else if (ctrlSig < 0) {
    digitalWrite(in1pin, LOW);
    digitalWrite(in2pin, HIGH);
  }

  // Writes a PWM to the enable pin of the H-Bridge
  unsigned int enPWMvalue = (int)constrain(abs(ctrlSig / 5 * 255), 0, 255);
  //Serial.println(enPWMvalue);
  analogWrite(enPWMpin, enPWMvalue);

  // Regulates the loop rate
  while (millis() - previousTime < deltaT) {}
  previousTime = millis();
  

  
  
  prevError = error;
}

// Interrupt functions for keeping track of the encoder
void chA () {
  if (digitalRead(A) != digitalRead(B)) {
    count++;
  }
  else {
    count--;
  }
}

void chB () {
  if (digitalRead(A) == digitalRead(B)) {
    count++;
  }
  else {
    count--;
  }
}
