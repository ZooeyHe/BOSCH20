/*
 * Zooey He, Randall Elkind | RSALab07 | 10.30.2019 | Part5
 */

//Importing the necessary libraries
#include <Wire.h>
#include <LIS3MDL.h>
#include <math.h>
#include <EEPROM.h>
#include <EEWrap.h>

LIS3MDL mag;
int16_e minX EEMEM;
int16_e minY EEMEM;
int16_e minZ EEMEM;
int16_e maxX EEMEM;
int16_e maxY EEMEM;
int16_e maxZ EEMEM;

// Enum for the state
enum States {initialize, show, cali};
States myState = initialize;

// Declare a button
int button = 5;

// Sets everything up
void setup()
{
  minX = -32000;
  minY = -32000;
  minZ = -32000;
  maxX = 32000;
  maxY = 32000;
  maxZ = 32000;
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Thinking1");
  pinMode(button, INPUT_PULLUP);
  mag.init();
  mag.enableDefault();
}

//Loop
void loop()
{
  // Checks through myState
  switch (myState) {
    case initialize:
      //Goes to the calibration mode when the button is pressed
      if (digitalRead(button) == LOW) {
        delay(500);
        myState = cali;
        maxX = -32000;
        maxY = -32000;
        maxZ = -32000;
        minX = 32000;
        minY = 32000;
        minZ = 32000;
      }
      break;
      //Calibration mode
    case cali:
      //Continuously finds min/max values of each direciton
      mag.read();
      minX = min(minX, mag.m.x);
      minY = min(minY, mag.m.y);
      minZ = min(minZ, mag.m.z);
      maxX = max(maxX, mag.m.x);
      maxY = max(maxY, mag.m.y);
      maxZ = max(maxZ, mag.m.z);

      //Prints out the calibration data for users
      Serial.print("min ");
      Serial.print(minX);
      Serial.print("  ");
      Serial.print(minY);
      Serial.print("  ");
      Serial.println(minZ);
      Serial.print(maxX);
      Serial.print("  ");
      Serial.print(maxY);
      Serial.print("  ");
      Serial.println(maxZ);

      //Goes to "Show" when the button is pressed
      if (digitalRead(button) == LOW) {
        delay(500);
        myState = show;
      }
      break;

    // Show
    case show:
      //Reads then calculates the directions in the xy plane.
      mag.read();
      float scaledX = 2.0 * (mag.m.x - minX) / (maxX - minX);
      float scaledY = 2.0 * (mag.m.y - minY) / (maxY - minY);
      float scaledZ = 2.0 * (mag.m.z - minZ) / (maxZ - minZ);

      // Calculating and printing the angle
      float theta = atan2(scaledY, scaledX) * 180 / PI;
      Serial.println(theta);

      //Goes to calibration mode when the button is pressed
      if (digitalRead(button) == LOW) {
        delay(500);
        myState = cali;
        maxX = -32000;
        maxY = -32000;
        maxZ = -32000;
        minX = 32000;
        minY = 32000;
        minZ = 32000;
      }
      break;
  }
  delay(500);
}
