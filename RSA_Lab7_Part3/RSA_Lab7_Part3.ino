/**
 * Zooey He, Randall Elkind | 10.30.2019 | Part3
 */

#include <SPI.h> // SPI Library
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // Begin an RF24 transceiver instance called "radio",
// where the CE and CSN pins on the nRF24L01 board
// are connected to pins 7 and 8 on the Arduino.
const byte rxAddr[6] = "00001"; // receiver "pipe" address
const byte txAddr[6] = "00011"; // transceiver "pipe" address
char myCode = 'a';

enum States {initialize, listening, incrementCharacter, transmitCharacter};
States myState = initialize;

const int incrButton = 5;
const int transmitButton = 6;

void setup() {
  Serial.begin(9600);
  radio.begin(); // begin transceiver communication
  radio.openReadingPipe(1, rxAddr); // pipe number 1 is opened for listening
  radio.startListening(); // begin listening

  pinMode(incrButton, INPUT_PULLUP); // initialize the increment pin as pullup
  pinMode(transmitButton, INPUT_PULLUP); // initialize the transmit pin as pullup
}
void loop() {
  // Our finite state machine switch statement
  switch (myState) {
    case initialize: // Initial state, just move to listening...
      myState = listening;
      break;
    case listening: // reads the radio signal, searches for transmission signal.
      if (radio.available()){ //If data is recieved...
        char text[32] = {0}; // make an array of characters to store text that is read
        radio.read(&text, sizeof(text)); // read data into text character array
        Serial.println(text); // print received text to Serial monitor
      }

      if (digitalRead(incrButton) == LOW){ // if button 1 pressed
        delay(500); // delay to prevent multiple repeated presses
        myState = incrementCharacter; // change state to incrementCharacter
      }
      if (digitalRead(transmitButton) == LOW){ // if button 2 is pressed
        delay(500); // delay to prevent multiple repeated presses
        myState = transmitCharacter; // change state to transmitCharacter
      }
      break;

    case incrementCharacter:
      myCode++; // Increment our secret code
      if (myCode > 106) { // if secret code > 'j' return to 'a'
        myCode = 97;
      }
      Serial.print("Changed hack to "); // helpful message for our code
      Serial.println((char) myCode);
      myState = listening; // Return to listening state
      break;
      
    case transmitCharacter:
      radio.setRetries(15, 15); // Set 15 tries to transmit signal
      radio.openWritingPipe(txAddr); // opens the writing pipe
      radio.stopListening(); // stops listening to prepare for writing 
      radio.write(&myCode, sizeof(myCode)); // writes the code over the radio
      radio.startListening(); // Resume listening again
      myState = listening; // Return ot listening state.
      break;

  }
}
