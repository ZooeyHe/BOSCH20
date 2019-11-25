//Randall Elkind, Zooey He Lab 6 Part C
//10.23.19

int A = 2;  //initialize low interrupt pin
int B = 3;  //initialize high interrupt pin
volatile int i = 0; //function counter for position
int pwmPin = 9;  //initialize PWM pin
unsigned long startTime = micros(); //get start time
int startCount = 0; //initialize starting counter


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //set Baud rate

  //declare pins for input channels A, B, and PWM
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(pwmPin, OUTPUT);

  //Set interrupts calling functions and to trigger when change is noticed 
  attachInterrupt(0, chA, CHANGE);  
  attachInterrupt(1, chB, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(pwmPin, HIGH);  //set PWM pin to high 
 unsigned long bigLoopStart = millis(); //get start time for the current loop
 while (millis() - bigLoopStart < 5000) {  //let the loop run for 5 sec
   unsigned long currentTime = micros();  //get current time
   int currentCount = i;  //set current count equal to counter
   float velocity = (currentCount - startCount) * 1000000/(currentTime - startTime); //velocity equation
  
  
   velocity = velocity / 24 * PI;  //convert velocity to rad/s
  
   Serial.println(velocity);  //print velocity to serial plotter

   //update current and start time for next iteration of the loop
   startTime = currentTime; 
   startCount = currentCount;
   //set constant time between velocity updates
   while (micros() - startTime < 20000){}
 }
 digitalWrite(pwmPin, LOW);  //turn pulse low for 5 seconds
 unsigned long lowTime = millis();
 while(millis() - lowTime < 5000)  {}
 startTime = millis();  //update start time for next iteration 
}


// function for interrupts to update the counter each time the encoder moves another step
void chA () {
  if (digitalRead(A) != digitalRead(B)) {
    i++;
  }
  else {
    i--;
  }
}

void chB () {
   if (digitalRead(A) == digitalRead(B)) {
    i++;
  }
  else {
    i--;
  }
}
