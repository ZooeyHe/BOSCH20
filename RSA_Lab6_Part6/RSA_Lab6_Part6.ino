//Randall Elkind, Zooey He Lab 6 Part C
//10.23.19

int A = 2;  //initialize low interrupt pin
int B = 3;  //initialize high interrupt pin
volatile int i = 0; //function counter for position
int pwmPin = 9;
unsigned long startTime = micros();
int startCount = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //set Baud rate

  //declare pins for input channels A and B
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(pwmPin, OUTPUT);

  //Set interrupts calling functions and to trigger when change is noticed 
  attachInterrupt(0, chA, CHANGE);  
  attachInterrupt(1, chB, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(pwmPin, HIGH);
 unsigned long bigLoopStart = millis();
 while (millis() - bigLoopStart < 5000) {
   unsigned long currentTime = micros();
   int currentCount = i;
   float velocity = (currentCount - startCount) * 1000000/(currentTime - startTime);
  
  
   velocity = velocity / 24 * PI;
  
   //Serial.print("Velocity: ");
   Serial.println(velocity);
   //Serial.println(" rad/s");
   startTime = currentTime; 
   startCount = currentCount;
   while (micros() - startTime < 20000){}
 }
 digitalWrite(pwmPin, LOW);
 unsigned long lowTime = millis();
 while(millis() - lowTime < 5000)  {}
 startTime = millis();
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
