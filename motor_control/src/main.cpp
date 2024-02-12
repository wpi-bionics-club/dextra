#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
long previousTime;

// put function declarations here:
void linearControl(int angleResolution,int startingAngle,int finalAngle,int timePerClock);
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  //int result = myFunction(2, 3);
  Serial.begin(9600);
  previousTime = millis();
}

void loop() {
  int timePerClock = 1000;
  int angleResolution = 10;
  int startingAngle = 0;
  int finalAngle = 180;
  // put your main code here, to run repeatedly:
  Serial.println("hello");

  while(1){
    if(((millis() - previousTime) >= timePerClock) ? previousTime = millis() : 0 ){
      linearControl(angleResolution, startingAngle, finalAngle, timePerClock);
      Serial.print("This is printing every second ");
      Serial.println(previousTime);
    }
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  millis();
  return x + y;
}


// a = delta theta / time in seconds
void linearControl(int angleIncrimint, int startingAngle, int finalAngle, int timeIncrimint){
  int goalTheta = startingAngle;

  while (startingAngle < finalAngle){
    goalTheta += angleIncrimint;
    // write(0, goalTheta);
  }

}