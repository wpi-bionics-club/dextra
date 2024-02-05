#include <Arduino.h>
//long timeEleapsed = millis();
long previousTime;

// put function declarations here:
int myFunction(int, int);


void setup() {
  // put your setup code here, to run once:
  //int result = myFunction(2, 3);
  Serial.begin(9600);
  previousTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("hello");

  while(1){
    if(((millis() - previousTime) >= 1000) ? previousTime=millis() : 0 ){
      //previousTime = millis();
      //run your code
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