#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BusIO_Register.h>
#include <Wire.h>
#include <math.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  475 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
long previousTime;
int prev_ServoPos = 0
long time_start = 0 
int ServoPos = 0;
// put function declarations here:
void pid(int, int);

void setup() {
  Serial.begin(9600); //begins serial monitor
  pinMode(A0, INPUT); //declares flexsensor as input

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  write(0, 0); // initilaizing servo motor
  delay(200);  
}

void write(int servo_num, int angle) {
  
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo_num, 0, pulse);
}

void pid(int start_theta, int final_theta) {
  float error = -1;
  float current_angle = start_theta;
  float error_dt = -1;
  float prev_error = 0;
  float error_sum = 0;
  float kp = 0.4;
  float ki = 0.1;
  float kd = 0;

  long time_start = millis();

  while (current_angle < final_theta) {
    error = final_theta - current_angle;

    if (millis() - time_start >= 100) {
      Serial.print("Error is: ");
      Serial.println(error);

      error_dt = error - prev_error;
      prev_error = error;

      if(abs(error) < 1) {
        error_sum += error;
      }

      float utility = kp * error + ki * error_sum;  // angle we want to increment by

      current_angle += utility;

      // Move the motor
      write(0, current_angle);  // Assuming this function sends step commands to the motor
      time_start = millis();
    }
  }

}


void loop() {

    int FlexVal = analogRead(A0); //reads value of sensor
    Serial.println(FlexVal);

    ServoPos = map(FlexVal, 550, 800, 180, 0); //maps sensor input to servo range of motion
    
    if (millis() - time_start >= 50) {  // waits for 50ms to read data can change later
        pid(prev_ServoPos, ServoPos);
        prev_ServoPos = ServoPos
        time_start = millis()
}
}