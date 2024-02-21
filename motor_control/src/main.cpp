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

// put function declarations here:
void linearControl(int angleResolution,int startingAngle,int finalAngle,int timePerClock);
void write(int servo_num, int angle);
void quintic(int startingAngle, int finalAngle, int interpolation_time);
void pid(int, int);

void setup() {
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
  Serial.begin(9600);
  write(0, 0);
  delay(200);
}

#define TIMEPERCLOCK 500
#define ANGLERESOLUTION 10
#define STARTINGANGLE 0
#define FINALANGLE 180

int loop_count = 0;

void loop() {
  delay(1000);
  if (loop_count == 0)
  {
    loop_count++;
    pid(0, 90);
  }
}

// a = delta theta / time in seconds
void linearControl(int angleIncrimint, int startingAngle, int finalAngle, int timeIncrimint){

  int goalTheta = startingAngle;
  previousTime = millis();

   while(goalTheta <= finalAngle){
    if((((millis() - previousTime) >= timeIncrimint) ? previousTime = millis() : 0) ){ // if dt is correct
      write(0, goalTheta);   
      Serial.println(goalTheta);     
      goalTheta += angleIncrimint;
    } 
    
  }

  // delay(1000);
}


void write(int servo_num, int angle) {
  
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  // Serial.println(pulse);
  pwm.setPWM(servo_num, 0, pulse);
}

void quintic(int start_theta, int final_theta, int interpolation_time) {
  
}

void pid(int start_theta, int final_theta) {
  Serial.println("I'm here!!");
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
      Serial.print("Control signal is: ");
      Serial.println(utility);

      current_angle += utility;

      // Move the motor
      write(0, current_angle);  // Assuming this function sends step commands to the motor
      time_start = millis();
    }
  }

  Serial.println("I'm out of here");
}


// void loop() {
//   // Drive each servo one at a time using setPWM()
//   Serial.println(servonum);
//   for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
//     pwm.setPWM(servonum, 0, pulselen);
//     delay(10);
//   }

//   delay(500);
//   for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
//     pwm.setPWM(servonum, 0, pulselen);
//     delay(10);
//   }

//   delay(500);
  
// }
