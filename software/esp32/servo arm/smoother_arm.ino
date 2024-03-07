#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

int servoNumber = 0;
int switch1;
float switch1Smoothed , switch1Prev , a , a6, a5,a4,a3,a2,a1;

void setup() {

  Serial.begin(115200);

  pinMode(0, INPUT_PULLUP);

  board1.begin();
  board1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {

  switch1 = digitalRead(0);      // read switch
  switch1 = switch1 * 100;        // multiply by 100

  // *** smoothing ***

  switch1Smoothed = (switch1 * 0.05) + (switch1Prev * 0.95);
  switch1Prev = switch1Smoothed;
  a6 = map(switch1Smoothed,1,100,30,90); // servo arm 6 
  a5 = map(switch1Smoothed,1,100,20,50); // servo arm 5
  a4 = map(switch1Smoothed,1,100,70,95); // servo arm 4
  a3 = map(switch1Smoothed,1,100,179,180); // servo arm 3 -still wrong direction
  a2 = map(switch1Smoothed,1,100,60,15); // servo arm 2 -still wrong direction 
  a1 = map(switch1Smoothed,1,100,179,2); // servo arm 1

  // Serial.println(switch1Smoothed);
  // Serial.println(switch1Prev);
  // Serial.println(a);
  // *** end of smoothing ***

  // Serial.print(switch1);                  // print to serial terminal/plotter
  // Serial.print(" , ");   
  // Serial.println(a);

  board1.setPWM(12, 0, angleToPulse(a6, 6)); // arm 6
  board1.setPWM(4, 0, angleToPulse(a5, 5)); // arm 5
  board1.setPWM(8, 0, angleToPulse(a4, 4)); // arm 4
  board1.setPWM(2, 0, angleToPulse(a3, 3)); // arm 3
  board1.setPWM(0, 0, angleToPulse(a2, 2)); // arm 2
  board1.setPWM(15, 0, angleToPulse(a1, 1)); // arm 1

  delay(20);                      // run loop 100 times/second

}
int angleToPulse(int ang, int arm){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("arm: ");Serial.print(arm);
   Serial.print(" Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}