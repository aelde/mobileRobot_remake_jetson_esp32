 
#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>
#define BUTTON_PIN 0 // Define button pin

int var = 180;
bool buttonState = false;


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
// Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
// Watch video V1 to understand the two lines below: http://youtu.be/y8X9X10Tn1k
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)


int servoNumber = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("32 channel Servo test!");
  pinMode(BUTTON_PIN, INPUT);
  board1.begin();
  // board2.begin();  
  board1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  // board2.setPWMFreq(60);
  //yield();
}

// the code inside loop() has been updated by Robojax
void loop() {
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH)
  {
  //  Serial.println("hi");
    // Button is pressed, wait for release
    while (digitalRead(BUTTON_PIN) == HIGH)
    {
      delay(200);
    }

    // Update var based on button press
    // if (var < 3)
    // {
    //   var++;
    // }
    // else
    // {
    //   var = 0;
    // }
    if (var < 1 ){
      servoNumber = 1;
    }
    if (servoNumber == 1){
      var += 10;
      if(var > 179){
      servoNumber = 0;
    }
    } else {
      var -= 10;
    }
    Serial.println(servoNumber);
  }
  board1.setPWM(0, 0, angleToPulse(var));
  Serial.println(var);
  // Perform actions based on var
  // switch (var)
  // {
  // case 0:
  // // myservo.write(0); // สั่งให้ Servo หมุนไปองศาที่ 0
  // board1.setPWM(0, 0, angleToPulse(0));
  // Serial.println("0");
  //   // do something for var 0
  //   break;
  // case 1:
  // // myservo.write(40); // สั่งให้ Servo หมุนไปองศาที่ 0
  // board1.setPWM(0, 0, angleToPulse(60));

  // Serial.println("60");
  //   // do something for var 1
  //   break;
  // case 2:
  // // myservo.write(60); // สั่งให้ Servo หมุนไปองศาที่ 0
  // board1.setPWM(0, 0, angleToPulse(90));

  // Serial.println("90");

  //   // do something for var 2
  //   break;
  // case 3:
  // board1.setPWM(0, 0, angleToPulse(180));

  // // myservo.write(45); // สั่งให้ Servo หมุนไปองศาที่ 0
  // Serial.println("180");

  //   // do something for var 3
  //   break;
  // }

  delay(200); // Add a small delay for stability

 
}

/*
 * angleToPulse(int ang)
 * gets angle in degree and returns the pulse width
 * also prints the value on seial monitor
 * written by Ahmad Nejrabi for Robojax, Robojax.com
 */
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
  //  Serial.print("Angle: ");Serial.print(ang);
  //  Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}
 