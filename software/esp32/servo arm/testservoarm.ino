#include <ESP32Servo.h>

Servo myservo; // Declare Servo variable
#define BUTTON_PIN 0 // Define button pin

int var = 0;
bool buttonState = false;

void setup()
{
  myservo.attach(33); // Attach servo to pin D22
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(115200);
}

void loop()
{
  // Read button state
  buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == HIGH)
  {
    // Button is pressed, wait for release
    while (digitalRead(BUTTON_PIN) == HIGH)
    {
      delay(200);
    }

    // Update var based on button press
    if (var < 3)
    {
      var++;
    }
    else
    {
      var = 0;
    }
  }

  // Perform actions based on var
  switch (var)
  {
  case 0:
  myservo.write(0); // สั่งให้ Servo หมุนไปองศาที่ 0
  Serial.println('0');
    // do something for var 0
    break;
  case 1:
  myservo.write(40); // สั่งให้ Servo หมุนไปองศาที่ 0
  Serial.println('90');
    // do something for var 1
    break;
  case 2:
  myservo.write(60); // สั่งให้ Servo หมุนไปองศาที่ 0
  Serial.println('180');

    // do something for var 2
    break;
  case 3:
  myservo.write(45); // สั่งให้ Servo หมุนไปองศาที่ 0
  Serial.println('90');

    // do something for var 3
    break;
  }

  delay(200); // Add a small delay for stability
}
