#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  125
#define SERVOMAX  575

int L1 = 100, L2 = 100, R1 = 100, R2 = 100;

void setup() {
  Serial.begin(115200);
  pinMode(0, INPUT_PULLUP);
  board1.begin();
  board1.setPWMFreq(60);
}

void loop() {
  if (Serial.available() > 0) {
    String jsonData = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.println("Error parsing JSON");
      return;
    }

    if (L1 >= 1 && L1 <= 100) {
      int l11 = doc["L1"];
      L1 = constrain(L1 - l11, 1, 100);
    }

    if (L2 >= 1 && L2 <= 100) {
      int l22 = doc["L2"];
      L2 = constrain(L2 - l22, 1, 100);
    }

    if (R1 >= 1 && R1 <= 100) {
      int r11 = doc["R1"];
      R1 = constrain(R1 - r11, 1, 100);
    }
    if (R2 >= 1 && R2 <= 100) {
      int r22 = doc["R2"];
      R2 = constrain(R2 - r22, 1, 100);
    }

    responseDoc["L1"] = L1;
    responseDoc["L2"] = L2;
    responseDoc["R1"] = R1;
    responseDoc["R2"] = R2;

    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    // Serial.println(jsonResponse);
  }

  int a6 = map(L1, 1, 100, 30, 90);
  int a5 = map(L2, 1, 100, 20, 50);
  int a4 = map(R1, 1, 100, 70, 95);
  // a3 = map(switch1Smoothed,1,100,179,180); // servo arm 3 -still wrong direction
  // a2 = map(switch1Smoothed,1,100,60,15); // servo arm 2 -still wrong direction 
  int a1 = map(R2, 1, 100, 110, 180);

  board1.setPWM(12, 0, angleToPulse(a6, 6));
  board1.setPWM(4, 0, angleToPulse(a5, 5)); // arm 5
  board1.setPWM(8, 0, angleToPulse(a4, 4)); // arm 4
  // board1.setPWM(2, 0, angleToPulse(a3, 3)); // arm 3
  // board1.setPWM(0, 0, angleToPulse(a2, 2)); // arm 2
  board1.setPWM(15, 0, angleToPulse(a1, 1)); // arm 1

  Serial.print("a6:"); //L1
  Serial.print(a6);
  Serial.print(", ");
  Serial.print("a5:"); //L2
  Serial.print(a5);
  Serial.print(", ");
  Serial.print("a4:"); //R1
  Serial.print(a4);
  Serial.print(", ");
  Serial.print("a1:"); //R2
  Serial.print(a1);
  Serial.println(", ");
  
  delay(20);
}

int angleToPulse(int ang, int arm) {
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}
