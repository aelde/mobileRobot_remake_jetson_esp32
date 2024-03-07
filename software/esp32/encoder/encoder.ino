#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  125
#define SERVOMAX  575

const int LED_PIN = 2;  
int a6 = 0,a5 = 0,a4 = 0,a3 = 0,a2 = 0,a1 = 0;
int a6_max = 180, a6_min = 0; // 180 start - 10 end
int a5_max = 180, a5_min = 80; // 180 start - 0 end
int a4_max = 180, a4_min = 0; // 180 start - 0 end
int a3_max = 90, a3_min = 0; // 90 start - 0 end
int a2_max = 90, a2_min = 0; // reversed servo 0 - 90
int a1_max = 180, a1_min = 110; // 180 start - 110 end
int L1 = 90;
int L2 = a5_max, R1 = a4_max, R2 = a1_max;
int Step = 3;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  board1.begin();
  board1.setPWMFreq(60);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming JSON data
    String jsonData = Serial.readStringUntil('\n');

    // Parse the JSON
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.println("Error parsing JSON");
      return;
    }

    // Access individual values from the received JSON
    if (L1 >= a6_min && L1 <= a6_max) { // a6
      int l11 = doc["L1"];
      L1 = constrain(L1 - (l11*Step), a6_min, a6_max);
    }
    if (L2 >= a5_min && L2 <= a5_max) { // a5
      int l22 = doc["L2"];
      L2 = constrain(L2 - (l22*Step), a5_min, a5_max);
    }
    if (R1 >= a4_min && R1 <= a4_max) {
      int r11 = doc["R1"];
      R1 = constrain(R1 - (r11*Step), a4_min, a4_max);
    }
    // if (R2 >= a3_min && R2 <= a3_max) {
    //   int r22 = doc["R2"];
    //   R2 = constrain(R2 - (r22*Step), a3_min, a3_max);
    // }
    // if (R2 >= a2_min && R2 <= a2_max) {
    //   int r22 = doc["R2"];
    //   R2 = constrain(R2 - (r22*Step), a2_min, a2_max);
    // }
    if (R2 >= a1_min && R2 <= a1_max) {
      int r22 = doc["R2"];
      R2 = constrain(R2 - (r22*Step), a1_min, a1_max);
    }

    // Prepare response JSON
    // responseDoc["status"] = "OK";
    // responseDoc["L1"] = L1; // Echo back the received values
    // responseDoc["L2"] = L2;
    // responseDoc["R1"] = R1;
    // responseDoc["R2"] = R2;
    responseDoc["a6"] = L1; // Echo back the received values
    responseDoc["a5"] = L2;
    responseDoc["a4"] = R1;
    responseDoc["a1"] = R2;

    // Serialize the JSON response
    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);

    // Send the response back to the serial port
    Serial.println(jsonResponse);
    
  }

  // a6 = map(L1, 1, 100, a6_min, a6_max); // 120 - 20
  // a5 = map(L2, 1, 100, a5_min, a5_max); // 
  // a4 = map(R1, 1, 100, a4_min, a4_max);
  // a3 = map(switch1Smoothed,1,100,179,180); // servo arm 3 -still wrong direction
  // a2 = map(switch1Smoothed,1,100,60,15); // servo arm 2 -still wrong direction 
  // a1 = map(R2, 1, 100, a1_min, a1_max);

  board1.setPWM(15, 0, angleToPulse(L1, 6));
  board1.setPWM(4, 0, angleToPulse(L2, 5)); // arm 5
  board1.setPWM(8, 0, angleToPulse(R1, 4)); // arm 4 // servo heating while moving
  // board1.setPWM(2, 0, angleToPulse(a3, 3)); // arm 3 // servo need to change
  // board1.setPWM(0, 0, angleToPulse(a2, 2)); // arm 2 // servo heating while moving
  board1.setPWM(0, 0, angleToPulse(R2, 1)); // arm 1

  delay(20);
}
int angleToPulse(int ang, int arm) {
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}
