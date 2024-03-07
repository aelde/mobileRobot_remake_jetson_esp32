#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  125
#define SERVOMAX  575

const int LED_PIN = 2; 
const int OE_PIN = 16; 
int a6 = 0,a5 = 0,a4 = 0,a3 = 0,a2 = 0,a1 = 0;
int a6_max = 180, a6_min = 0; // 180 start - 10 end
int a5_max = 180, a5_min = 80; // 180 start - 80 end(maximum for stall 15 kg load)-> dss m15
int a4_max = 180, a4_min = 0; // 180 start - 0 end
int a3_max = 90, a3_min = 0; // 90 start - 0 end
int a2_max = 90, a2_min = 0; // reversed servo 0 - 90
int a1_max = 180, a1_min = 110; // 180 start - 110 end
int L1_a6 = 90
int L2_a5 = a5_max
int R1_a4 = a4_max
int R2_a1 = a1_max;
int L1_a3 = a3_max;
int L2_a2 = 0;
int Step = 3;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH);
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
    if (L1_a6 >= a6_min && L1_a6 <= a6_max) { // a6
      int L1_a61 = doc["L1_a6"];
      L1_a6 = constrain(L1_a6 - (L1_a61*Step), a6_min, a6_max);
    }
    if (L2_a5 >= a5_min && L2_a5 <= a5_max) { // a5
      int L2_a52 = doc["L2_a5"];
      L2_a5 = constrain(L2_a5 - (L2_a52*Step), a5_min, a5_max);
    }
    if (R1_a4 >= a4_min && R1_a4 <= a4_max) {
      int R1_a41 = doc["R1_a4"];
      R1_a4 = constrain(R1_a4 - (R1_a41*Step), a4_min, a4_max);
    }
    if (L1_a3 >= a3_min && L1_a3 <= a3_max) {
      int L1_a32 = doc["L1_a3"];
      L1_a3 = constrain(L1_a3 - (L1_a32*Step), a3_min, a3_max);
    }
    if (L2_a2 >= a2_min && L2_a2 <= a2_max) {
      int L2_a22 = doc["L2_a2"];
      L2_a2 = constrain(L2_a2 - (L2_a22*Step), a2_min, a2_max);
    }
    if (R2_a1 >= a1_min && R2_a1 <= a1_max) {
      int R2_a12 = doc["R2_a1"];
      R2_a1 = constrain(R2_a1 - (R2_a12*Step), a1_min, a1_max);
    }
    // Echo back the received values
    responseDoc["a6"] = L1_a6; 
    responseDoc["a5"] = L2_a5;
    responseDoc["a4"] = R1_a4;
    responseDoc["a3"] = L1_a3;
    responseDoc["a2"] = L2_a2;
    responseDoc["a1"] = R2_a1;

    // Serialize the JSON response
    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);

    // Send the response back to the serial port
    Serial.println(jsonResponse);
    
  }

  board1.setPWM(15, 0, angleToPulse(L1_a6, 6));
  board1.setPWM(4, 0, angleToPulse(L2_a5, 5)); // arm 5
  board1.setPWM(8, 0, angleToPulse(R1_a4, 4)); // arm 4 // servo heating while moving
  board1.setPWM(2, 0, angleToPulse(L1_a3, 3)); // arm 3 // servo need to change
  board1.setPWM(0, 0, angleToPulse(L2_a2, 2)); // arm 2 // servo heating while moving
  board1.setPWM(0, 0, angleToPulse(R2_a1, 1)); // arm 1

  delay(20);
}
int angleToPulse(int ang, int arm) {
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}
