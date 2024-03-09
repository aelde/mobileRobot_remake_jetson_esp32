#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 125
#define SERVOMAX 575

const int LED_PIN = 2;
const int OE_PIN = 16;

//   a1.  a2.  a3.  a4. a5. a6.
int angles_Default[] = { 180, 0, 90, 180, 180, 90 };  // Default angles
int angles[] = { 180, 0, 90, 180, 180, 90 };          // Initial angles
int angleMins[] = { 110, 0, 0, 0, 80, 0 };
int angleMaxs[] = { 180, 90, 90, 180, 180, 180 };

int prevAngles[6] = { 180, 0, 90, 180, 180, 90 };

int step = 3;

int pre_OE = 1, cur_OE = 1;

int s = 0;
int smoothingFactor = 5;
// float smoothingFactor = 0.05;
float complementFactor = 0.95;
// float smoothAngles[6] = {0};
float smoothAngles[6] = { 180, 0, 90, 180, 180, 90 };
float prevSmooth[6] = { 0 };
float switch1Smoothed, switch1Prev, a6;

int aaa[6] = { 0, 0, 0, 0, 0, 0 };

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH);
  board1.begin();
  board1.setPWMFreq(60);
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    String jsonData = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, jsonData);
    if (error) {
      Serial.println("Error parsing JSON");
      return;
    }

    cur_OE = doc["OE"];
    if (cur_OE != pre_OE) {
      // digitalWrite(OE_PIN, cur_OE);
      if (cur_OE == 1) {
        Serial.println("im in bruhhhhhhhh!!!");
        for (int i = 0; i < 6; i++) {
          aaa[i] = prevAngles[i] = angles[i];
        //   angles[i] = smoothAngles[i] = prevAngles[i];
        }
      } else {
        for (int i = 0; i < 6; i++) {
          angles[i] = smoothAngles[i] = prevAngles[i] = angles_Default[i];
        }
      }
      digitalWrite(LED_PIN, cur_OE);
      pre_OE = cur_OE;
      s = (s == 100) ? 0 : 100;
    }

    if (cur_OE == 0 && angles[0] >= angleMins[0] && angles[0] <= angleMaxs[0])  // arm1
    {
      int R2_a1 = doc["R2_a1"];
      angles[0] = constrain(angles[0] - (R2_a1 * step), angleMins[0], angleMaxs[0]);
    }
    if (cur_OE == 0 && angles[1] >= angleMins[1] && angles[1] <= angleMaxs[1])  // arm2
    {
      int L2_a2 = doc["L2_a2"];
      angles[1] = constrain(angles[1] + (L2_a2 * step), angleMins[1], angleMaxs[1]);
    }
    if (cur_OE == 0 && angles[2] >= angleMins[2] && angles[2] <= angleMaxs[2])  // arm3
    {
      int L1_a3 = doc["L1_a3"];
      angles[2] = constrain(angles[2] - (L1_a3 * step), angleMins[2], angleMaxs[2]);
    }
    if (cur_OE == 0 && angles[3] >= angleMins[3] && angles[3] <= angleMaxs[3])  // arm4
    {
      int R1_a4 = doc["R1_a4"];
      angles[3] = constrain(angles[3] + (R1_a4 * step), angleMins[3], angleMaxs[3]);
    }
    if (cur_OE == 0 && angles[4] >= angleMins[4] && angles[4] <= angleMaxs[4])  // arm5
    {
      int L2_a5 = doc["L2_a5"];
      angles[4] = constrain(angles[4] - (L2_a5 * step), angleMins[4], angleMaxs[4]);
    }
    if (cur_OE == 0 && angles[5] >= angleMins[5] && angles[5] <= angleMaxs[5])  // arm6
    {
      int L1_a6 = doc["L1_a6"];
      angles[5] = constrain(angles[5] + (L1_a6 * step), angleMins[5], angleMaxs[5]);
    }
  }

  responseDoc["OE"] = cur_OE;
  for (int i = 0; i < 6; i++) {
    responseDoc["a" + String(i + 1)] = (cur_OE == 0) ? angles[i] : aaa[i];
  }
  String jsonResponse;
  serializeJson(responseDoc, jsonResponse);
  Serial.println(jsonResponse);


  setDefault2();


  if (cur_OE == 1) {
    for (int i = 0; i < 6; i++) {
      board1.setPWM(i, 0, angleToPulse(aaa[i], i));
    }
  } else {
    for (int i = 0; i < 6; i++) {
      board1.setPWM(i, 0, angleToPulse(angles[i], i));
    }
  }
  // printt(cur_OE);
  delay(20);
}

int angleToPulse(int ang, int arm) {
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}
void setDefault2() {
  for (int i = 0; i < 6; i++) {
    switch1Smoothed = (s * 0.05) + (switch1Prev * 0.95);
    switch1Prev = switch1Smoothed;
    aaa[i] = map(switch1Smoothed, 0, 100, angles_Default[i], prevAngles[i]);
  }
}
void printt(int OE) {
  Serial.print("OE:");
  Serial.print(cur_OE);

  for (int i = 0; i < 6; i++) {
    Serial.print(" a" + String(i + 1) + ":");
    Serial.print((OE == 0) ? angles[i] : aaa[i]);
  }
  Serial.print(" || ");
  Serial.print(prevAngles[0]);
  Serial.print(" ");
  Serial.print(prevAngles[1]);
  Serial.print(" ");
  Serial.print(prevAngles[2]);
  Serial.print(" ");
  Serial.print(prevAngles[3]);
  Serial.print(" ");
  Serial.print(prevAngles[4]);
  Serial.print(" ");
  Serial.print(prevAngles[5]);
  Serial.print(" || ");
  Serial.print(" a1:");
  Serial.print(aaa[0]);
  Serial.print(" a2:");
  Serial.print(aaa[1]);
  Serial.print(" a3:");
  Serial.print(aaa[2]);
  Serial.print(" a4:");
  Serial.print(aaa[3]);
  Serial.print(" a5:");
  Serial.print(aaa[4]);
  Serial.print(" a6:");
  Serial.print(aaa[5]);
  Serial.print(" s:");
  Serial.println(s);
}