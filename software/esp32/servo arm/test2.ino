#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

// L1 = a6, L2 = a5, R1 = a4, R2 = a5
int L1,L2,R1,R2,sum;

int servoNumber = 0;
int switch1 = 0;
float switch1Smoothed , switch1Prev , a , a6, a5,a4,a3,a2,a1,sm,pm;
// int switch1 = 0;
float m = 90.00;
void setup() {

  Serial.begin(115200);

  pinMode(0, INPUT_PULLUP);

  board1.begin();
  board1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {
  if (Serial.available() > 0) {
    Serial.println("gi");
    // Read the incoming JSON data
    String jsonData = Serial.readStringUntil('\n');

    // Parse the JSON
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.println("Error parsing JSON");
      return;
    }

    // switch1 = digitalRead(0);      // read switch
    // switch1 = switch1 * 100;        // multiply by 100
    // switch1Smoothed = (switch1 * 0.05) + (switch1Prev * 0.95);
    // switch1Prev = switch1Smoothed;
    // Process the received data
    // int values[4];
    L1 = doc["L1"];
    L2 = doc["L2"];
    R1 = doc["R1"];
    R2 = doc["R2"];
  // {"L1":1,"L2":2,"R1":3,"R2":4}
    // for (int i = 0; i < 4; i++) {
    //   values[i] = doc["values"][i];
    // }

    // Perform some operation on the received values
    // int sum = 0;
    // for (int i = 0; i < 4; i++) {
      // sum += values[i];
    // }
    sum = L1+L2+R1+R2;
    // Serial.print(switch1);                  // print to serial terminal/plotter
    // Serial.print(" , ");   
    // Serial.print(switch1Smoothed);                  // print to serial terminal/plotter
    // Serial.print(" , ");   
    // Serial.println(switch1Prev);
    // Create a JSON response
    responseDoc["sum"] = sum;
    // Serialize the JSON response
    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    // Send the response back to the serial port
    Serial.println(jsonResponse);

    if(sum == 10){
      Serial.println("shit");
      if (digitalRead(2) == 1){
        digitalWrite(2,0);
      } else {
        digitalWrite(2,1);
      }
    }

  } //sdsdwdsdsdsdsdsdsdssdsdsdsdwdsdsdsdsdsdsdssdsdsdsdwdsdsdsdsdsdsdssdsd

  // switch1 = digitalRead(0);      // read switch
  // switch1 = switch1 * 100;        // multiply by 100


  // // *** smoothing ***

  // switch1Smoothed = (switch1 * 0.05) + (switch1Prev * 0.95);
  // switch1Prev = switch1Smoothed;
  
  // a6 = map(switch1Smoothed,1,100,30,90); // servo arm 6
  // int a6_1 = map(switch1Smoothed,1,100,30,80); // servo arm 6
  // int a6_2 = map(switch1Smoothed,1,100,30,70); // servo arm 6
  if (switch1 == 0){
    m -= 0.5;
    if (m < 2){
      Serial.println("ga");
      m = 1;
      switch1 = 1;
    }
  } else {
      m+=0.5;
    if (m > 91){
      Serial.println("gafff");
      m = 90;ฟ
      switch1 = 0;
    } 
  }
 
 
    sm = (m * 0.05) + (pm * 0.95);
  pm = sm;
 // a5 = map(switch1Smoothed,1,100,20,50); // servo arm 5
  // a4 = map(switch1Smoothed,1,100,70,95); // servo arm 4
  // a3 = map(switch1Smoothed,1,100,179,180); // servo arm 3 -still wrong direction
  // a2 = map(switch1Smoothed,1,100,60,15); // servo arm 2 -still wrong direction 
  // a1 = map(switch1Smoothed,1,100,110,180); // servo arm 1

  // // Serial.println(switch1Smoothed);
  // // Serial.println(switch1Prev);
  // // Serial.println(a);
  // // *** end of smoothing ***

  Serial.print(m);                  // print to serial terminal/plotter
  Serial.print(" , ");   
    Serial.print(sm);                  // print to serial terminal/plotter
  Serial.print(" , ");   
  Serial.print(pm);
  //   Serial.print(" , "); 
  //   Serial.print(a6);                  // print to serial terminal/plotter
  //     Serial.print(" , ");   
  // Serial.print(a6_1);
  //   Serial.print(" , ");   
  // Serial.print(a6_2);
      Serial.print(" , ");   
  Serial.println(switch1);
 
  // board1.setPWM(12, 0, angleToPulse(a6, 6)); // arm 6
  // board1.setPWM(4, 0, angleToPulse(a5, 5)); // arm 5
  // board1.setPWM(8, 0, angleToPulse(a4, 4)); // arm 4
  // board1.setPWM(2, 0, angleToPulse(a3, 3)); // arm 3
  // board1.setPWM(0, 0, angleToPulse(a2, 2)); // arm 2
  // board1.setPWM(15, 0, angleToPulse(a1, 1)); // arm 1

  delay(20);                      // run loop 100 times/second

}
int angleToPulse(int ang, int arm){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
  //  Serial.print("arm: ");Serial.print(arm);
  //  Serial.print(" Angle: ");Serial.print(ang);
  //  Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}
// String ppp(){

// }