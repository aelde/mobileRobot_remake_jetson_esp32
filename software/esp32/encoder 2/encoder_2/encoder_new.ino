#include <Wire.h>
#include <ArduinoJson.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

#include <SoftwareSerial.h>
//#include <NewPing.h>
// Incremental Encoder Spec   30 ppr
// define for motor driver connection via SoftwareSerial.h
#define TXF 23
#define TXR 13
#define RXF 15
#define RXR 15

// define for wheel encoder sensor
#define FLA 34
#define FLB 35
// #define FLA 35
// #define FLB 34

#define FRA 32
#define FRB 33
#define RLA 25 // RLA
#define RLB 26 // RLB
#define RRA 27 // RRA
#define RRB 14 // RRB

//define for ultrasonic sensor
#define UE  4
#define UR  2
#define UL  19 
#define UB  5
#define UF  18

//define for IMU
#define SDA_PIN 21
#define SCL_PIN 22

#define MAX_DISTANCE 250 
#define SOUND_SPEED 0.034



SoftwareSerial MDDS60Serial(RXF, TXF);
SoftwareSerial MDDS60Serial1(RXR, TXR);


long durationF,durationB,durationR,durationL;
hw_timer_t * timer = NULL;
                  // Front Left   
volatile int cFL=0; // Current counter 
volatile int lFL=0; // Last counter 
volatile int dFL=0; // displacement counter 
                  // Front Right
volatile int cFR=0; // Current counter 
volatile int lFR=0; // Last counter
volatile int dFR=0; // displacement counter 
                  // Rear Left
volatile int cRL=0; // Current counter 
volatile int lRL=0;// Last counter
volatile int dRL=0; // displacement counter 
                  // Rear Right
volatile int cRR=0; // Current counter 
volatile int lRR=0;// Last counter
volatile int dRR=0; // displacement counter 
int Front = 0;
int Back = 0;
int Left = 0;
int Right = 0;

void jsonPrint() {
    responseDoc["FL_cFL"] = cFR;
    // responseDoc["FR_dFR"] = dFR;
    responseDoc["FR_cFR"] = cFL;
    // responseDoc["FL_dFL"] = dFL;
    responseDoc["RL_cRL"] = cRR;
    // responseDoc["RR_dRR"] = dRR;
    responseDoc["RR_cRR"] = cRL; 
    // responseDoc["RL_dRL"] = dRL;
    // for (int i = 0; i < 6; i++) {
    //   responseDoc["a" + String(i + 1)] = angles[i];
    // }
    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    Serial.println(jsonResponse);
}

void IRAM_ATTR onTimer(){ 
  dFL=(cFL-lFL)*10;
  lFL=cFL;
  dFR=(cFR-lFR)*10;
  lFR=cFR;
  dRL=(cRL-lRL)*10;
  lRL=cRL;
  dRR=(cRR-lRR)*10;
  lRR=cRR;
}

void IRAM_ATTR decoder_fl(){
  if(digitalRead(FLA) == digitalRead(FLB)) {
    cFL++; 
  } else {
    cFL--; 
  }
}
void IRAM_ATTR decoder_fr(){
  if(digitalRead(FRA) == digitalRead(FRB)) {
    cFR--; 
  } else {
    cFR++; 
  }
}

void IRAM_ATTR decoder_rl(){
  if(digitalRead(RLA) == digitalRead(RLB)) {
    cRL++; 
  } else {
    cRL--; 
  }
}
void IRAM_ATTR decoder_rr(){
  if(digitalRead(RRA) == digitalRead(RRB)) {
    cRR--; 
  } else {
    cRR++; 
  }
}

void setup() {

  pinMode(FLA,INPUT_PULLDOWN);
  pinMode(FLB,INPUT_PULLDOWN);
  pinMode(FRA,INPUT);
  pinMode(FRB,INPUT);
  pinMode(RLA,INPUT);
  pinMode(RLB,INPUT);
  pinMode(RRA,INPUT);
  pinMode(RRB,INPUT);
  // pinMode(UF,INPUT);
  // pinMode(UB,INPUT);
  // pinMode(UL,INPUT);
  // pinMode(UR,INPUT);
  
  // pinMode(TXR,OUTPUT);
  // pinMode(TXF,OUTPUT);
  // pinMode(UE,OUTPUT);
  

  attachInterrupt(FLA,decoder_fl,RISING);
  attachInterrupt(FRA,decoder_fr,RISING);
  attachInterrupt(RLA,decoder_rl,RISING);
  attachInterrupt(RRA,decoder_rr,RISING);


  // timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer 
  // timerAttachInterrupt(timer, &onTimer, true);
  //Set alarm to call onTimer function every second 1 tick is 1us
  // => 1 second is 1000000us 
  // Repeat the alarm (third parameter) 
  // timerAlarmWrite(timer, 100000, true);
  // timerAlarmEnable(timer); //enable 
  
  Serial.begin(115200);
  
  // MDDS60Serial.begin(9600); 
  // MDDS60Serial1.begin(9600);
  
  // Straightahead();

}

void loop() {
    // delay(5);
    /*
    digitalWrite(UE, LOW);
    delayMicroseconds(2);  
    digitalWrite(UE, HIGH);
    delayMicroseconds(10);
    digitalWrite(UE, LOW);
    durationF = pulseIn(UF, HIGH);
    Front = durationF * SOUND_SPEED/2;
    digitalWrite(UE, LOW);
    delayMicroseconds(2);  
    digitalWrite(UE, HIGH);
    delayMicroseconds(10);
    digitalWrite(UE, LOW);
    durationB = pulseIn(UB, HIGH);
    Back = durationB * SOUND_SPEED/2;
    digitalWrite(UE, LOW);
    delayMicroseconds(2);  
    digitalWrite(UE, HIGH);
    delayMicroseconds(10);
    digitalWrite(UE, LOW);
    durationR = pulseIn(UR, HIGH);
    Right = durationR * SOUND_SPEED/2;
    digitalWrite(UE, LOW);
    delayMicroseconds(2);  
    digitalWrite(UE, HIGH);
    delayMicroseconds(10);
    digitalWrite(UE, LOW);
    durationL = pulseIn(UL, HIGH);
    Left = durationL * SOUND_SPEED/2;
*/

 
// Serial.print("Front Left:[");
// Serial.print(cFL);
// Serial.print("][");
// Serial.print(dFL);
// Serial.print("], Front Right:[");
// Serial.print(cFR);
// Serial.print("][");
// Serial.print(dFR);
// Serial.print("], Rear Left:[");
// Serial.print(cRL);
// Serial.print("][");
// Serial.print(dRL);
// Serial.print("], Rear Right:[");
// Serial.print(cRR);
// Serial.print("][");
// Serial.print(dRR);
// Serial.print("]");
// Serial.print(", F: ");
// Serial.print(Front);
// Serial.print(", B: ");
// Serial.print(Back);
// Serial.print(", L: ");
// Serial.print(Left);
// Serial.print(", R:");
// Serial.println(Right);
jsonPrint();
delay(50);

}
