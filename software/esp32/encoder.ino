
#define ppr 330

volatile int32_t temp, counter;

#define maxTurn 10
float turn;

void setup() {
  Serial.begin (9600);
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3

  attachInterrupt(0, EncoderA, RISING);
  attachInterrupt(1, EncoderB, RISING);
}
   
void loop() {
  
  if( counter != temp ){
    Serial.print("Encoder count : ");
    Serial.print(counter);
    Serial.print("     ");
    
    turn = (float)counter/LPD3806;

    Serial.print(turn,3); //นับรอบที่ encoder
    Serial.println(" turn");
    temp = counter;
  }
}
   
void EncoderA(){
  if(digitalRead(3)==LOW) counter++;
  else counter--;
}

void EncoderB(){
  if(digitalRead(2)==LOW) counter--;
  else counter++;
}