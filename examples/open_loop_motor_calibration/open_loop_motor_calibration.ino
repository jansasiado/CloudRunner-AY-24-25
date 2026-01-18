
#define L_SPEED_PIN 10 //D10
#define R_SPEED_PIN 9 //D9
#define L_FORWARD A1  //A0
#define L_BACKWARD A0 //A1
#define R_FORWARD A2 //A2
#define R_BACKWARD A3 //A3                                                                      

void setup() {
  //Setting up pinmodes
  pinMode(L_SPEED_PIN,OUTPUT);
  pinMode(R_SPEED_PIN,OUTPUT);
  pinMode(L_FORWARD,OUTPUT);
  pinMode(L_BACKWARD,OUTPUT);
  pinMode(R_FORWARD,OUTPUT);
  pinMode(R_BACKWARD,OUTPUT);
  //delay(3000);
}

void loop() {
  //Run pins motor forward
  digitalWrite(L_FORWARD,HIGH);
  digitalWrite(L_BACKWARD,LOW);

  digitalWrite(R_FORWARD,HIGH);
  digitalWrite(R_BACKWARD,LOW);
  
  
  // analogWrite(L_SPEED_PIN,255);
  // analogWrite(R_SPEED_PIN,255);
  // delay(500);
  analogWrite(L_SPEED_PIN,0);
  analogWrite(R_SPEED_PIN,0);
  delay(5000);
  analogWrite(L_SPEED_PIN,50);
  analogWrite(R_SPEED_PIN,0);
  delay(500);
  analogWrite(L_SPEED_PIN,0);
  analogWrite(R_SPEED_PIN,50);
  delay(500);
  // analogWrite(L_SPEED_PIN,31);
  // analogWrite(R_SPEED_PIN,28);
  // delay(1000);
  // analogWrite(L_SPEED_PIN,63);
  // analogWrite(R_SPEED_PIN,56);
  // delay(1000);
  analogWrite(L_SPEED_PIN,127);
  analogWrite(R_SPEED_PIN,127);
  delay(500);
  analogWrite(L_SPEED_PIN,191);
  analogWrite(R_SPEED_PIN,191);
  delay(500);
  analogWrite(L_SPEED_PIN,255);
  analogWrite(R_SPEED_PIN,255);
  delay(500);
  //After running the code, if you motor operates in reverse, swap its pinx
  //If all the pins are now correct, kindly change the same pins in the constants header file (constants.h)
}

