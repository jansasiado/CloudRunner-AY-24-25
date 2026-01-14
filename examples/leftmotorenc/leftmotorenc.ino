// =======================
// Encoder pins (LEFT MOTOR)
// =======================
#define ENC_A A4   // PCINT12
#define ENC_B A5   // PCINT13

volatile long count = 0;
volatile uint8_t lastState = 0;

// =======================
// Motor pins
// =======================
#define lspeed 10
#define lforward A0 
#define lbackward A1

const int CPR = 2763;  // measured counts per output shaft revolution

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(lspeed, OUTPUT);
  pinMode(lforward, OUTPUT);
  pinMode(lbackward, OUTPUT);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  lastState = (PINC >> 4) & 0x03;

  // Enable PCINT for A4/A5
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT12);
  PCMSK1 |= (1 << PCINT13);

  Serial.println("=== Motor Angle Test ===");
  delay(3000);
}

// =======================
// PCINT ISR
// =======================
ISR(PCINT1_vect) {
  uint8_t state = (PINC >> 4) & 0x03;

  if ((lastState == 0b00 && state == 0b01) ||
      (lastState == 0b01 && state == 0b11) ||
      (lastState == 0b11 && state == 0b10) ||
      (lastState == 0b10 && state == 0b00)) {
    count++;
  } 
  else if (
      (lastState == 0b00 && state == 0b10) ||
      (lastState == 0b10 && state == 0b11) ||
      (lastState == 0b11 && state == 0b01) ||
      (lastState == 0b01 && state == 0b00)) {
    count--;
  }

  lastState = state;
}

// =======================
// Function to turn a specific angle
// =======================
void turnDegrees(int angle, int speed) {
  long targetCounts = (long)((CPR / 360.0) * abs(angle));
  count = 0;  // reset encoder

  if (angle > 0) {  // forward
    digitalWrite(lforward, HIGH);
    digitalWrite(lbackward, LOW);
  } else {          // backward
    digitalWrite(lforward, LOW);
    digitalWrite(lbackward, HIGH);
  }

  analogWrite(lspeed, speed);

  // Wait until we reach target
  while (abs(count) < targetCounts) {
    // Optionally print RPM while moving
    static unsigned long lastPrint = 0;
    static long lastCount = 0;

    if (millis() - lastPrint >= 200) {  // update every 0.2 sec
      long delta = count - lastCount;
      float rpm = (float)delta / CPR * 60.0 * 5; // scale 0.2 sec → 1 sec
      Serial.print("Counts: ");
      Serial.print(count);
      Serial.print(" | RPM: ");
      Serial.println(rpm);
      lastCount = count;
      lastPrint = millis();
    }
  }

  // Stop motor
  digitalWrite(lforward, LOW);
  digitalWrite(lbackward, LOW);
  analogWrite(lspeed, 0);
  Serial.print("Target angle ");
  Serial.print(angle);
  Serial.println("° reached!");
}

void loop() {
  //delay(1000);  // wait 1 second before starting

  // Turn 90 degrees forward
  turnDegrees(360, 150);
  delay(2000);

  // Turn 180 degrees backward
  turnDegrees(-360, 150);
  delay(2000);


}
