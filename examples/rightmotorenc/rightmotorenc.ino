// =======================
// Encoder pins (RIGHT MOTOR)
// =======================
#define ENC_A 3   // D3
#define ENC_B 6   // D6

volatile long count = 0;
volatile uint8_t lastState = 0;

// =======================
// Motor pins (RIGHT MOTOR)
// =======================
#define rspeed 9
#define rforward A2
#define rbackward A3

const int CPR = 2763;  // measured counts per output shaft revolution

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(rspeed, OUTPUT);
  pinMode(rforward, OUTPUT);
  pinMode(rbackward, OUTPUT);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Initial quadrature state
  lastState = ((PIND >> 3) & 0x01) | (((PIND >> 6) & 0x01) << 1);

  // Enable PCINT for D3/D6 (PCINT19 and PCINT22)
  PCICR |= (1 << PCIE2);       // Enable PCINT2
  PCMSK2 |= (1 << PCINT19);    // D3
  PCMSK2 |= (1 << PCINT22);    // D6

  Serial.println("=== Motor Angle Test (D3/D6) ===");
  delay(3000);
}

// =======================
// PCINT2 ISR
// =======================
ISR(PCINT2_vect) {
  uint8_t state = ((PIND >> 3) & 0x01) | (((PIND >> 6) & 0x01) << 1);

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
    digitalWrite(rforward, HIGH);
    digitalWrite(rbackward, LOW);
  } else {          // backward
    digitalWrite(rforward, LOW);
    digitalWrite(rbackward, HIGH);
  }

  analogWrite(rspeed, speed);

  // Wait until we reach target
  while (abs(count) < targetCounts) {
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
  digitalWrite(rforward, LOW);
  digitalWrite(rbackward, LOW);
  analogWrite(rspeed, 0);
  Serial.print("Target angle ");
  Serial.print(angle);
  Serial.println("° reached!");
}

void loop() {
  // Turn 360 degrees forward
  turnDegrees(360, 150);
  delay(2000);

  // Turn 360 degrees backward
  turnDegrees(-360, 150);
  delay(2000);
}
