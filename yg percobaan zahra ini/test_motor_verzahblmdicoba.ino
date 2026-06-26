// ============================================================
//  TEST MOTOR DC + L298N
//  Buka Serial Monitor 115200 baud
//  Ketik perintah:
//    f = maju
//    b = mundur
//    l = putar kiri
//    r = putar kanan
//    s = stop
//    + = speed naik 10
//    - = speed turun 10
// ============================================================

#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_IN3  27
#define MOTOR_IN4  14
#define MOTOR_ENA  32
#define MOTOR_ENB  33

int speed = 150; // mulai dari 150, range 0-255

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  stopMotor();
  Serial.println("=== TEST MOTOR ===");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'f': maju();    Serial.println("MAJU | speed=" + String(speed));   break;
      case 'b': mundur();  Serial.println("MUNDUR | speed=" + String(speed)); break;
      case 'l': kiri();    Serial.println("PUTAR KIRI");                      break;
      case 'r': kanan();   Serial.println("PUTAR KANAN");                     break;
      case 's': stopMotor(); Serial.println("STOP");                          break;
      case '+': speed = min(255, speed + 10);
                Serial.println("Speed=" + String(speed));                     break;
      case '-': speed = max(0, speed - 10);
                Serial.println("Speed=" + String(speed));                     break;
      case '?': printHelp();                                                   break;
    }
  }
}

void maju() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, speed); analogWrite(MOTOR_ENB, speed);
}

void mundur() {
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, speed); analogWrite(MOTOR_ENB, speed);
}

void kiri() {
  // Motor kiri mundur, kanan maju
  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, speed); analogWrite(MOTOR_ENB, speed);
}

void kanan() {
  // Motor kiri maju, kanan mundur
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, speed); analogWrite(MOTOR_ENB, speed);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);    analogWrite(MOTOR_ENB, 0);
}

void printHelp() {
  Serial.println("-------------------");
  Serial.println("f = maju");
  Serial.println("b = mundur");
  Serial.println("l = putar kiri");
  Serial.println("r = putar kanan");
  Serial.println("s = stop");
  Serial.println("+ = speed naik 10");
  Serial.println("- = speed turun 10");
  Serial.println("? = tampilkan ini");
  Serial.println("-------------------");
}
