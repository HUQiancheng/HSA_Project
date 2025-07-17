const int ENA = 5, IN1 = 6, IN2 = 7;
const int ENB = 9, IN3 = 10, IN4 = 11;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setMotor(int pwmA, int dirA, int pwmB, int dirB) {
  analogWrite(ENA, pwmA);
  digitalWrite(IN1, dirA == 1);
  digitalWrite(IN2, dirA == 0);

  analogWrite(ENB, pwmB);
  digitalWrite(IN3, dirB == 1);
  digitalWrite(IN4, dirB == 0);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // 形如 "100,1,100,0\n"
    input.trim();
    int pwmA, dirA, pwmB, dirB;
    if (sscanf(input.c_str(), "%d,%d,%d,%d", &pwmA, &dirA, &pwmB, &dirB) == 4) {
      setMotor(pwmA, dirA, pwmB, dirB);
    }
  }
}
