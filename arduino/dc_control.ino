const int SHUTDOWN = 0;
const int FORWARD = 1;
const int BACKWARD = 2;
const int SHIFT_LEFT = 3;
const int SHIFT_RIGHT = 4;
const int BRAKE = 5;
const int TURN_LEFT = 8;
const int TURN_RIGHT = 9;

// PIN
const int SLP = 7;
const int A_PWM = 11; // Left front wheel
const int A_DIR = 12;
const int B_PWM = 9;  // Right front wheel
const int B_DIR = 8;
const int C_PWM = 5;  // Left rear wheel
const int C_DIR = 6;
const int D_PWM = 2;  // Right rear wheel
const int D_DIR = 3;

int signal;
char _;

int speed = 0;
int max_speed = 200;
int A_speed = 0;
int B_speed = 0;
int C_speed = 0;
int D_speed =  0;
int Amax_speed = 225;
int Bmax_speed = 160;
int Cmax_speed = 225;
int Dmax_speed = 160;

void setup() {
   Serial.begin(115200); // Set Baud rate
   pinMode(SLP, OUTPUT);
   pinMode(A_PWM, OUTPUT);
   pinMode(A_DIR, OUTPUT);
   pinMode(B_PWM, OUTPUT);
   pinMode(B_DIR, OUTPUT);
   pinMode(C_PWM, OUTPUT);
   pinMode(C_DIR, OUTPUT);
   pinMode(D_PWM, OUTPUT);
   pinMode(D_DIR, OUTPUT);
}

void loop() {
  digitalWrite(SLP, HIGH); // Set HIGH to enable the motor driver
  
  if (Serial.available()) { // Receive signal
    signal = Serial.read();
    signal -=48;
    _ = Serial.read();
    Serial.println(signal);
  }

  // Command handling
  switch (signal) {
    case SHUTDOWN:
      speed = 0;
      ABCD_PWM(speed, speed, speed, speed);
      break;

    case FORWARD:     // Forward
      ABCD_DIR(HIGH, HIGH, HIGH, HIGH);
      A_speed = Amax_speed;
      B_speed = Bmax_speed;
      C_speed = Cmax_speed;
      D_speed = Dmax_speed;
      ABCD_PWM(A_speed, B_speed, C_speed, D_speed);
      break;

    case BACKWARD:      // Backward
      A_speed = Amax_speed;
      B_speed = Bmax_speed;
      C_speed = Cmax_speed;
      D_speed = Dmax_speed;
      ABCD_DIR(LOW, LOW, LOW, LOW);
      speed = max_speed;
      ABCD_PWM(A_speed, B_speed, C_speed, D_speed);
      break;

    case SHIFT_LEFT:      // Shift left
      A_speed = 150;
      B_speed = 170;
      C_speed = 150;
      D_speed = 150;
      ABCD_DIR(HIGH, LOW, LOW, HIGH);
      speed = max_speed;
      ABCD_PWM(A_speed, B_speed, C_speed, D_speed);
      break;

    case SHIFT_RIGHT:     // Shift right
      ABCD_DIR(LOW, HIGH, HIGH, LOW);
      A_speed = 150;
      B_speed = 170;
      C_speed = 150;
      D_speed = 150;
      ABCD_PWM(A_speed, B_speed, C_speed, D_speed);
      break;

    case TURN_LEFT:     // Turn left
      ABCD_DIR(LOW, HIGH, LOW, HIGH);
      A_speed = 200;
      B_speed = 200;
      C_speed = 200;
      D_speed = 200;
      ABCD_PWM(A_speed, B_speed, C_speed, D_speed);
      break;

    case TURN_RIGHT:     // Turn right
      ABCD_DIR(HIGH, LOW, HIGH, LOW);
      A_speed = 200;
      B_speed = 200;
      C_speed = 200;
      D_speed = 200;
      ABCD_PWM(A_speed, B_speed, C_speed, D_speed);
      break;

    case BRAKE:     // Brake
      speed = 0;
      ABCD_PWM(speed, speed, speed, speed);
      break;
  }
}

void ABCD_PWM(int A_speed, int B_speed, int C_speed, int D_speed) {
  analogWrite(A_PWM, A_speed);
  analogWrite(B_PWM, B_speed);
  analogWrite(C_PWM, C_speed);
  analogWrite(D_PWM, D_speed);
}

void ABCD_DIR(int A_HL, int B_HL, int C_HL, int D_HL) {
  digitalWrite(A_DIR, A_HL);
  digitalWrite(B_DIR, B_HL);
  digitalWrite(C_DIR, C_HL);
  digitalWrite(D_DIR, D_HL);
}