const char SHUTDOWN = '0';
const char FORWARD = '1';
const char BACKWARD = '2';
const char SHIFT_LEFT = '3';
const char SHIFT_RIGHT = '4';
const char BRAKE = '5';

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

char signal;
char prevSig; // Previous signal
char prevChangeSig; // Previous change signal

int speed = 0;
int max_speed = 150;

int acc = 30; // Acceleration
int brake = 0;

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
  
  if (Serial.available() > 0) { // Receive signal
    signal = Serial.read();
  }

  // Speed down if the signal is changed
  if ((signal != BRAKE) && (signal != SHUTDOWN) && (prevSig != signal) && (prevSig != BRAKE) ){   
    for (; speed > 0; speed -= acc) {
      if (Serial.available() > 0) {   // Received signal while decelerating
        signal = Serial.read();
        if ((signal != BRAKE) && (signal != SHUTDOWN) && (prevChangeSig == signal)) { // If the signal and previous change signal are the same, stop decelerating
          break;
        }
      }
      ABCD_PWM(speed);
      delay(100);
    }
    if (prevChangeSig != signal) {
      speed = 0;
      ABCD_PWM(speed);
      delay(100);
    }
    prevSig = signal; 
  }



  // Command handling
  switch (signal) {
    case SHUTDOWN:
      speed = 0;
      ABCD_PWM(speed);
      prevSig = signal;
      break;

    case FORWARD:     // Forward
      ABCD_DIR(HIGH, HIGH, HIGH, HIGH);
      prevSig = signal;
      prevChangeSig = signal;
      for (; speed <= max_speed; speed += acc) {   // Slowly speed up
        if (speed > max_speed) {
          speed = max_speed;
        }
        brake = brake_judge();
        if (brake == 1) {
          break;
        }
        ABCD_PWM(speed);
        delay(200);
      }
      break;

    case BACKWARD:      // Backward
      ABCD_DIR(LOW, LOW, LOW, LOW);
      prevSig = signal;
      prevChangeSig = signal;
      for (; speed <= max_speed; speed += acc) {
        if (speed > max_speed) {
          speed = max_speed;
        }
        brake = brake_judge();
        if (brake == 1) {
          break;
        }
        ABCD_PWM(speed);
        delay(200);
      }
      break;

    case SHIFT_LEFT:      // Shift left
      ABCD_DIR(HIGH, LOW, LOW, HIGH);
      prevSig = signal;
      prevChangeSig = signal;
      for (; speed <= max_speed; speed += acc) {
        if (speed > max_speed) {
          speed = max_speed;
        }
        brake = brake_judge();
        if (brake == 1) {
          break;
        }
        ABCD_PWM(speed);
        delay(200);
      }
      break;

    case SHIFT_RIGHT:     // Shift right
      ABCD_DIR(LOW, HIGH, HIGH, LOW);
      prevSig = signal;
      prevChangeSig = signal;
      for (; speed <= max_speed; speed += acc) {
        if (speed > max_speed) {
          speed = max_speed;
        }
        brake = brake_judge();
        if (brake == 1) {
          break;
        }
        ABCD_PWM(speed);
        delay(200);
      }
      break;

    case BRAKE:     // Brake
      for (; speed > 0; speed -= acc) {
        if (Serial.available() > 0) {
          signal = Serial.read();
          if ((signal != BRAKE) && (signal != SHUTDOWN) && (prevChangeSig == signal)) {
              break;
          }
        }
        ABCD_PWM(speed);
        delay(100);
      }
      if (prevChangeSig != signal) {
        speed = 0;
        ABCD_PWM(speed);
        delay(100);
      }
      prevSig = signal;
      break;
  }
}

int brake_judge() {
  int brake = 0;
  if (Serial.available() > 0) {
    signal = Serial.read();
    if ((prevSig != signal) || (prevChangeSig == signal)) {
      brake = 1;
    } else {
      brake = 0;
    }
  }
  return brake;
}

void ABCD_PWM(int speed) {
  analogWrite(A_PWM, speed);
  analogWrite(B_PWM, speed);
  analogWrite(C_PWM, speed);
  analogWrite(D_PWM, speed);
}

void ABCD_DIR(int A_HL, int B_HL, int C_HL, int D_HL) {
  digitalWrite(A_DIR, A_HL);
  digitalWrite(B_DIR, B_HL);
  digitalWrite(C_DIR, C_HL);
  digitalWrite(D_DIR, D_HL);
}
