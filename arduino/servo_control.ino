#include <Servo.h>   // 內建的，不用安裝

Servo shootGate;  // 建立servo物件
Servo shootRod;
//Servo supplyRod;

int signal;
char _;
int gate = 0;

const int SHUTDOWN = 0;
const int FIRE = 6;
const int OPENGATE = 7;

void setup() {
  Serial.begin(115200); // Set Baud rate
  shootGate.attach(10);  // PIN for the launching gate
  shootRod.attach(11);   // PIN for the launching rod
//  supplyRod.attach(8);  // PIN for the supplementary rod
  shootGate.write(35);
  delay(200);
  shootRod.write(180);
//  delay(200);
//  supplyRod.write(0);
}

void loop() {
  if (Serial.available()) {
    signal = Serial.read();
    signal -= 48;
    _ = Serial.read();
    if (signal == OPENGATE) {
      delay(100);
      if (gate == 0) {
        shootGate.write(0);
        delay(200);
        gate = 1;

      }
      else if (gate == 1) {
        shootGate.write(35);
        delay(200);
        gate = 0;
      }
    } else if (signal == FIRE) {
      delay(100);
      shootRod.write(80);
      delay(800);
      shootRod.write(180);
      delay(1300);
      
    } else if (signal == SHUTDOWN) {
      shootGate.write(35);
      delay(200);
      shootRod.write(80);
      delay(800);
    }
  }
}