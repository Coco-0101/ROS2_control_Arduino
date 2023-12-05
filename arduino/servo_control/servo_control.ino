#include <Servo.h>   // 內建的，不用安裝

Servo shootGate;  // 建立servo物件
Servo shootRod;
Servo supplyRod;
char signal;

int gate_init = 35;
int gate_act = 20;
int shoot_init = 80;
int shoot_act = 180;
int supply_init = 50;
int supply_act = 150;

void setup() {
  Serial.begin(115200); // Set Baud rate
  shootGate.attach(8);  // PIN for the launching gate
  shootRod.attach(9);   // PIN for the launching rod
  supplyRod.attach(10);  // PIN for the supplementary rod
  shootGate.write(35);
  shootRod.write(80);
  supplyRod.write(0);
}

void loop() {
  if(Serial.available()>0){
    signal = Serial.read();
    if(signal == '6'){
    shootGate.write(0); 
    delay(1000);
    shootGate.write(35);
    delay(1000);
    }else if(signal == '7'){
      shootGate.write(0); 
      delay(200);
      shootRod.write(80);
      delay(800);
      shootRod.write(180);
      delay(800);
      shootGate.write(35);
      delay(200);
    }else if(signal == '8'){
      shootGate.write(0);
      shootRod.write(80);
      supplyRod.write(80);

      delay(800);
      supplyRod.write(0);
      delay(1000);
      shootRod.write(180);
      delay(800);
      shootGate.write(35);
    }else if(signal == '0'){
      shootGate.write(10);
      shootRod.write(10);
      supplyRod.write(10);
      delay(1000);
    }
  }

}
