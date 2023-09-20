#include <Servo.h>   // 內建的，不用安裝

Servo shootGate;  // 建立servo物件
Servo shootRod;
Servo supplyRod;
char signal;

void setup() {
  Serial.begin(115200); // Set Baud rate
  shootGate.attach(6);  // PIN for the launching gate
  shootRod.attach(7);   // PIN for the launching rod
  supplyRod.attach(8);  // PIN for the supplementary rod
  shootGate.write(10);
  shootRod.write(10);
  supplyRod.write(10);
}

void loop() {
  if(Serial.available()>0){
    signal = Serial.read();
    if(signal == '6'){
    shootGate.write(150); 
    delay(1000);
    shootGate.write(10);
    delay(1000);
    }else if(signal == '7'){
      shootRod.write(150);
      delay(1500);
      shootRod.write(10);
      delay(1000);
    }else if(signal == '8'){
      supplyRod.write(150);
      delay(1000);
      supplyRod.write(10);
      delay(1000);
    }else if(signal == '0'){
      shootGate.write(10);
      shootRod.write(10);
      supplyRod.write(10);
      delay(1000);
    }
  }
}