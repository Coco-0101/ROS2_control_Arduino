#include <Servo.h>   //內建的，不用安裝

Servo myservo;  // 建立myservo物件
char receiverChar;

void setup() {
  Serial.begin(115200); //Baud rate
  pinMode(LED_BUILTIN, OUTPUT);
  myservo.attach(5);  // PIN
  myservo.write(30); // 歸零
}

void loop() {
  if(Serial.available()>0){
    receiverChar = Serial.read();
  }
  if(receiverChar == '0'){
      myservo.write(30);  //旋轉到0度
      delay(1000);
  }else if(receiverChar == '1'){
      myservo.write(90);
      delay(1000);
  }else if(receiverChar == '2'){
        myservo.write(150);
        delay(1000);
  }
}