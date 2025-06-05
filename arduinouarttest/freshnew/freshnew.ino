#include <Servo.h>

#define enA 9
#define in1 5
#define in2 6

Servo myservo;

int pos = 90;

void setup() {
  //Serial.begin(115200);
  
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  

  myservo.attach(3);
  
}

void loop() {
  digitalWrite(enA, 100); // PWM control for speed 
  
//  if (Serial.available() > 0) {
//    int inChar = Serial.read();
//    char inp = (char)inChar;
//    //Serial.println(inp);
//    
//    
//    int val = inp - '0';
//    
//    if (val == 1) {
//      Serial.println('s');
//      digitalWrite(enA, 100); // PWM control for speed 
//      if (pos==180){
//        for (int i=180; i>=90; i-=10){
//          myservo.write(i);
//          delay(50);
//        }
//        delay(50);
//      }
//      pos=90;
//    }
//    else if (val == 2) {
//      Serial.println('c');
//      digitalWrite(enA, 0);
//      if (pos==90){
//        for (int i=90; i<=180; i+=10){
//          myservo.write(i);
//          delay(50);
//        }
//      }
//      pos=180;
//    }
//    else {
//      Serial.println('e');
//      digitalWrite(enA, 0);
//      if (pos==180){
//        for (int i=180; i>=90; i-=10){
//          myservo.write(i);
//          delay(50);
//        }
//      }
//      pos=90;
//      
//    }
//  }
}

