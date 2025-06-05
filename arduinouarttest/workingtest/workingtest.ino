#include <Servo.h>

#define enA 9
#define in1 5
#define in2 6

Servo myservo;

int pos = 86;
int lowLim = 86;
int highLim = 130;

void setup() {
  Serial.begin(115200);
  
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  

  myservo.attach(3);
  myservo.write(lowLim);
  //digitalWrite(enA, 100); // PWM control for speed 
  printf("Hello testing change");
  
}

void loop() {
  if (Serial.available() > 0) {
    int inChar = Serial.read();
    char inp = (char)inChar;
    //Serial.println(inp);
    
    
    int val = inp - '0';
    
    if (val == 1) {
      Serial.println('s');
      digitalWrite(enA, 10); // PWM control for speed 
      if (pos==highLim){
        for (int i=highLim; i>=lowLim; i-=10){
          myservo.write(i);
          delay(50);
        }
        myservo.write(lowLim);
        delay(50);
      }
      pos=lowLim;
    }
    else if (val == 2) {
      Serial.println('c');
      digitalWrite(enA, 0);
      if (pos==lowLim){
        for (int i=lowLim; i<=highLim; i+=10){
          myservo.write(i);
          delay(50);
        }
        myservo.write(highLim);
      }
      pos=highLim;
    }
    else {
      Serial.println('e');
      digitalWrite(enA, 0);
      if (pos==highLim){
        for (int i=highLim; i>=lowLim; i-=10){
          myservo.write(i);
          delay(50);
        }
        myservo.write(lowLim);
      }
      pos=lowLim;
      
    }
  }
}

