#include <Servo.h>

#define enA 9
#define in1 6
#define in2 7

Servo myservo;

int pos = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // myservo.attach(5,530,2600); //BETTER TO USE SERVO ON PINS 9 AND 10 ACCORDING TO DOCUMENTATION
  myservo.attach(5); //BETTER TO USE SERVO ON PINS 9 AND 10 ACCORDING TO DOCUMENTATION
}

void loop() {
  // Rotate motor in one direction
  digitalWrite(enA, 100); // PWM control for speed
  delay(5000);
  digitalWrite(enA, 0); // PWM control for speed
  delay(5000);
  myservo.write(0);
  delay(1000);
  myservo.write(180);
  delay(1000);
}
