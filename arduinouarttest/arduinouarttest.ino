#include <Servo.h>

#define enA 9
#define in1 5
#define in2 6

Servo myservo;

int pos = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  myservo.attach(3);

  Serial.begin(115200);

  while (!Serial) {
    ;
  }

}

const char terminator = '|';

void loop() {
  
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  const long interval = 500;

  if (Serial.available() > 0) {

    String jetmsg = Serial.readStringUntil(terminator);
    
    //print(jetmsg)

    String ackmsg = "Acknowledged.Command From Jetson: " + jetmsg;

    Serial.print(ackmsg);
    
//    print(ackmsg)

  }

  delay(5);



  // digitalWrite(enA, 100); // PWM control for speed

  // // Rotate motor in one direction
  // myservo.write( 0 );
  // delay( 500 );
  // myservo.write( 180 );
  // delay(500 );
  // myservo.write( 0 );
  // delay( 180 );
  // myservo.write( 180 );
  // delay( 180 );
}
