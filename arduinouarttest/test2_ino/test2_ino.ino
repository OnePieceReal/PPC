void setup() {
    Serial.begin(9600);  // Initialize serial communication at 9600 bps
}

void loop() {
    if (Serial.available() > 0) {
        String received = Serial.readString();  // Read the incoming data
        Serial.print("Received: ");
        Serial.println(received);  // Echo the received data back
    }
}

