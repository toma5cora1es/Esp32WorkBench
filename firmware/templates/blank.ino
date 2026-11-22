void setup() {
    Serial.begin(115200);
    pinMode(2, OUTPUT);
}

void loop() {
    digitalWrite(2, !digitalRead(2));
    delay(500);
}