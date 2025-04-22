const int RELAY_PIN = 2; // G2

void setup()
{
    pinMode(RELAY_PIN, OUTPUT);
}

void loop()
{
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);
    digitalWrite(RELAY_PIN, LOW);
    delay(500);
}
