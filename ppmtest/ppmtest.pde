
void setup()
{
    Serial.begin(57600);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    pinMode(5, INPUT);
    
}

unsigned long duration = 0;

void loop()
{
    duration = pulseIn(5, HIGH);
    Serial.println(duration);
}
