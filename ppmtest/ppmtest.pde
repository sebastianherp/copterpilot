
void setup()
{
    Serial.begin(57600);
    pinMode(2, INPUT);
    
}

unsigned long duration[8];
unsigned long duration2 = 0;

void loop()
{
    while(pulseIn(2, HIGH) < 4000) {}

    for(int i=0;i<8;i++) {
      duration[i] = pulseIn(2, HIGH);
      
    }
    for(int i=0;i<8;i++) {
      Serial.print(duration[i]);
      Serial.print(" ");
    }
    Serial.println("");
}
