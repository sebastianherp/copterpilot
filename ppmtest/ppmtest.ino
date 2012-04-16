
void setup()
{
    Serial.begin(57600);
    pinMode(2, INPUT);
    attachInterrupt(0, flanke, CHANGE);
}


char ppm_on = 0;
char counter = 0;
unsigned long start;
unsigned long duration = 0;
volatile unsigned long values[8];

unsigned long duration2 = 0;
unsigned long timer = 0;

void loop()
{
    delay(20);
    for(int i=0;i<8;i++) {
      //values[i] = pulseIn(2, LOW);
      Serial.print(values[i]);
      Serial.print(" ");
    }
    Serial.println("");
}

void flanke() {
   if(!digitalRead(2)) {
     start = micros();
   } else {
     duration = micros() - start;
     if(duration > 4000)
       counter = 0;
     else {
       //if(counter >= 4)
       //  values[counter]  = (values[counter] * 9 + duration) / 10;
       //else
         values[counter]  = duration;
       counter++;
       if(counter >= 8)
         counter = 0;
     }
   }
}
