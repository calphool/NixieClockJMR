#include <PinChangeInt.h> // necessary otherwise we get undefined reference errors.
#include <AdaEncoder.h>

#define a_PINA 2
#define a_PINB 3
#define b_PINA A3
#define b_PINB A4

int8_t clicks=0;
char id=0;

void setup()
{
  Serial.begin(115200); Serial.println("---------------------------------------");
  AdaEncoder::addEncoder('a', a_PINA, a_PINB);
  AdaEncoder::addEncoder('b', b_PINA, b_PINB);  
}

void loop()
{
  encoder *thisEncoder;
  thisEncoder=AdaEncoder::genie(&clicks, &id);
  if (thisEncoder != NULL) {
    thisEncoder=AdaEncoder::getFirstEncoder();
    
    Serial.print(id); Serial.print(':');
    if (clicks > 0) {
      Serial.println(" CW");
    }
    if (clicks < 0) {
       Serial.println(" CCW");
    }
  }
}
/*
void loop()
{
  encoder *thisEncoder;
  thisEncoder=AdaEncoder::genie();
  if (thisEncoder != NULL) {
    Serial.print(thisEncoder->id); Serial.print(':');
    if (thisEncoder->clicks > 0) {
      Serial.println(" CW"); thisEncoder->clicks--;
    }
    if (thisEncoder->clicks < 0) {
       Serial.println(" CCW"); thisEncoder->clicks++;
    }
  }
}
*/
