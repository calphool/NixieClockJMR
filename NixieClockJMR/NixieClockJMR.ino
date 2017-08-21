/*******************************************************************************************************************************
 * 
 * Code for my custom designed/built Nixie clock, built around an Arduino UNO
 * 
 * 6 tube nixie clock (IN14) driven by 3 shift registers that fan out to six 74141 BCD to Nixie chips
 * Power supply looks like:  http://imgur.com/a/DifX8 (originally from:  http://www.ledsales.com.au/kits/nixie_supply.pdf)
 * Buck converter was added to the 9v line to derive 5v as well, like this one: http://imgur.com/a/aGJAv (originally https://images-na.ssl-images-amazon.com/images/I/61RQ2tjmbDL._SY355_.jpg)
 * This code also uses the ShiftPWM library to drive six RGB LEDs that were built around three shift registers
 * Finally, there are two rotary encoders and two pushbuttons that are used to manipulate the hours and minutes, to change between military time and normal time, and 
 * at the time of this writing to randomly change the colors of the LEDs.  At some point the behavior of the second button may change
 * to put the clock in an "alarm set" mode, but right now there's no piezo or speaker, so that's a future plan.
 * 
 ******************************************************************************************************************************* 
*/


#include <PinChangeInt.h> // used by AdaEncoder
#include <AdaEncoder.h>   // Enables rotary encoder support (this clock as two)
#include <Wire.h>         // used by RTCLib
#include "RTClib.h"       // Enables support for external clock board (DS3231 board like these: https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout/wiring-and-test)

// time mode constants
#define MILITARYTIME 0
#define NORMALTIME 1

#define a_PINA 3          // rotary encoder A pin
#define a_PINB 4          // rotary encoder A pin
#define b_PINA 5          // rotary encoder B pin
#define b_PINB 6          // rotary encoder B pin
int8_t clicks=0;          // counter used by rotary encoders
char id=0;                // rotary encoder identifier
encoder *thisEncoder;     // encoder object pointer


#define left_button_pin 7    // push button pin
#define right_button_pin 9   // push button pin
int b1;                      // button 1 current state
int b2;                      // button 2 current state
int b1LastState = HIGH;      // button 1 previous state


#define SHIFTPWM_NOSPI
const int ShiftPWM_latchPin=8;        // pins for LEDs controlled by ShiftPWM routines (hardware managed SPI has issues when used in this configuration, probably some interrupt thing)
const int ShiftPWM_dataPin = 11;
const int ShiftPWM_clockPin = 13;
const bool ShiftPWM_invertOutputs = false; 
const bool ShiftPWM_balanceLoad = false;
#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!
unsigned char maxBrightness = 64;     // number of divisions in RGB brightness
unsigned char pwmFrequency = 40;      // how often ShiftPWM is being invoked (too high = high CPU use, too low = flickering LEDs)
int numRegisters = 3;                 // number of shift registers running LEDs
byte ctr=0;     // used by LEDs
int ctrdir = 1; // used by LEDs


const int nixie_latch_pin=10;         // pins for Nixie tube shift registers
const int nixie_clock_pin=12;
const int nixie_data_pin=2;


// clock related variable definitions
byte s=11;      // seconds
byte m=57;      // minutes
byte h=9;       // hours
byte olds=255;  // second comparator (since loop runs continuously, we need to know when to change the Nixie display -- when this number and the current s disagree)
byte canRunTubeSweep; // determines whether 111111,222222,333333... sweep can happen
byte hourMode = NORMALTIME;


RTC_DS3231 rtc;  // variable for rtc interface
DateTime now;    // variable to hold results of rtc call

void setup(){
  // set pin modes
  pinMode (nixie_latch_pin, OUTPUT);  
  pinMode (nixie_data_pin, OUTPUT);
  pinMode (nixie_clock_pin, OUTPUT);
  pinMode (left_button_pin, INPUT);
  pinMode (right_button_pin, INPUT);

  // set initial value for buttons
  digitalWrite(left_button_pin, HIGH);
  digitalWrite(right_button_pin, HIGH);

  // turn on serial communication
  Serial.begin(9600);

  // create two encoder objects
  AdaEncoder::addEncoder('a', a_PINA, a_PINB);
  AdaEncoder::addEncoder('b', b_PINA, b_PINB); 

  // wait for clock to come online
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
    delay(100);
  }

  // if the clock has lost power, set its date/time to the date/time of this code's compilation
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  
  ShiftPWM.SetAmountOfRegisters(numRegisters); // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetPinGrouping(1);                  // stuff related to how wiring of LEDs was done
  ShiftPWM.Start(pwmFrequency,maxBrightness);  // starts up timer interrupt for LED ShiftPWM
  ShiftPWM.SetAll(0);                          // turn all the LEDs off 
}




void handleButtons() {
  b1 = digitalRead(left_button_pin);              // get current button state
  if(b1 == LOW && b1LastState == HIGH) {          // it's depressed (it's LOW), and it was previously HIGH (meaning we just switched to depressed)
    if(hourMode == NORMALTIME)                    // if we're in NORMALTIME
        hourMode = MILITARYTIME;                  //     switch to MILITARYTIME
    else                                          // otherwise
        if(hourMode == MILITARYTIME)              // if we're in MILITARYTIME
            hourMode = NORMALTIME;                //     switch to NORMALTIME
    b1LastState = LOW;                            // set lastState to LOW so we don't keep toggling hourMode back and forth
  }
  if(b1 == HIGH)                                  // if the button is not depressed, then set its last state to HIGH
      b1LastState = HIGH;

  b2 = digitalRead(right_button_pin);             // get current button state
  if(b2 == LOW) {                                 // it's depressed
      ctr = random(360);                          // set the LED color to some random location in the rainbow
  }
}

void updateLEDs() {
   ctr = ctr + ctrdir;            // move the rainbow forward or backward in its current direction
   if(ctr >= 359)                 // if we hit the end of the rainbow on the right, change directions
       ctrdir = -ctrdir;
   if(ctr <= 1)                   // if we hit the end of the rainbow on the left, change directions
       ctrdir = -ctrdir;
       
   for(int led=0;led<6;led++){            // loop over all LED's
    int hue = ((led)*360/(63)+ctr)%360;   // Set hue from 0 to 360 from first to last led and shift the hue
    ShiftPWM.SetHSV(led, hue, 255, 255);  // write the HSV values, with saturation and value at maximum
   }
}


// knob handler code
void handleKnobTurn() {
  thisEncoder = AdaEncoder::genie(&clicks, &id);    // get reference to AdaEncoder object 
  if (thisEncoder != NULL) {                        // we've got one that wants to tell us something
    if(id == 'b') {                                 // if it's encoder 'b'
      if(clicks > 0)                                // clockwise
          rtc.adjust(rtc.now().unixtime() + 3600);  // forward an hour
      if(clicks < 0)                                // counterclockwise
          rtc.adjust(rtc.now().unixtime() - 3600);  // backward an hour
    }
    else {
      if(clicks > 0)                                // same for encoder 'b'
          rtc.adjust(rtc.now().unixtime() + 60);
      if(clicks < 0) 
          rtc.adjust(rtc.now().unixtime() - 60);
    }
    syncClockVariables();                           // since we just changed the clock, now sync the variables 
    olds=s+1;                                       // force a display update by making olds != s
  }
}


// pulls data from the DS3231 clock chip and puts it in variables
void syncClockVariables() {
    now = rtc.now();
    s = now.second();
    m = now.minute();
    h = now.hour();
    if(hourMode == NORMALTIME) {    // jimmy with the hour if we're in "normal" mode
      if(h>12) h=h-12;
      if(h==0) h = 12;
    }
}


// updates the display if "olds != s"
void updateDisplay() {
  if(olds != s) {
    digitalWrite(nixie_latch_pin, LOW);
    shiftOut(nixie_data_pin, nixie_clock_pin, MSBFIRST, flipValue(s));
    shiftOut(nixie_data_pin, nixie_clock_pin, MSBFIRST, flipValue(m));
    shiftOut(nixie_data_pin, nixie_clock_pin, MSBFIRST, flipValue(h));
    digitalWrite(nixie_latch_pin, HIGH);
    olds = s;
  }
}


// this code deals with silly wiring decisions I made in my hardware
byte flipValue(byte in) {         // in = 59                              
  byte a,b,c;
  
  a = in / 10;   // a = 5  
  b = in % 10;   // b = 9                                
                 // in is now split into tens and ones
  a = 10 - a;    // a = 5                                
  b = 10 - b;    // b = 1
  if(a==10) a = 0;
  if(b==10) b = 0;                                
  c = (b << 4) | a;     // c = 00010101 = 1+4+16 = 21
  return c;
}


// most of the time this delay just waits 100ms.  
// however, if the seconds value is zero, and a random number between 0 and 100 > 80, and we didn't just sweep through
// the display, then it'll do a "sweep" of values through the HH, MM, and SS Nixie tubes.  It helps with burn in.
void doDelay() {
  if(s == 0 && random(100) > 80 && canRunTubeSweep == HIGH) {
    for(byte i=0;i<=99;i=i+11) {  // this code sweeps all the tubes with values of 11,22,33,44, etc. to keep from burning out elements... happens randomly on the 0 second
      digitalWrite(nixie_latch_pin, LOW);
      shiftOut(nixie_data_pin, nixie_clock_pin, MSBFIRST, flipValue(i));
      shiftOut(nixie_data_pin, nixie_clock_pin, MSBFIRST, flipValue(i));
      shiftOut(nixie_data_pin, nixie_clock_pin, MSBFIRST, flipValue(i));
      digitalWrite(nixie_latch_pin, HIGH);
      delay(75);
      canRunTubeSweep = LOW;
    }
  }
  else 
    delay(100);  // this is what normally happens, we sleep for 100ms

  if(s > 0)           // if we're not sitting on zero, then we need to reset the "canRunTubeSweep" variable back to HIGH
      canRunTubeSweep = HIGH;
}



// Main loop
void loop()
{ 
  syncClockVariables(); // pull data from DS3231 board into s,m,h values
  handleButtons();      // deal with any buttons being pressed
  handleKnobTurn();     // deal with any knobs having been turned
  updateDisplay();      // update the display
  updateLEDs();         // tweak the LED colors
  doDelay();            // wait a short time so we're not burning CPU for no reason
}

