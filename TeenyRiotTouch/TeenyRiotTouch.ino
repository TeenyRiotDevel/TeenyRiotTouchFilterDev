#define USE_MIDI

#include <TeenyMidi.h>

#include "TeenyTouchDusjagr.h"
#include "AnalogTouch.h"
#include "SampleFilter.h"

MIDIMessage midimsg;

//TeenyTouchDusjagr test2;
// ATMEL ATTINY85
//
//                   +-\/-+
//             PB5  1|    |8  VCC
//ref     ADC3/PB3  2|    |7  PB2/ADC1
//capsens ADC2/PB4  3|    |6  PB1*
//             GND  4|    |5  PB0*
//                   +----+
//
// * indicates PWM port
//

int i = -1023;
int value = 0;
int value2 = 0;
int prevValue = 0;
int velocityValue = 0;

int prevValue2 = 0;
int velocityValue2 = 0;

uint8_t samples;
uint8_t midi_delay;
uint8_t sense_mode;
uint8_t send_mode;
uint8_t multiplex_ch;
uint8_t multiplex_ch2;

uint16_t timer = 0;
uint8_t note_off = 0;
uint8_t multiplexer_mapping[8]  = {5,7,6,4,3,0,1,2}; //remap multiplexer pin

unsigned long previousMillis = 0;        // will store last time LED was updated
const uint8_t interval = 16;           // interval at which to blink (milliseconds)

SampleFilter filter_samp;

//class filter
//{
//    public:
//        filter()
//        {
//            for(int i=0; i <= 2; i++)
//                v[i]=0;
//        }
//    private:
//        short v[3];
//    public:
//        short step(short x)
//        {
//            v[0] = v[1];
//            v[1] = v[2];
//            long tmp = ((((x * 1372318L) >>  3)	//= (   1.6359307451e-1 * x)
//                + ((v[0] * -1937502L) >> 3)	//+( -0.2309681871*v[0])
//                + ((v[1] * 1209209L) >> 1)	//+(  0.5765958891*v[1])
//                )+524288) >> 20; // round and downshift fixed point /1048576

//            v[2]= (short)tmp;
//            return (short)((
//                 (v[0] + v[2])
//                +2 * v[1])); // 2^
//        }
//};



void setAnalogMultiplexCh(const uint8_t _pin_index)
{
    // set switch to output (not sure why, but must be set everytime..)
    //pinMode(PB2, OUTPUT);
    pinMode(PB1, OUTPUT);
    pinMode(PB0, OUTPUT);

    // set multiplexer, select channel
    digitalWrite(PB2, (_pin_index & 0x01));
    digitalWrite(PB1, ((_pin_index>>1) & 0x01));
    digitalWrite(PB0, ((_pin_index>>2) & 0x01));

}


void delay(uint8_t * us)
{
    TeenyMidi.delay_us(*us);
}

void usb_poll()
{
    usbPoll();
}

void wait_for_tick(void)
{
    unsigned long timestamp;
    timestamp = millis();
    while(millis() == timestamp)
        ;
}


void wait_for_five_ticks(void)
{
    wait_for_tick();
    wait_for_tick();
    wait_for_tick();
    wait_for_tick();
    wait_for_tick();
}


uint8_t readCapacitivePin(uint8_t adcPin) {
  // Variables used to translate from Arduino to AVR pin naming
//  volatile uint8_t* port;
//  volatile uint8_t* ddr;
//  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
//  byte bitmask;
//  port = portOutputRegister(digitalPinToPort(pinToMeasure));
//  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
//  bitmask = digitalPinToBitMask(pinToMeasure);
//  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
//  *port &= ~(bitmask);
//  *ddr  |= bitmask;

  PORTB &= ~(1<<adcPin);
  DDRB |= (1<<adcPin);


  //delayMicroseconds(100);
  delay(1);
  // Make the pin an input with the internal pull-up on
//  *ddr &= ~(bitmask);
//  *port |= bitmask;

  DDRB &= ~(1<<adcPin);
  PORTB |= (1<<adcPin);

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
       if (PINB & (1<<adcPin)) { cycles =  0;}
  else if (PINB & (1<<adcPin)) { cycles =  1;}
  else if (PINB & (1<<adcPin)) { cycles =  2;}
  else if (PINB & (1<<adcPin)) { cycles =  3;}
  else if (PINB & (1<<adcPin)) { cycles =  4;}
  else if (PINB & (1<<adcPin)) { cycles =  5;}
  else if (PINB & (1<<adcPin)) { cycles =  6;}
  else if (PINB & (1<<adcPin)) { cycles =  7;}
  else if (PINB & (1<<adcPin)) { cycles =  8;}
  else if (PINB & (1<<adcPin)) { cycles =  9;}
  else if (PINB & (1<<adcPin)) { cycles = 10;}
  else if (PINB & (1<<adcPin)) { cycles = 11;}
  else if (PINB & (1<<adcPin)) { cycles = 12;}
  else if (PINB & (1<<adcPin)) { cycles = 13;}
  else if (PINB & (1<<adcPin)) { cycles = 14;}
  else if (PINB & (1<<adcPin)) { cycles = 15;}
  else if (PINB & (1<<adcPin)) { cycles = 16;}

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
//  *port &= ~(bitmask);
//  *ddr  |= bitmask;

       PORTB &= ~(1<<adcPin);
       DDRB |= (1<<adcPin);

  return cycles;
}
// End of capacitive sensing code


void setup()
{

    TeenyMidi.init();

    TeenyTouchDusjagr.begin();
    TeenyTouchDusjagr.setAdcSpeed(4);
    TeenyTouchDusjagr.delay = 8;
    TeenyTouchDusjagr.delay_cb = &delay;
    TeenyTouchDusjagr.usb_poll = &usb_poll;

    midi_delay = 5;
    sense_mode = 1;
    send_mode = 0;
    samples = 100;
    multiplex_ch = 0;
    multiplex_ch = 0;

    pinMode(PB0, OUTPUT);
    setAnalogMultiplexCh(2);

    TeenyMidi.delay(500);
    SampleFilter_init(&filter_samp);
}


void loop()
{
//    int total;
//    //unsigned long ms;

  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    if (TeenyMidi.read(&midimsg)) {                                    // need to put the ampersand "&" before "message"
        if (midimsg.key == 2) //touch ADCSRA
        {
            TeenyTouchDusjagr.setAdcSpeed(midimsg.value);
        }
        if (midimsg.key == 3) //touch delay
        {
            TeenyTouchDusjagr.delay = midimsg.value;
        }
    }
//    TeenyMidi.sendCCHires(value, 1);
  //  TeenyMidi.sendCCHires(value, 1);
    TeenyMidi.sendCCHires(SampleFilter_get(&filter_samp), 1);
   // TeenyMidi.delay(1);
   // wait_for_tick();
   // TeenyMidi.update();
    //velocityValue = value-prevValue;
    //prevValue = value;
//TeenyMidi.update();

  }


    //wait_for_tick();
    ///ms = millis();
  value = TeenyTouchDusjagr.sense(PB4,PB3, 29 );
  SampleFilter_put(&filter_samp, value);

  //value = filterx.step(TeenyTouchDusjagr.sense(PB4,PB3, 29 ));
  TeenyMidi.delay(5);


   //TeenyMidi.sendCCHires(value, 1);

    //value = readCapacitivePin(PB4);
    //value = TeenyTouchDusjagr.sense(PB4,PB3, 29);
    //ms = millis() - ms;

//    wait_for_tick();
//    total = TeenyTouchDusjagr.sense(PB4,PB3, 1);
//   wait_for_five_ticks();
//   total += TeenyTouchDusjagr.sense(PB4,PB3, 1);
//   wait_for_five_ticks();
//   total += TeenyTouchDusjagr.sense(PB4,PB3, 1);
//   wait_for_five_ticks();
//   total += TeenyTouchDusjagr.sense(PB4,PB3, 1);

//ms = micros() - ms;
//    wait_for_tick();
//    total = analogRead(2);
//    wait_for_five_ticks();
//    total += analogRead(2);
//    wait_for_five_ticks();
//    total += analogRead(2);
//    wait_for_five_ticks();
//    total += analogRead(2);

 //   total  /= 4;


    //TeenyMidi.sendCCHires(velocityValue, 1);
    //TeenyMidi.sendCCHires((int)ms, 1);
    //TeenyMidi.sendCCHires((int)test.step(value), 5);




//    mic
//    value = TeenyTouchDusjagr.sense(PB4,PB2, samples);
//    velocityValue = value-prevValue;
//    prevValue = value;
//    analogWrite(PB0, velocityValue);
//    TeenyMidi.sendCCHires(velocityValue, 5);



}
