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

int value[8] = {0,0,0,0,0,0,0,0};
//int prevValue = 0;
//int velocityValue = 0;
uint16_t offset_adc[8] = {0,0,0,0,0,0,0,0};
SampleFilter filter_samp[8];


uint8_t multiplexer_mapping[8]  = {6,7,6,4,3,0,1,2}; //remap multiplexer pin

unsigned long previousMillis = 0;        // will store last time LED was updated


#define PIN_SELECT 0
#define NUM_CHANNEL 2

void setAnalogMultiplexCh(const uint8_t _pin_index)
{
    // set switch to output (not sure why, but must be set everytime..)
    pinMode(PB2, OUTPUT);
    pinMode(PB1, OUTPUT);
    pinMode(PB0, OUTPUT);

    // set multiplexer, select channel
    digitalWrite(PB2, (_pin_index & 0x01));
    digitalWrite(PB1, ((_pin_index>>1) & 0x01));
    digitalWrite(PB0, ((_pin_index>>2) & 0x01));

}

void usb_poll()
{
    usbPoll();
}

void setup()
{


    TeenyMidi.init();

    TeenyTouchDusjagr.begin();
    TeenyTouchDusjagr.setAdcSpeed(5);
    TeenyTouchDusjagr.delay = 8;
    //TeenyTouchDusjagr.delay_cb = &delay;
    TeenyTouchDusjagr.usb_poll = &usb_poll;

//    midi_delay = 5;
//    sense_mode = 1;
//    send_mode = 0;
//    samples = 100;
//    multiplex_ch = 0;
//    multiplex_ch = 0;

//    pinMode(PB0, OUTPUT);

    for (uint8_t i = 0; i < NUM_CHANNEL; i++)
    {
        setAnalogMultiplexCh(i);
        offset_adc[i] = TeenyTouchDusjagr.sense(PB4,PB3, 8 );
    }


    TeenyMidi.delay(500);
}


void loop()
{

    if (millis()-previousMillis >= 16)  {  // 0% data loss
        //TeenyMidi.sendCCHires(value, 1);
        for (uint8_t i = 0; i < NUM_CHANNEL; i++)
        {
            TeenyMidi.sendCCHires(SampleFilter_get(&filter_samp[i]), (4*i)+1);
        }
        previousMillis = millis();
    }

  //sample 8 times -> good -> sometimes usb disconnect
  // sample 6 times -> good -> still testing
  for (uint8_t i = 0; i < NUM_CHANNEL; i++)
  {
      setAnalogMultiplexCh(i);
      value[i] = TeenyTouchDusjagr.sense(PB4,PB3, 6 ) - offset_adc[i];
      if (value[i] > 0) SampleFilter_put(&filter_samp[i], value[i]);
      TeenyMidi.update();
  }



  TeenyMidi.delay(4);

   // velocityValue = value-prevValue;
   // prevValue = value;


}
