#define USE_KEYBOARD
#define USE_VELO

#ifdef USE_MIDI
#include <TeenyMidi.h>
MIDIMessage midimsg;
#endif

#ifdef USE_KEYBOARD
#include <TeenyKeyboard.h>
#endif


#include "TeenyTouchDusjagr.h"
#include "AnalogTouch.h"
#include "SampleFilter.h"

char key[] = {'I','Y','O','K','K','K','K','K','O','O','O','O','O'};

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

#ifdef USE_VELO
int prevValue[8] = {0,0,0,0,0,0,0,0};
int velocityValue[8] = {0,0,0,0,0,0,0,0};
#endif

uint8_t note_off[8] = {1,1,1,1,1,1,1,1};

uint16_t offset_adc[8] = {0,0,0,0,0,0,0,0};
SampleFilter filter_samp[8];

uint8_t pin_queue = 0;

uint8_t multiplexer_mapping[8]  = {6,7,4,4,3,0,1,2}; //remap multiplexer pin

unsigned long previousMillis = 0;        // will store last time LED was updated


#define PIN_SELECT 0
#define NUM_CHANNEL 3

static inline void setAnalogMultiplexCh(const uint8_t _pin_index)
{
    // set switch to output (not sure why, but must be set everytime..)
    //DDRB |= _BV(PB2) | _BV(PB1) | _BV(PB0);

    pinMode(PB2, OUTPUT);
    pinMode(PB1, OUTPUT);
    pinMode(PB0, OUTPUT);

    // set multiplexer, select channel
    digitalWrite(PB2, (_pin_index & 0x01) );
    digitalWrite(PB1, ((_pin_index>>1) & 0x01));
    digitalWrite(PB0, ((_pin_index>>2) & 0x01));

    //faster cc http://www.crash-bang.com/arduino-digital-io/
//    PORTB |= ((_pin_index & 0x01)<<PB2);
  //  PORTB |= (((_pin_index>>1) & 0x01)<<PB1);
 //   PORTB |= (((_pin_index>>2) & 0x01)<<PB0);


}

void usb_poll()
{
    //TeenyMidi.update();
    usbPoll();
}

void setup()
{

#ifdef USE_MIDI
    TeenyMidi.init();
#endif


#ifdef USE_KEYBOARD
    TeenyKeyboard.update();
#endif


    TeenyTouchDusjagr.begin();
    TeenyTouchDusjagr.setAdcSpeed(4);
    TeenyTouchDusjagr.delay = 4;
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
        #ifdef USE_MIDI
        TeenyMidi.delay(100);
        #endif
    }



}


void loop()
{

    if (millis()-previousMillis >= 5)  {  // 0% data loss
        //TeenyMidi.sendCCHires(value, 1);
        int filtered_value = 0;
        for (uint8_t i = 0; i < NUM_CHANNEL; i++)
        {
            filtered_value = SampleFilter_get(&filter_samp[i]);
            //velocityValue[i] = filtered_value - prevValue[i];
            //prevValue[i] = filtered_value;

     //       TeenyMidi.sendCCHires(filtered_value, (4*i)+1);
        //    TeenyMidi.sendCCHires(value[i], (4*i)+1);

                if (filtered_value >= 100)
                    {
                        if (note_off[i] == 1)
                            {
#ifdef USE_MIDI
                                TeenyMidi.send(MIDI_NOTEON,i, 127 );
#endif
#ifdef USE_KEYBOARD
                                TeenyKeyboard.print(key[i]);
#endif
                                note_off[i] = 0;
                            }
                    }
                else
                    {
                        if (note_off[i] == 0)
                            {
                                //TeenyMidi.send(MIDI_NOTEOFF,i,127);
                                note_off[i] = 1;
                            }
                    }


        }
        previousMillis = millis();
    }

  //sample 8 times -> good -> sometimes usb disconnect
  // sample 6 times -> good -> still testing
  //for (uint8_t i = 0; i < NUM_CHANNEL; i++)
  //{
      setAnalogMultiplexCh(pin_queue);
      value[pin_queue] = TeenyTouchDusjagr.sense(PB4,PB3, 7 ) - offset_adc[pin_queue];
      if (value[pin_queue] > 0) SampleFilter_put(&filter_samp[pin_queue], value[pin_queue]);
#ifdef USE_MIDI
      TeenyMidi.update();
#endif
  //}

  pin_queue++;
  if (pin_queue > NUM_CHANNEL) pin_queue = 0;

#ifdef USE_MIDI
  TeenyMidi.delay(1);
#endif

#ifdef USE_KEYBOARD
  //IIIIIIIIIIIIIIIOOOOOOOOOOOOOOOOOOOOOOOOOOOOOIIIIIIITeenyKeyboard.update();
  TeenyKeyboard.delay(1);
#endif


//    velocityValue[pin_queue] = value[pin_queue]-prevValue[pin_queue];
//    prevValue[pin_queue] = value[pin_queue];


}
