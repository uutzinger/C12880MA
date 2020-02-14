/*
  c12880.cpp - Library for interacting with the Hamamatsu C12880
                          microspectrometer
  Created by Craig Wm. Versek, 2017-03-01
  
  Much of the initial implmentation was based on this project:
  https://github.com/groupgets/c12880ma/blob/master/arduino_c12880ma_example/arduino_c12880ma_example.ino
 */
#include <Arduino.h>

#if defined(CORE_TEENSY)
#include <ADC.h> /* https://github.com/pedvide/ADC */
#define LED_BUILTIN 13
#else
#include <elapsedMillis.h>
#endif

#include "c12880.h"
 
#define CLOCK_FREQUENCY 50000


////////////////////////////////////////////////////////////////////////////////
// High performance helper functions

//this function produces a delay for *half* clock period (100ns), approaching 5MHz
//for Teensyduino works up to 180MHz clock such as in the Teensy 3.6
static inline void _ultrashort_delay_100ns(){
#if defined(CORE_TEENSY)
  #if F_CPU <= 10000000
  //1 nops
  asm volatile("nop \n\t");
  #elif F_CPU <= 20000000
  //2 nops
  asm volatile("nop \n\tnop \n\t");
  #elif F_CPU <= 30000000
  //3 nops
  asm volatile("nop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 40000000
  //4 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 50000000
  //5 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 60000000
  //6 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 70000000
  //7 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 80000000
  //8 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 90000000
  //9 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 100000000
  //10 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 110000000
  //11 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 120000000
  //12 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 130000000
  //13 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 140000000
  //14 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 150000000
  //15 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 160000000
  //16 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 170000000
  //17 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 180000000
  //18 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #endif
#else
  //default to something that should always work in Arduino
  delayMicroseconds(1);
#endif
}

////////////////////////////////////////////////////////////////////////////////

C12880_Class::C12880_Class(const int TRG_pin,
                             const int ST_pin,
                             const int CLK_pin,
                             const int VIDEO_pin
                             ){
  _TRG_pin = TRG_pin;
  _ST_pin  = ST_pin;
  _CLK_pin = CLK_pin;
  _VIDEO_pin = VIDEO_pin;
  #if defined(CORE_TEENSY)
  _adc = new ADC();        // adc object
  #endif
  _clock_delay_micros = 1; // half of a clock period
  _min_integ_micros = 0;   // this is correction which is platform dependent and 
                           // should be measured in `begin`
  set_integration_time(0.010);  // integration time default to 1ms
}

inline void C12880_Class::_pulse_clock(int cycles){
  for(int i = 0; i < cycles; i++){
    #if defined(CORE_TEENSY)
    digitalWriteFast(_CLK_pin, HIGH);
    _ultrashort_delay_100ns();
    digitalWriteFast(_CLK_pin, LOW);
    _ultrashort_delay_100ns();
    #else
    digitalWrite(_CLK_pin, HIGH);
    _ultrashort_delay_100ns();
    digitalWrite(_CLK_pin, LOW);
    _ultrashort_delay_100ns();
    #endif
  }
}

inline void C12880_Class::_pulse_clock_timed(int duration_micros){
  elapsedMicros sinceStart_micros = 0;
  while (sinceStart_micros < duration_micros){
    #if defined(CORE_TEENSY)
    digitalWriteFast(_CLK_pin, HIGH);
    _ultrashort_delay_100ns();
    digitalWriteFast(_CLK_pin, LOW);
    _ultrashort_delay_100ns();
    #else
    digitalWrite(_CLK_pin, HIGH);
    _ultrashort_delay_100ns();
    digitalWrite(_CLK_pin, LOW);
    _ultrashort_delay_100ns();
    #endif
  }
}

void C12880_Class::begin() {
  #if defined(CORE_TEENSY)
  ///// ADC0 ////--------------------------------------------------------------
  //adc->setReference(ADC_REF_INTERNAL, ADC_0); For Teensy 3.x ADC_REF_INTERNAL is 1.2V, default is 3.3V
  _adc->setAveraging(1); // set number of averages
  _adc->setResolution(16); // set bits of resolution
  // it can be VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // see the documentation for more information
  _adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);//HIGH_SPEED_16BITS); // change the conversion speed
  // it can be VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  _adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);//HIGH_SPEED); // change the sampling speed
  #else
  //DUE and ZERO (SAMD21) have 12-bit capability
  analogReadResolution(12);
  #endif
  //Set desired pins to OUTPUT
  pinMode(_CLK_pin, OUTPUT);
  pinMode(_ST_pin, OUTPUT);
  
  digitalWrite(_CLK_pin, LOW); // Start with CLK High
  digitalWrite(_ST_pin, LOW);  // Start with ST Low
  
  _measure_min_integ_micros();
}

void C12880_Class::_measure_min_integ_micros() {
  //48 clock cycles are required after ST goes low
  elapsedMicros sinceStart_micros = 0;
  _pulse_clock(48);
  _min_integ_micros = sinceStart_micros;
}

void C12880_Class::set_integration_time(float seconds) {
  _integ_time = max(seconds, 0);
}


void C12880_Class::read_into(uint16_t *buffer) {
  //compute integration time
  int duration_micros = (int) (_integ_time*1e6);
  duration_micros -= _min_integ_micros; //correction based on 48 pulses after ST goes low
  duration_micros = max(duration_micros,0);
  // Start clock cycle and set start pulse to signal start
  digitalWrite(_CLK_pin, HIGH);
  _ultrashort_delay_100ns();
  digitalWrite(_CLK_pin, LOW);
  digitalWrite(_ST_pin, HIGH);
  _ultrashort_delay_100ns();
  //pixel integration starts after three clock pulses
  _pulse_clock(3);
   _timings[0] = micros();
  //Integrate pixels for a while
  //_pulse_clock(_integ_clock_cycles);
  _pulse_clock_timed(duration_micros);
  //Set _ST_pin to low
  digitalWrite(_ST_pin, LOW);
  _timings[1] = micros();
  //Sample for a period of time
  //integration stops at pulse 48 th pulse after ST went low
  _pulse_clock(48);
  _timings[2] = micros();
  //pixel output is ready after last pulse #88 after ST wen low
  _pulse_clock(40);
  _timings[3] = micros();
  //Read from SPEC_VIDEO

#if defined(CORE_TEENSY)
  #if defined(MICROSPEC_ADC_PIPELINE)
  //WARNING this creates artifacts at clock speeds greater than 96MHz!
  //use non-blocking methods to stagger conversion with next pixel clock-out
  _adc->startContinuous(_VIDEO_pin, ADC_0); //non-blocking start single-shot mode
  for(int i = 0; i < C12880_NUM_CHANNELS; i++){
    _pulse_clock(1);                          //continue to clock in the next sample while the conversion completes
    while(!_adc->isComplete()){}; //poll for completion
    buffer[i] = (uint16_t) _adc->analogReadContinuous();
  }
  _adc->stopContinuous();
  #else
   for(int i = 0; i < C12880_NUM_CHANNELS; i++){
     buffer[i] = _adc->analogRead(_VIDEO_pin);
     _pulse_clock(1);
   }
  #endif
#else
  for(int i = 0; i < C12880_NUM_CHANNELS; i++){
    buffer[i] = analogRead(_VIDEO_pin);
    _pulse_clock(1);
  }
#endif
  _timings[4] = micros();
}

