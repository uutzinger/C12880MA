/*
  c12880.cpp - Library for interacting with the Hamamatsu C12880
                          microspectrometer
  Created by Craig Wm. Versek, 2017-03-01
  
  Much of the initial implmentation was based on this project:
  https://github.com/groupgets/c12880ma/blob/master/arduino_c12880ma_example/arduino_c12880ma_example.ino
 */
#include <Arduino.h>
#include <ADC.h> /* https://github.com/pedvide/ADC */
#include <DMAChannel.h>
#include "c12880.h"

#define SAMPLE_AVERAGING 0                  // 0, 4, 8, 16 or 32
#define SAMPLE_RESOLUTION 12                // 8, 10, 12 or 16 

////////////////////////////////////////////////////////////////////////////////
// High performance helper functions

//this function produces a delay for *half* clock period (100ns), approaching 5MHz
//for Teensyduino works up to 180MHz clock such as in the Teensy 3.6
static inline void _ultrashort_delay_100ns(){
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
}

////////////////////////////////////////////////////////////////////////////////

C12880::C12880(const int TRG_pin,
               const int ST_pin,
               const int CLK_pin,
               const int VIDEO_pin
                     ){
  _TRG_pin            = TRG_pin;
  _ST_pin             = ST_pin;
  _CLK_pin            = CLK_pin;
  _VIDEO_pin          = VIDEO_pin;
  _adc                = new ADC(); // adc object
  _clock_delay_micros = 1;         // half of a clock period
  _min_integ_micros   = 0;         // this is correction which is platform dependent and 
                                   // should be measured in `begin`
  set_integration_time(0.010);     // integration time default to 1ms
}

inline void C12880::_pulse_clock(int cycles){
  // takes 261.05 nano seconds per cycle
  for(int i = 0; i < cycles; i++){
    digitalWriteFast(_CLK_pin, HIGH); // takes 10.6 nano seconds
    _ultrashort_delay_100ns();        // takes 120.09 nano seconds
    digitalWriteFast(_CLK_pin, LOW);
    _ultrashort_delay_100ns();
  }
}

inline void C12880::_pulse_clock_timed(int duration_micros){
  elapsedMicros sinceStart_micros = 0;
  while (sinceStart_micros < duration_micros){
    digitalWriteFast(_CLK_pin, HIGH);
    _ultrashort_delay_100ns();
    digitalWriteFast(_CLK_pin, LOW);
    _ultrashort_delay_100ns();
  }
}

void C12880::begin() {
  ///// ADC0 ////--------------------------------------------------------------adc->disablePGA(ADC_0)
  _adc->adc0->disablePGA();
  _adc->adc0->setReference(ADC_REFERENCE::REF_3V3); //For Teensy 3.x ADC_REF_INTERNAL is 1.2V, default is 3.3V
  _adc->adc0->setAveraging(SAMPLE_AVERAGING); // set number of averages
  _adc->adc0->setResolution(SAMPLE_RESOLUTION); // set bits of resolution
  // it can be VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  _adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  // it can be VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  _adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);    // change the sampling speed, we can use high speed since we have buffer

  //Set desired pins to OUTPUT
  pinMode(_CLK_pin, OUTPUT);
  pinMode(_ST_pin,  OUTPUT);
  
  digitalWrite(_CLK_pin, LOW);  // Start with CLK High
  digitalWrite(_ST_pin,  LOW);  // Start with ST Low
  
  _measure_min_integ_micros();
}

void C12880::_measure_min_integ_micros() {
  //48 clock cycles are required after ST goes low
  elapsedMicros sinceStart_micros = 0;
  _pulse_clock(48);
  _min_integ_micros = sinceStart_micros;
}

void C12880::set_integration_time(float seconds) {
  _integ_time = max(seconds, float(_min_integ_micros)/1.0E6);
}

void C12880::read(uint16_t *buffer) {
  //compute integration time
  int duration_micros = (int) (_integ_time*1e6);  // integration time in micro seconds
  duration_micros -= _min_integ_micros;           // correction based on 48 pulses after ST goes low
  duration_micros = max(duration_micros,0);       // at least 0 micro seconds interation times
  // Start clock cycle 
  // Set start pulse to signal start
  digitalWriteFast(_CLK_pin, HIGH);
  _ultrashort_delay_100ns();
  digitalWriteFast(_CLK_pin, LOW);
  digitalWriteFast(_ST_pin, HIGH);
  _ultrashort_delay_100ns();
  // Signal integration starts after three clock pulses
  _pulse_clock(3);
   _timings[0] = micros();
  // Continue with clock until desired intergration time (-48 clocks) is reached
  //_pulse_clock(_integ_clock_cycles-48);
  _pulse_clock_timed(duration_micros);
  //Set _ST_pin to low
  digitalWriteFast(_ST_pin, LOW);
  _timings[1] = micros();
  //Sample for a period of time
  //integration stops at pulse 48 th pulse after ST went low
  _pulse_clock(48);
  _timings[2] = micros();
  //pixel output is ready after last pulse #88 after ST wen low
  _pulse_clock(40);
  _timings[3] = micros();
  //Read from SPEC_VIDEO

  #if defined(MICROSPEC_ADC_PIPELINE)
  //WARNING this creates artifacts at clock speeds greater than 96MHz!
  //use non-blocking methods to stagger conversion with next pixel clock-out
  _adc->adc0->startContinuous(_VIDEO_pin); //non-blocking start single-shot mode
  for(int i = 0; i < C12880_NUM_CHANNELS; i++){
    _pulse_clock(1);                          //continue to clock in the next sample while the conversion completes
    while(!_adc->adc0->isComplete()){}; //poll for completion
    buffer[i] = (uint16_t) _adc->adc0->analogReadContinuous();
  }
  _adc->adc0->stopContinuous();
  #else
   for(int i = 0; i < C12880_NUM_CHANNELS; i++){
     buffer[i] = _adc->adc0->analogRead(_VIDEO_pin);
     _pulse_clock(1);
   }
  #endif
  _timings[4] = micros();
}
