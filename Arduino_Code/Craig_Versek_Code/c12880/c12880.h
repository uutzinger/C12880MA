/*
  c12880.h - Library for interacting with the Hamamatsu C12880 microspectrometer
  Created by Craig Wm. Versek, 2017-03-01
 */
#ifndef _C12880_H_INCLUDED
#define _C12880_H_INCLUDED

#include <Arduino.h>

#if defined(CORE_TEENSY)
#include <ADC.h> /* https://github.com/pedvide/ADC */
#endif

#define C12880_NUM_CHANNELS 288
//uncomment for experimental feature, WARNING creates artifacts above 96MHz system clock
//#define MICROSPEC_ADC_PIPELINE
/*******************************************************************************
  C12880_Class
  
*******************************************************************************/
class C12880_Class{
public:
  C12880_Class(const int TRG_pin,
                const int ST_pin,
                const int CLK_pin,
                const int VIDEO_pin
               );
  //Configuration methods
  void begin();
  void set_integration_time(float seconds);
  //Functionality methods
  void read_into(uint16_t *buffer);
  unsigned int get_timing(int index){
    return _timings[index];
  }
private:
  //helper methods
  inline void _pulse_clock(int cycles);
  inline void _pulse_clock_timed(int duration_micros);
  void _measure_min_integ_micros();
  //Attributes
  int _TRG_pin;
  int _ST_pin;
  int _CLK_pin;
  int _VIDEO_pin;
  #if defined(CORE_TEENSY)
  ADC *_adc; // adc object
  #endif
  int _clock_delay_micros;
  float _integ_time;
  int _min_integ_micros;
  unsigned int _timings[10];
};



#endif /* _C12880_H_INCLUDED */
