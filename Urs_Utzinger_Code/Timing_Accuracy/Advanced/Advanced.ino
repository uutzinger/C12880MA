#include <ADC.h>         /* https://github.com/pedvide/ADC */

const int TRG_pin   = A0;
const int ST_pin    = A1;
const int CLK_pin   = A2;
const int VIDEO_pin = A3;
#define C12880_NUM_CHANNELS 288
#define MICROSPEC_ADC_PIPELINE

uint16_t data[C12880_NUM_CHANNELS];
ADC *adc = new ADC();

int clock_delay_micros = 1;
float       integ_time = 0.01;
int   min_integ_micros = 0;
unsigned int timings[10];

//this function produces a delay for *half* clock period (100ns), approaching 5MHz
//for Teensyduino works up to 180MHz clock such as in the Teensy 3.6
static inline void ultrashort_delay_100ns(){
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

inline void pulse_clock(int cycles){
  for(int i = 0; i < cycles; i++){
    digitalWriteFast(CLK_pin, HIGH);
    ultrashort_delay_100ns();
    digitalWriteFast(CLK_pin, LOW);
    ultrashort_delay_100ns();
  }
}

inline void pulse_clock_timed(int duration_micros){
  elapsedMicros sinceStart_micros = 0;
  while (sinceStart_micros < duration_micros){
    digitalWriteFast(CLK_pin, HIGH);
    ultrashort_delay_100ns();
    digitalWriteFast(CLK_pin, LOW);
    ultrashort_delay_100ns();
  }
}

void measure_min_integ_micros() {
  //48 clock cycles are required after ST goes low
  elapsedMicros sinceStart_micros = 0;
  pulse_clock(48);
  min_integ_micros = sinceStart_micros;
}

void setup() {
  ///// ADC0 ////--------------------------------------------------------------
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0); // For Teensy 3.x ADC_REF_INTERNAL is 1.2V, default is 3.3V
  adc->setAveraging(0); // set number of averages
  adc->setResolution(12); // set bits of resolution
  adc->disableCompare(ADC_0);
  // it can be VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  // it can be VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);    // change the sampling speed, we can use high speed since we have buffer

  //Set desired pins to OUTPUT
  pinMode(CLK_pin, OUTPUT);
  pinMode(ST_pin,  OUTPUT);

  digitalWrite(CLK_pin, LOW);  // Start with CLK High
  digitalWrite(ST_pin,  LOW);  // Start with ST Low
  
  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.println("Testing timing for C12880MA");
  
  measure_min_integ_micros();
  memset((void*)data, 0, sizeof(data));

 }

void loop() {
  //compute integration time
  int duration_micros = (int) (integ_time*1e6);  // integration time in micro seconds
  duration_micros -= min_integ_micros;           // correction based on 48 pulses after ST goes low
  duration_micros = max(duration_micros,0);       // at least 0 micro seconds interation times
  // Start clock cycle 
  // Set start pulse to signal start
  digitalWriteFast(CLK_pin, HIGH);
  ultrashort_delay_100ns();
  digitalWriteFast(CLK_pin, LOW);
  digitalWriteFast(ST_pin, HIGH);
  ultrashort_delay_100ns();
  // Signal integration starts after three clock pulses
  pulse_clock(3);
  timings[0] = micros();
  // Continue with clock until desired intergration time (-48 clocks) is reached
  //pulse_clock(integ_clock_cycles-48);
  pulse_clock_timed(duration_micros);
  //Set ST_pin to low
  digitalWriteFast(ST_pin, LOW);
  timings[1] = micros();
  //Sample for a period of time
  //integration stops at pulse 48 th pulse after ST went low
  pulse_clock(48);
  timings[2] = micros();
  //pixel output is ready after last pulse #88 after ST wen low
  pulse_clock(40);
  timings[3] = micros();
  //Read from SPEC_VIDEO

  #if defined(MICROSPEC_ADC_PIPELINE)
  //WARNING this creates artifacts at clock speeds greater than 96MHz!
  //use non-blocking methods to stagger conversion with next pixel clock-out
  // 1 clock cycle takes 260ns
  // 288 channels, take 267 microseconds to readout
  // 927 ns per analog read and clock cycle generation , therefore
  // 667 ns to convert and store a sample which is 1.5MSamples/s
  adc->startContinuous(VIDEO_pin, ADC_0);          //non-blocking start single-shot mode
  for(int i = 0; i < C12880_NUM_CHANNELS; i++){
    pulse_clock(1);                               //continue to clock in the next sample while the conversion completes
    while(!adc->isComplete()){};                  //poll for completion
    data[i] = (uint16_t) adc->analogReadContinuous();
  }
  adc->stopContinuous();
  #else
   // 288 channels, take 1662 microseconds to readout
   // 5771 ns per analog read and clock generation, therfore
   // 5511 ns to convert and store which is 182kS/s
   for(int i = 0; i < C12880_NUM_CHANNELS; i++){
     data[i] = adc->analogRead(VIDEO_pin);
     pulse_clock(1);
   }
  #endif
  timings[4] = micros();

  Serial.println(timings[0]);
  Serial.println(timings[1]);
  Serial.println(timings[2]);
  Serial.println(timings[3]);
  Serial.println(timings[4]);
  Serial.println();
  Serial.println(timings[1]-timings[0]);
  Serial.println(timings[2]-timings[1]);
  Serial.println(timings[3]-timings[2]);
  Serial.println(timings[4]-timings[3]);
  Serial.println();
}
