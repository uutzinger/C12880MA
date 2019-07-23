// *********************************************************************************************//
// Hamamtsu C12880 sensor control with Teensy 3
// *********************************************************************************************//
// This sofware requires a Teensy 3.x Microcontroller.
// This software utilizes a microncotroller genereated clock to synchorinze readout of the 
// spectrometer. The spectromter provides an analog to digitcal conversion clock to trigger 
// the ADC of the teensy externally. Data is stored in a DMA buffer after each conversion.
// The maximum ADC reference votlage on the Teensy is 3.3V while the staturation on C12880 is 4.3V
// The output offset voltage is about 0.5V. The readoutout noise is about 1.8 mV rms. Therefore the
// the ADC resolution should be set to 12 bit (1bit = 0.8mV). 
// With a Teensy 3.2, a maximum ADC clock rate of 830kHz can be expected at "very high" conversion
// speeds and 430kHz at "high" conversion speeds at 12bit resolution. The C12880 allows a readout 
// rate between 200kHz and 5MHz. The clock frequency should be set to 200kHz and increased until
// noise and performance meet your requirements.
//
// The C12880 288 spectral channels cover 340-850nm with a resolutin of 12nm.
//
// This software implementation can not be ported to an Arduino UNO or simlar as it requires an
// ADC that can be externally triggered and has >=200 kS/sec performance at 12 bit resolution.
//
// This driver was developed because the open source programs for the C12880 read the sensor
// at frequencies below the sensors specifications or the clock they are producing is not stable.
//
// Setup
//
// The SPECTROMETER CLOCK will need to be connected to the pin specified by SPEC_CLK
// The SPECTROMETER CLOCK will also need to be connected to INT_Pin.
// The SPEC_TRG will need to be connected to pin 11 on the Teensy 3.2 to trigger the PDB (ADC). 
// (It is possible to also trigger ADC with LPTMR input pin but for the implemention here
// pin 11 was chosen. PDB external trigger can only be on pin 11.)
// The SPECTROMETER ST will need to be connected to the SPEC_ST pin on the Teensy. 
// It is used to start the conversion and to control the sensor exposure time.
// The SPECTROMETER VIDEO signal will need to be connected to an analog input pin on the Teensy.
// The SPECTROMTER EOS (end of scan) signal will need to be connected to the SPEC_EOS pin on
// the Teensy.
// SPEC_CLK             23 
// INT_Pin              21 // connected also to SPEC_CLK to count clocks
// SPEC_TRG             11 // ADC signal trigger to microcontroller, needs to go to pin 11
// SPEC_ST              12 // Sensor start pulse from microcontroller
// SPEC_VIDEO           14 // Sensor signal, analog in, buffered, low impedance to microcontroller
// SPEC_EOS             13 // Sensor End of Scan, to microcontroller
// The above connections are mandatory. 
// FLASH_TRIGGER        18 // Light on/off from microcontroller
// The above connection is voluntary. 
//
// Some breakout boards such as the 
// https://groupgets.com/manufacturers/hamamatsu-photonics/products/c12880ma-micro-spectrometer
// provide a white LED which can be triggered with FLASH_TRIGGER.
// If you provide your own illumination a digital output pin can be configured to start the 
// illumination before the exposure occurs and to turn it off after readout completed.
// The FLASH_TRIGGER pin is used for this purpose.
//
// Although readout occurs in the background, it is recommended to not disturb the microcontroller
// during a sensor readout when high speed readout is utilized. 
// After the readout, the micronctroller can transfer data to the host and deal with user input.
//
// With the Teensy, the USB speed is 12 Mbit/sec which means that in binary transfer mode
// up to 2000 spectra per second could be sent to the host computer.
// The minimum measurement cycle on the C12880 is 381/clock_freq and with a 800kHz clock that would 
// result in 2,000 spectra per second with an integration time of 54/800kHz (68 microseconds).
// If we had a 5M samples/sec ADC conversion available we could measure up to 12,000 spectra
// per second and for transfer to the host, USB 2.0 would be needed.
//
// Urs Utzinger, March 2019, Tucson Arizona, USA
//
// *********************************************************************************************//
// Copyright (c) 2019 - Urs Utzinger
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
// and associated documentation files (the "Software"), to deal in the Software without restriction, 
// including without limitation the rights to use, copy, modify, merge, publish, distribute, 
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial 
// portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// *********************************************************************************************//
//
// The ADC library was forked from pedvide/ADC to https://github.com/uutzinger/ADC. It was modified
// to include external triggering capabilities.

#include "ADC.h"
#include <DMAChannel.h>

#define C12880_NUM_CHANNELS 288 // Number of spectral channels
#define SPEC_TRG             11 // ADC signal trigger to microcontroller, needs to go to pin 11
#define SPEC_ST              12 // Sensor start pulse from microcontroller
#define SPEC_CLK             23 // Sensor clock from microcontroller
#define SPEC_VIDEO           14 // Sensor signal, buffered, low impedance to microcontroller
#define SPEC_EOS             13 // Sensor End of Scan, to microcontroller
#define FLASH_TRIGGER        18 // Light on/off from microcontroller
#define INT_Pin              21 // Connect to SPEC_CLK to count clock cycles
// ----------------------------------------------------------
// PWM pins can be 3,4,5,6,9,10,20,21,22,23,25,32
// ADC pins can be 14,15,16,17,18,19,20,21,22,23,26,27,28, 29,30,31,A10,A11,A12,A13
// Pin 11 is used for ADC trigger and can not be changed
// SERIAL_TX and RX can be: 1,0 3,4
// EOS, FLASH, INT_PIN can be: 0..23, 24..33
// https://www.pjrc.com/teensy/pinout.html
// ----------------------------------------------------------
// RX/TX will be used to connect Teensy to host controller via RX/TX (if host does not have USB)
// This will require using the serial libary
// https://www.pjrc.com/teensy/td_uart.html
#define mySerial Serial         // Communicatge with USB
// #define mySerial Serial1        // Hardware Serial, communicate with RX,TX bins (0,1) to other microcontroller
#define SERIAL_TX             1 // Serial transmission
#define SERIAL_RX             0 // Serial receiving
#define BAUDRATE         115200 // 2000000 works likely also

// ----------------------------------------------------------
// States:
// ----------------------------------------------------------
// Idle:        Do nothing related to sensor readout
// PreStart:    Prepare start
// Start_ST:    Run 1 clock cycle then set ST high
// Stop_ST:     Run 3 clocks + integrationtime - 48 clocks then set ST low
// Pre_Readout: Run 48 + 40 clock cycles then start ADC
// Readout:     Run C12880_NUM_CHANNELS clocks for readout
// ----------------------------------------------------------
// [In this implementation ADC is started when ST goes low and stopped when EOS goes high]

enum states {preStart, startST, stopST, preReadout, readout, idle, userInput, trasferData};
volatile states myState = idle;

// Interrupt Counter
volatile long int count = 0; 
volatile long int integrationCounts=0;
volatile bool spectrumReady = false;
float  integrationTime = 0.01;

uint16_t CLK_Pin       = SPEC_CLK;                       // Pin to connect spectrometer clock
uint32_t CPU_Frequency = F_CPU/1E6;                      // MHz
uint32_t CLK_Frequency = 200000;                         // Clock Frequency
uint32_t PWM_MaxFreq;                                    //
uint16_t PWM_Resolution=4;                               // 4: 0..15
uint16_t PWM_MaxValue;                                   // if resolution is 4 then max value is 15
float    PWM_Duty      = 50.0;                           // a nice symmetric clock
bool     highSpeed     = true;                           // Very high speed for sampling and conversion
bool     verbose       = true;                           // Transmitt date in verbose mode
bool     continous     = false;                          // Dont start with continous transmission
bool     aPlotter      = false;                          // Format data for Arduino serial plotter
// variables used in the interrupt service routines (need to be volatile)
volatile uint32_t pdbticks, adcticks, adcval, last_read;
volatile bool     adc0_busy = false;
volatile bool     flashEnabeld = false;

/******************************************************************************/
ADC *adc = new ADC();
DMAChannel dma0;
DMAMEM static uint16_t spectrum[C12880_NUM_CHANNELS+88+1];         // buffer, 
// ADC starts recording when ST signal goes low and continues until EOS 
// goes high. Therfore ADC buffer should be 88 cyles longer in beginning and 
// one cycle longer at end compared to the number of spectral channels available.
/******************************************************************************/

/******************************************************************************/
// Clock Counter and State Machine
/******************************************************************************/
// This interrupt service routine counts the clk pulses sent to spectrometer 
// on the INT_pin and creates the ST signal 
// [The CLK is produced with analogWrite for the C12880 SPEC_CLK pin]
// ----------------------------------------------------------
// ST Cycle minimum length    381/f_clk
// ST High period minimum       6/f_clk
// ST Low perdiod minimum     375/f_clk 
// ST Low is start configuration
// ST High starts sequence
// Integration starts at 4th clock cycle
// Integration finishes 51 cycles after ST switched to low
// Integration time is ST high length plus 48 clks
// Signal appears at 89th pulse and continues 288 pulses
// ----------------------------------------------------------
// 1 Run clock one cycle  
// 2 Set ST_pin high
// 3 Run clock 3 pulses to start signal integration
// 4 Run clock until up to 48 clock cycles before integration time expires
// 5 Set ST pin low
// 6 Run clock for 48 cycles until end of integration time
// 7 Run clock for an other 40 cycles 
// 8 Run clock for C12880_NUM_CHANNELS and run ADC
// ----------------------------------------------------------
void counterloop() { 
  count--; // allways count down when clk rises
  // when coutner is at zero, complete the state and prepare for next state
  if (count <= 0) { 
    switch(myState)
    {       
      case idle:
        // do nothing                     
        break;
      case preStart:
        myState = startST;
        count = 1;
        break;
      case startST:      
        digitalWriteFast(SPEC_ST, HIGH); 
        if (flashEnabeld) { digitalWrite(FLASH_TRIGGER,HIGH); }
        myState = stopST;
        count =  integrationCounts; // 3+integrationtime-48;  
        break;
      case stopST:       
        digitalWriteFast(SPEC_ST, LOW); 
        myState = preReadout;
        count = 88; // 48+40;                 
        break;
      case preReadout:   
        // enable ADC;
        myState = readout;
        count = C12880_NUM_CHANNELS;
        break;
      case readout:  
        // complete ADC 
        myState = idle;
        break;
      default:
        break;
    }
  }
}


/******************************************************************************/
// Programmable Delay Block Interrupt Service Routine
// Not needed
void pdb_isr(void) {
  PDB0_SC = (PDB_SC_TRGSEL(00) | PDB_SC_PDBEN | PDB_SC_PDBIE ) | PDB_SC_LDOK; // (also clears interrupt flag)
  pdbticks++;
}

/******************************************************************************/
// ADC interrupt service routine
// Its not needed when DMA is enabled
void adc0_isr() {
  adcticks++;
  adcval = adc->adc0->readSingle(); // read and clear interrupt,  dma still occurs
}

/******************************************************************************/
// DMA interrupt service routine
// This is called when the DMA buffer is full
void dma0_isr(void) {
  adc0_busy = false;
  dma0.clearInterrupt();
  dma0.clearComplete();
}

/******************************************************************************/
// End of scan interrupt service routine
void EOS_isr() {
  last_read = adcticks - 1; // last valid ADC reading was one clock ago
  adc0_busy = false;
  spectrumReady = true;
  // Stop ADC
  dma0.disable();
  adc->disableDMA(ADC_0);
  adc->adc0->stopExtTrigPDB(true);
  if (flashEnabeld) { digitalWrite(FLASH_TRIGGER,LOW); }
}

/******************************************************************************/
// ST interrupt service routine
// Start ADC
void STfall_isr() {
  spectrumReady = false;
  adc0_busy = true;
  adcticks = 0;
  last_read = 0;
  adc->adc0->startExtTrigPDB(false);               // enable external trigger and its interrupt
  adc->adc0->startSingleRead(SPEC_VIDEO);          // call this to setup everything before the pdb starts
  adc->enableInterrupts(ADC_0);                    // not needed, but usuful here
  adc->enableDMA(ADC_0);                           // set ADC_SC2_DMAEN
  dma0.enable();                                   //
}

/******************************************************************************/
// Setup ADC and DMA
void setADC() {
  // Initialize ADC
  // Maximum Resolution of C12880 is 12bit
  // VeryHigh conversion speed might be outside of Teensy specifications
  // VeryHigh sampling speed requires low impednace output buffer at signal stage to provide sufficient current
  // Enabeling programmable gain amplifier might reduce ADC speed
  // ADC reference is 3.3V max 
  adc->disablePGA(ADC_0);        
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
  adc->setAveraging(0, ADC_0); 
  adc->setResolution(12, ADC_0); 
  adc->disableCompare(ADC_0);
  if (highSpeed) {
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_0);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_0);
  } else {
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_0);
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_0);
  }
  // Initialize dma
  dma0.source((volatile uint16_t&)ADC0_RA);
  dma0.destinationBuffer(spectrum, sizeof(spectrum));
  dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0.interruptAtCompletion();
  dma0.disableOnCompletion();                    // Depends on what you want to do after DMA buffer is full
  dma0.attachInterrupt(&dma0_isr);
}

// SET CLK signal on CLK pin using PWM modulation. Usually duty cycle is 50.0%
void setCLK(uint16_t CLK_pin, uint32_t CLK_Frequency, float PWM_Duty, float PWM_MaxValue){
  analogWriteFrequency(CLK_pin, CLK_Frequency);
  analogWrite(CLK_pin, uint16_t(PWM_Duty*PWM_MaxValue/100.0));
}

/******************************************************************************/
// Transmit Spectrum to Host 
// Either transmit ASCII data separated by "," or transmit binary data 
// At end of transmission send "new line" character.
//
// Currently tranmit all data to host.
// If things look allright it should transmit 
//    [last_read-C12880_NUM_CHANNELS+1 .. last_read] 
//    additional data could be used to determine ADC offset and noise
void transmitSpectrum(uint16_t *data) {
  size_t i;
  if (aPlotter) {
    for (i = 0; i < C12880_NUM_CHANNELS+88+1; i++) { 
      mySerial.printf("%d\n",data[i]); 
    }
    mySerial.write('\n');
  } else {
    if (!verbose) {
      for (i = 0; i < C12880_NUM_CHANNELS+88+1; i++) {
        mySerial.write( (byte *) &data[i], sizeof(data[i]));
      }
    } else {
      for (i = 0; i < C12880_NUM_CHANNELS+88+1; i++) { 
        mySerial.printf("%d, ",data[i]); 
      }
    }
    mySerial.write('\n');
  }
}

/******************************************************************************/
// What are the software options?
void printHelp() {
  mySerial.println("C12880 Controller");
  mySerial.println("===========================================");
  mySerial.println("Set  Intergation time: ............ I0.01");
  mySerial.println("Show Integration time: ............ i");
  mySerial.println("Set  CLK frequency: ............... C200000");
  mySerial.println("Show CLK frequency: ............... c");
  mySerial.println("Set  CLK duty: .................... P50");
  mySerial.println("Show CLK duty: .................... p");
  mySerial.println("Use/dont use flash: ............... F/f");
  mySerial.println("Use/dont use high speed conversion: H/h");
  mySerial.println("Set ADC parameters: ............... s");
  mySerial.println("Aquire spectrum: .................. x");
  mySerial.println("Continously aquire and transmit: .. R");
  mySerial.println("Transmit spectrum buffer:.......... r");
  mySerial.println("Use/dont use binary transmission: . B/b");
  mySerial.println("Format for Arduino serial plotter:  A/a");
  mySerial.println("================== 2019 ===================");
  mySerial.println("Urs Utzinger, Tucson, Arizona, USA         ");
  mySerial.println("===========================================");
}

/******************************************************************************/
// SerialCommand handlers

void processInstruction(String instruction) {
  String value    = "0.01";
  String command  = "o";
  float  tempFloat;
  long   tempInt;
  int instructionLength = instruction.length();
  if (instructionLength > 0) { command = instruction.substring(0,1); } 
  if (instructionLength > 1) {   value = instruction.substring(1,instructionLength); }
  //mySerial.println(command);
  //mySerial.println(value);

  if (command =='I') { // set integration Time
    integrationTime = value.toFloat();
    if ((integrationTime < 0.0) || (integrationTime > 9999.99)) { mySerial.println("Integration time out of valid Range"); }
      if (myState == idle) {
        mySerial.printf("Integratin time is: %g\n", integrationTime);
        integrationCounts = uint32_t(integrationTime * float(CLK_Frequency)) - 45;  // 3 + integrationTime - 48 
      } else { mySerial.printf("Instrument is reading data, can not change readout parameters!"); }
  } else if (command =='i') { // display integration time
    mySerial.printf("Integration time is: %g\n", integrationTime);

  } else if (command == "f") { // turn off flash
    flashEnabeld = false;
    mySerial.printf("Flash is off\n");
  } else if (command == "F") { // turn on flash
    flashEnabeld = true;
    mySerial.println("Flash is on\n");
  
  } else if (command == "b") { // verbose
    verbose = false;
    mySerial.printf("Verbose is on\n");
  } else if (command == "B") { // binary
    verbose = true;
    mySerial.printf("Binary is on\n");

  } else if (command == "a") { // regular data transmission
    aPlotter = false;
    mySerial.printf("Regular data transmission is on\n");
  } else if (command == "A") { // Arduino Serial Plotter
    aPlotter = true;
    mySerial.printf("Arduino Serial Plotter mode is on\n");

  } else if (command == "x") { // expose and measure
    mySerial.printf("Reading Spectrum\n");
    if (myState == idle) {
      myState = preStart;
    } else { mySerial.printf("Instrument is reading data, can not start reading!"); }

  } else if (command == "h") { // verbose
    if (myState == idle) { 
    highSpeed = false;
    setADC(); 
    mySerial.printf("Highspeed sampling is off\n");
    } else { mySerial.printf("Instrument is reading data, can not change readout parameters!"); }
  } else if (command == "H") { // binary
    if (myState == idle) { 
    highSpeed = true;
    setADC();
    mySerial.printf("Highspeed sampling is on\n");
    } else { mySerial.printf("Instrument is reading data, can not change readout parameters!"); }

  } else if (command == "s") { // set ADC and readout electronics
    mySerial.printf("Programming Readout Electronics\n");
    if (myState == idle) {
      setCLK(CLK_Pin, CLK_Frequency, PWM_Duty, PWM_MaxValue);
      setADC();
    } else { mySerial.printf("Instrument is reading data, can not change readout parameters!"); }

  } else if (command == "r") { // readout once
    continous = false;
    if (myState == idle) {
      transmitSpectrum(spectrum);
      spectrumReady = false;
    } else { mySerial.printf("Instrument is already reading data!"); }
  } else if (command == "R") { // readout continous
    continous = true;

  } else if (command == "C") { // set readout clock
    tempInt = value.toInt();
    if ((PWM_MaxFreq > tempInt) && (tempInt >= 50000)) {
      if (myState == idle) {
        CLK_Frequency = tempInt;
        setCLK(CLK_Pin, CLK_Frequency, PWM_Duty, PWM_MaxValue);
        integrationCounts = uint32_t(integrationTime * float(CLK_Frequency)) - 45;  // 3 + integrationTime - 48 
      } else { mySerial.printf("Instrument is reading data, can not change readout parameters!"); }
    }
  } else if (command == "c") {
    mySerial.printf("CLK Frequency is: %u\n", CLK_Frequency);

  } else if (command == "P") { // set clock duty cycle
    tempInt = value.toFloat();
    if ((tempFloat < 100.0) && (tempFloat > 0.0)) {
      if (myState == idle) {
        PWM_Duty = tempFloat;
        setCLK(CLK_Pin, CLK_Frequency, PWM_Duty, PWM_MaxValue);
      } else { mySerial.printf("Instrument is reading data, can not change readout parameters!"); }
    } 
  } else if (command == "p") {
    mySerial.printf("CLK Dutycycle is: %f\n", PWM_Duty);

  } else { // invalid input, display status
    printHelp();
  } // end if else switch
} // end process instruction

float GetMaxPWMFreqValue(uint32_t FREQ, uint16_t PWM_Resolution)
{
  /* for Teensy CPU frequency 24MHZ 48MHZ 72MHZ 96MHZ 120MHZ  */
  int FREQ_pointer=-1;
  float PWM_ideal_frequency[5][15]
  {
    { 6000000, 3000000, 1500000,  750000, 375000, 187500,  93750,  46875  , 23437.5 , 11718.75 ,  5859.375, 2929.687, 1464.843,  732.421, 366.2109},
    {12000000, 6000000, 3000000, 1500000, 750000, 375000, 187500,  93750  , 46875   , 23437.5  , 11718.75 , 5859.375, 2929.687, 1464.843, 732.4218},
    { 9000000, 4500000, 2250000, 1125000, 562500, 281250, 140625,  70312  , 35156.25, 17578.12 ,  8789.062, 4394.531, 2197.265, 1098.632, 549.3164},
    {12000000, 6000000, 3000000, 1500000, 750000, 375000, 187500,  93750  , 46875   , 23437.5  , 11718.75 , 5859.375, 2929.687, 1464.843, 732.4218},
    {15000000, 7500000, 3750000, 1875000, 937500, 468750, 234375, 117187.5, 58593.75, 29296.875, 14648.537, 7324.219, 3662.109, 1831.055, 915.527 }
  };
  switch(FREQ){
    case 24:  FREQ_pointer=0;  break;
    case 48:  FREQ_pointer=1;  break;
    case 72:  FREQ_pointer=2;  break; 
    case 96:  FREQ_pointer=3;  break; 
    case 120: FREQ_pointer=4;  break;          
    default:  FREQ_pointer=-1; break;          
  }
  if (FREQ_pointer >= 0) { return(PWM_ideal_frequency[FREQ_pointer][PWM_Resolution-2]); }
  else {                   return(488.28); }
} // end getMaxPWMFreqValue


/******************************************************************************/
void setup(){

  while (millis() < 3000) ;
  mySerial.begin(BAUDRATE);
  mySerial.setTimeout(1);                              // Serial read timeout
  printHelp();

  memset((void*)spectrum, 0, sizeof(spectrum));
  PWM_MaxFreq = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
  PWM_MaxValue = pow(2,PWM_Resolution);  

  pinMode(SPEC_TRG,      INPUT);
  pinMode(SPEC_ST,       OUTPUT);
  pinMode(CLK_Pin,       OUTPUT);
  pinMode(SPEC_VIDEO,    INPUT);
  pinMode(SPEC_EOS,      INPUT);
  pinMode(INT_Pin,       INPUT);
  pinMode(FLASH_TRIGGER, OUTPUT);
  pinMode(SERIAL_TX,     OUTPUT);
  pinMode(SERIAL_RX,     INPUT);

  digitalWriteFast(SPEC_ST,       LOW); 
  digitalWriteFast(CLK_Pin,       LOW); 
  digitalWriteFast(FLASH_TRIGGER, LOW); 
  digitalWriteFast(SERIAL_TX,     LOW); 

  attachInterrupt(INT_Pin,  counterloop,  RISING);
  attachInterrupt(SPEC_EOS, EOS_isr,      RISING);  // end of scan to stop ADC and illumination
  attachInterrupt(SPEC_ST,  STfall_isr,   FALLING); // shortly before video starts to start ADC

  // Start Clock
  setCLK(CLK_Pin, CLK_Frequency, PWM_Duty, PWM_MaxValue);
  setADC();

  count = 1;
  myState = idle;

} // end setup

/******************************************************************************/
// Variables for main loop
char   inBuff[] = "----------------";
int    bytesread;

void loop(){
  if (mySerial.available()) {
     if (myState == idle) {
       bytesread=mySerial.readBytesUntil('\n', inBuff, 16); // Read from serial until CR is read or timeout exceeded
       inBuff[bytesread]='\0';
       String instruction = String(inBuff);
       processInstruction(instruction);
     }
  }

  if (continous) {
    if ((myState == idle) && (spectrumReady==false)) {
      myState = preStart;
    }
    if (spectrumReady == true) {
      transmitSpectrum(spectrum);
      spectrumReady = false;
    }
  } 
}
