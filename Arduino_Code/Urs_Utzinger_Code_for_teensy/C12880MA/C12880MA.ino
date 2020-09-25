#include "c12880.h"

#define SPEC_TRG         A0 // ADC signal trigger to microcontroller
#define SPEC_ST          A1 // Sensor start pulse from microcontroller
#define SPEC_CLK         A2 // Sensor clock from microcontroller
#define SPEC_VIDEO       A3 // Sensor signal, buffered, low impedance to microcontroller
#define SPEC_EOS         A4   // Sensor End of Scan, not connected yet, to microcontroller
#define FLASH_TRIGGER    A5 // Light on/off from microcontroller

/*
ST Cycle minimum length is 381/f_clk
ST High period minimum 6/f_clk
ST Low perdiod minimum 375/f_clk 
ST Low is start configuration
ST High starts sequence
Integration starts at 4th clock cycle
ST Low
Integration finishes 51 cycles after ST switched to low
Integration time is ST high length plus 48 clks
Signal appears at 89th pulse and continues 288 pulses
*/

IntervalTimer flashTimer;

uint16_t spectrum[C12880_NUM_CHANNELS];

C12880 c12880(SPEC_TRG,SPEC_ST,SPEC_CLK,SPEC_VIDEO);

/******************************************************************************/
void setup(){
  
  pinMode(FLASH_TRIGGER,OUTPUT);
  digitalWrite(FLASH_TRIGGER,LOW);

  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.setTimeout(1);                              // Serial read timeout
  printHelp();

  memset((void*)spectrum, 0, sizeof(spectrum));
  c12880.begin();
 
}

/******************************************************************************/
char   inBuff[] = "----------------";
int    bytesread;
String value    = "0.01";
String command  = "o";
float  integrationTime = 0.01;
float  triggerTime     = 0.01;
bool   continous = false;
bool   BINARY = false;

void loop(){
  if (Serial.available()) {
     bytesread=Serial.readBytesUntil('\n', inBuff, 16); // Read from serial until CR is read or timeout exceeded
     inBuff[bytesread]='\0';
     String instruction = String(inBuff);
     processInstruction(instruction);
  }
  if (continous) {
    c12880.read(spectrum);
    transmitSpectrum(spectrum);
  } else {
    delay(10);
  }
}

/******************************************************************************/
// SerialCommand handlers

void processInstruction(String instruction) {
   int instructionLength = instruction.length();
   if (instructionLength > 0) { command = instruction.substring(0,1); } 
   if (instructionLength > 1) {   value = instruction.substring(1,instructionLength); }
   //Serial.println(command);
   //Serial.println(value);

   if (command =='I') { // set integration Time
     integrationTime = value.toFloat();
     if ((integrationTime < 0.0) || (integrationTime > 9999.99)) { Serial.println("Integration time out of valid Range"); }
     c12880.set_integration_time(integrationTime);
     Serial.printf("Integratin time is: %g\n", integrationTime);
   } else if (command =='i') { // display integration time
     Serial.printf("Integration time is: %g\n", integrationTime);
     
   } else if (command == "f") { // turn off flash
     Serial.printf("Flash is off\n");
   } else if (command == "F") { // turn on flash
     int trig_micros = (int)(triggerTime*1e6);
     trig_micros = constrain(trig_micros,25,10000000);
     flashTimer.begin(flash_trigger, trig_micros);
     Serial.println("Flash is on\n");
    
   } else if (command == "b") { // verbose
     BINARY = false;
     Serial.printf("Verbose is on\n");
   } else if (command == "B") { // binary
     BINARY = true;
     Serial.println("Binary is on\n");
      
   } else if (command == "r") { // readout once
     continous = false;
     c12880.read(spectrum);
     transmitSpectrum(spectrum);
   } else if (command == "R") { // readout continous
     continous = true;
   } else { // invalid input, display status
     printHelp();
    
   } // end if else switch
}

void printHelp() {
  Serial.println("C12880 Controller");
  Serial.println("=================");
  Serial.println("Set Intergation time:   I0.01");
  Serial.println("Show Integration time:  i");
  Serial.println("Use/Dont use flash:     F/f");
  Serial.println("Use/Dont use binary:    B/b");
  Serial.println("Read one spectrum:      r");
  Serial.println("Read continous spectra: R");
}

void transmitSpectrum(uint16_t *data) {
  size_t i;
  if (BINARY) {
    for (i = 0; i < C12880_NUM_CHANNELS; i++) {
      Serial.write( (byte *) &data[i], sizeof(data[i]));
    }
  } else {
    for (i = 0; i < C12880_NUM_CHANNELS; i++) { 
      Serial.printf("%d, ",data[i]); 
    }
  }
  Serial.write('\n');
}

void flash_trigger(){
  digitalWrite(FLASH_TRIGGER,HIGH);
  flashTimer.end();
  digitalWrite(FLASH_TRIGGER,LOW);
}
