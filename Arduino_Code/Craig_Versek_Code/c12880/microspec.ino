#include <SerialCommand.h>   /* http://github.com/p-v-o-s/Arduino-SerialCommand */

#if defined(CORE_TEENSY)
#include <ADC.h> /* https://github.com/pedvide/ADC */
#else
#include <elapsedMillis.h>
#endif

#include "c12880.h"

#define SPEC_TRG         A0
#define SPEC_ST          A1
#define SPEC_CLK         10
#define SPEC_VIDEO       A3

#if defined(CORE_TEENSY)
#define FLASH_TRIGGER    12
IntervalTimer flashTimer;
#endif

uint16_t data[C12880_NUM_CHANNELS];
C12880_Class spec(SPEC_TRG,SPEC_ST,SPEC_CLK,SPEC_VIDEO);

SerialCommand sCmd(Serial);// the SerialCommand parser object
/******************************************************************************/
void setup(){
  #if defined(CORE_TEENSY)
  pinMode(FLASH_TRIGGER,OUTPUT);
  digitalWrite(FLASH_TRIGGER,LOW);
  #endif
  Serial.begin(115200); // Baud Rate set to 115200
  // Setup callbacks for SerialCommand commands
  sCmd.setDefaultHandler(UNRECOGNIZED_sCmd_default_handler);
  sCmd.addCommand("SPEC.INTEG", SPEC_INTEG_sCmd_config_handler);     //configures integration time
  sCmd.addCommand("SPEC.READ?", SPEC_READ_sCmd_query_handler);       //reads out the whole spectrum
  sCmd.addCommand("SPEC.TIMING?", SPEC_TIMING_sCmd_query_handler);   //reads out timings
  #if defined(CORE_TEENSY)
  sCmd.addCommand("FLASH.TIMER_ONCE!", FLASH_TIMER_ONCE_sCmd_action_handler); //set flash to trigger on microsecond delay
  #endif
  spec.begin();
 
}
/******************************************************************************/
void loop(){
  int num_bytes = sCmd.readSerial();      // fill the buffer
  if (num_bytes > 0){
    sCmd.processCommand();  // process the command
  }
  delay(10);
}

/******************************************************************************/
// SerialCommand handlers

// Unrecognized command
void UNRECOGNIZED_sCmd_default_handler(SerialCommand this_sCmd)
{
  SerialCommand::CommandInfo command = this_sCmd.getCurrentCommand();
  this_sCmd.print(F("### Error: command '"));
  this_sCmd.print(command.name);
  this_sCmd.print(F("' not recognized ###\n"));
}

void SPEC_INTEG_sCmd_config_handler(SerialCommand this_sCmd){
  char *arg = this_sCmd.next();
  float integ_time;
  if (arg == NULL){
    this_sCmd.print(F("### Error: SPEC.INTEG requires 1 argument 'time' (s)\n"));
  }
  else{
    integ_time = atof(arg);
    spec.set_integration_time(integ_time);
  }
}

void SPEC_READ_sCmd_query_handler(SerialCommand this_sCmd){
  spec.read_into(data);
  for (int i = 0; i < C12880_NUM_CHANNELS - 1; i++){
    this_sCmd.print(data[i]);
    this_sCmd.print(',');
  }
  //last value gets special formatting
  this_sCmd.print(data[C12880_NUM_CHANNELS - 1]);
  this_sCmd.print("\n");
}

void SPEC_TIMING_sCmd_query_handler(SerialCommand this_sCmd){
  for(int i=0; i < 9; i++){
    this_sCmd.print(spec.get_timing(i));
    this_sCmd.print(",");
  }
  this_sCmd.print(spec.get_timing(9));
  this_sCmd.print("\n");
}

#if defined(CORE_TEENSY)
void FLASH_TIMER_ONCE_sCmd_action_handler(SerialCommand this_sCmd){
  char *arg = this_sCmd.next();
  float trig_time;
  if (arg == NULL){
    this_sCmd.print(F("### Error: FLASH.TIMER_ONCE! requires 1 argument 'time' (s)\n"));
  }
  else{
    trig_time = atof(arg);
    trig_time = max(25e-6,trig_time);
    int trig_micros = (int)(trig_time*1e6);
    trig_micros = constrain(trig_micros,25,10000000);
    flashTimer.begin(flash_trigger, trig_micros);
  }
}

void flash_trigger(){
  digitalWrite(FLASH_TRIGGER,HIGH);
  flashTimer.end();
  digitalWrite(FLASH_TRIGGER,LOW);
}
#endif
