// Demo Code for SerialCommand Library
// Craig Versek, Jan 2014
// based on code from Steven Cogswell, May 2011

#include <SerialCommand.h>

#define arduinoLED 13   // Arduino LED on board

SerialCommand sCmd(Serial);         // The demo SerialCommand object, initialize with any Stream object

void setup() {
  pinMode(arduinoLED, OUTPUT);      // Configure the onboard LED for output
  digitalWrite(arduinoLED, LOW);    // default to LED off

  Serial.begin(9600);

  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("ON",    LED_on);          // Turns LED on
  sCmd.addCommand("OFF",   LED_off);         // Turns LED off
  sCmd.addCommand("HELLO", sayHello);        // Echos the string argument back
  sCmd.addCommand("P",     processCommand);  // Converts two arguments to integers and echos them back
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
  Serial.println("Ready");
}

void loop() {
  int num_bytes = sCmd.readSerial();      // fill the buffer
  if (num_bytes > 0){
    sCmd.processCommand();  // process the command
  }
  delay(10);
}


void LED_on(SerialCommand this_sCmd) {
  this_sCmd.println("LED on");
  digitalWrite(arduinoLED, HIGH);
}

void LED_off(SerialCommand this_sCmd) {
  this_sCmd.println("LED off");
  digitalWrite(arduinoLED, LOW);
}

void sayHello(SerialCommand this_sCmd) {
  char *arg;
  arg = this_sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    this_sCmd.print("Hello ");
    this_sCmd.println(arg);
  }
  else {
    this_sCmd.println("Hello, whoever you are");
  }
}


void processCommand(SerialCommand this_sCmd) {
  int aNumber;
  char *arg;

  this_sCmd.println("We're in processCommand");
  arg = this_sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);    // Converts a char string to an integer
    this_sCmd.print("First argument was: ");
    this_sCmd.println(aNumber);
  }
  else {
    this_sCmd.println("No arguments");
  }

  arg = this_sCmd.next();
  if (arg != NULL) {
    aNumber = atol(arg);
    this_sCmd.print("Second argument was: ");
    this_sCmd.println(aNumber);
  }
  else {
    this_sCmd.println("No second argument");
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(SerialCommand this_sCmd) {
  SerialCommand::CommandInfo command = this_sCmd.getCurrentCommand();
  this_sCmd.print("Did not recognize \"");
  this_sCmd.print(command.name);
  this_sCmd.println("\" as a command.");
}
