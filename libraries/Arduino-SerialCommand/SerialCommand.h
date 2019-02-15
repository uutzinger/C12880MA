/**
 * SerialCommand - A Wiring/Arduino library to tokenize and parse commands
 * received over a serial port.
 * 
 * Copyright (C) 2012 Stefan Rado
 * Copyright (C) 2011 Steven Cogswell <steven.cogswell@gmail.com>
 *                    http://husks.wordpress.com
 * 
 * Version 20120522
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SerialCommand_h
#define SerialCommand_h

#if defined(WIRING) && WIRING >= 100
  #include <Wiring.h>
#elif defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <string.h>
#include <Stream.h>

// Size of the input buffer in bytes (maximum length of one command plus arguments)
#define SERIALCOMMAND_BUFFER 64

#define SERIALCOMMAND_MAXCOMMANDS_DEFAULT 10

// Uncomment the next line to run the library in debug mode (verbose messages)
//#define SERIALCOMMAND_DEBUG

#ifndef DEBUG_PORT
    #define DEBUG_PORT Serial
#endif

/******************************************************************************/
// SerialCommand (extends Print)
// so that callbacks print 
class SerialCommand : public Print {
  public:
    //structure to hold Command Info and callback function
    struct CommandInfo{
      const char *name;
      void (*function)(SerialCommand);
    };                                     
    
    SerialCommand(Stream &port,
                  int maxCommands = SERIALCOMMAND_MAXCOMMANDS_DEFAULT
                 );       // Constructor
    void addCommand(const char *command, void(*function)(SerialCommand));           // Add a command to the processing dictionary.
    void addCommand(__FlashStringHelper *command, void(*function)(SerialCommand));  // Add a command to the processing dictionary.
    void setDefaultHandler(void (*function)(SerialCommand));                        // A handler to call when no valid command received.

    int  readSerial();     // Fills the buffer with a command and returns it's length
    void processCommand(); // Performs command lookup, excution, and clears buffer
    void matchCommand();   // parses buffer and looks up first token as command
    void lookupCommandByName(char *name);  // attempts command lookup and sets _current_command
    void runCommand();    // executes the callback associate with _current_command
    void setBuffer(char *text_line); 
    void clearBuffer();   // Clears the input buffer.
    char *next();         // Returns pointer to next token found in command buffer (for getting arguments to commands).
    //provide method for printing
    size_t write(uint8_t val);
    
    //accessors
    CommandInfo getCurrentCommand() {return _current_command;}

  private:
    //Stream object for data IO
    Stream &_port;
    // Command/handler dictionary
    CommandInfo *_commandList;   // Actual definition for command/handler array
    CommandInfo _current_command; //command ready to dispatch
    CommandInfo _default_command; //called when a command name is not recognized
    int  _commandCount;
    int  _maxCommands;


    char _delim[2]; // null-terminated list of character to be used as delimeters for tokenizing (default " ")
    char _term;     // Character that signals end of command (default '\n')

    char _buffer[SERIALCOMMAND_BUFFER + 1]; // Buffer of stored characters while waiting for terminator character
    int  _bufPos;                        // Current position in the buffer
    char *_last;                         // State variable used by strtok_r during processing
};

#endif //SerialCommand_h
