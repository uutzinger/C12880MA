# C12880MA
C12880 mini spectrometer code for teensy.
This code has not been tested yet. Expected by Fall 2019.

Code for reading of Hamamatsu spectrometer with teensy 3.x microcontroller.
It uses externally triggered ADC and DMA. The clock is adjustable.
This needs modified ADC library with external triggering functions (provided and tested).

The hardware can be purchased here
https://groupgets.com/manufacturers/hamamatsu-photonics/products/c12880ma-micro-spectrometer

Connections to several pins on Teensy will need to be made according the instructions in the header of the program.
There is no additional hardware required.
