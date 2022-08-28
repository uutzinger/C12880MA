# C12880MA
C12880 mini spectrometer code for Teensy 3.x (Teensy 4.x is not supported)

This code reads Hamamatsu spectrometer using the spectrometer provided ADC trigger signal and DMA. 
The ADC clock is adjustable but the Teensy is limitted to about 800kS/s.
The Teensy ADC library (with external clock option) is needed (modified from original library: https://github.com/uutzinger/ADC).

The hardware can be purchased here https://groupgets.com/manufacturers/hamamatsu-photonics/products/c12880ma-micro-spectrometer

Urs Utzinger, Summer 2019, Fall 2020

Connections to several pins on Teensy will need to be made:

C12880MA  => Teensy 3.2

=========================================================================

SPEC_CLK => 23      // From Teensy to Spectrometer

SPEC_CLK => 21      // CLK cycles are counted with Teensy interrupt, pin 21 and 23 both need to be connected to SPEC_CLK

SPEC_TRG => 11      // From Spectrometer to Teensy, ADC signal trigger

SPEC_ST  => 12      // Sensor start pulse from Teensy

SPEC_VIDEO => 14    // Sensor signal, analog in, buffered, low impedance to Teensy

SPEC_EOS  => 13     // Sensor End of Scan, to Teensy

FLASH_TRIGGER => 18 // Light on/off from Teensy, will also work on other pins

Connect either 5V or 3.3V to Groupgets board, not both.

Do not place the Groupgets board onto your desk without first attaching insultation to the back of the PCB.
If the board is no longer working, likely the voltage regulator was shorted and needs to be replaced.
