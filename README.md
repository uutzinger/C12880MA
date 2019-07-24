# C12880MA
C12880 mini spectrometer code for Teensy 3.x

This code reads Hamamatsu spectrometer using the spectrometer provided ADC trigger signal and DMA. 
The ADC clock is adjustable but the Teensy is limitted to about 800kS/s.
The ADC library for Teensy is needed with exterbal triggering functions integrated (provided).

The hardware can be purchased here https://groupgets.com/manufacturers/hamamatsu-photonics/products/c12880ma-micro-spectrometer


Urs Utzinger, Summer 2019

Connections to several pins on Teensy will need to be made:

C12880MA  => Teensy 3.2

=========================================================================

SPEC_CLK => 23 // From Teensy to Spetrometer

SPEC_CLK => 21 // CLK cycles are counted with teensy interrupt, pin 21 and 23 both need to be connected to SPEC_CLK

SPEC_TRG => 11 // From Spectrometer to Teensy, ADC signal trigger

SPEC_ST  => 12 // Sensor start pulse from microcontroller

SPEC_VIDEO => 14 // Sensor signal, analog in, buffered, low impedance to microcontroller

SPEC_EOS  => 13 // Sensor End of Scan, to microcontroller

FLASH_TRIGGER => 18 // Light on/off from microcontroller will also work on other pins
