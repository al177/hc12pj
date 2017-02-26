HC12 Packet Junkie

Alternate firmware for HC12 serial RF modem modules.

The HC12 is an inexpensive 433MHz RF module found for under $5 on your
Chinese import site of choice.  Out of the box it communicates with other
HC12 modules over long distances (reportedly 1-2km) using a proprietary
radio protocol.  A simple command interface over a UART on the 5 pin
header is used to talk to the modem.  The radio IC is a Silicon Labs Si4463
which has excellent receive sensitivity, 20dBm (0.1W) transmit power, and
a very configurable FSK modem and packet processor.  An ST Micro STM8S003
microcontroller manages the radio and the serial host interface.

The anonymous designer of this PCB was kind enough to put test points on
the back of the HC12 PCB for the SWIM and RST pins of the microcontroller,
making it easy to replace the factory firmware with our own.

*** This is a work in progress! ***

Based on the STM8_template examples and library from Georg Icking-Konert
available at https://github.com/gicking/STM8_templates.git .


