# home-theatre-remote
Arduino based remote control for home theatre devices, i.e. Projector, Amplifier

1st attempt uses RS-232 connections to the A/V Amplifier and Projector
had problems with getting this to work

9 Feb 2021
2nd attempt uses Infrared Receiver/Transmitter

2 step process
Step 1. build a setup to receive the existing remote control IR signals
  store the hex codes for all buttons that are required, e.g. 
  Projector ON
  Projector OK
  Amplifier Power
  Amplifier Input Select - Apple TV...

Step 2. Final device to send IR commands from button presses/web server commands

https://create.arduino.cc/projecthub/electropeak/use-an-ir-remote-transmitter-and-receiver-with-arduino-1e6bc8

IR Receiver
https://www.jaycar.co.nz/arduino-compatible-infrared-receiver-module/p/XC4427
$5.20

IR Transmitter
https://www.jaycar.co.nz/arduino-compatible-infrared-transmitter-module/p/XC4426
$5.90
