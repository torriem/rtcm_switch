My roving RTK GPS receives usually receive RTCMv3 messages over a radio modem.  However sometimes I wish to use NTRIP over a cell phone instead. This sketch allows a ESP32 to automatically switch between receiving RTCM over bluetooth and over a serial radio modem.

By default the ESP32 takes all data coming in HardwareSerial port 2 (pins 21 and 22), connected to the radio, and passes it to HardwareSerial port 1 (pins 16 and 17), connected to the GPS receiver, through an RS232 converter.

If a bluetooth connection is established and RTCM data received over bluetooth, the sketch will stop passing data from the radio on HardwareSerial 2 and instead pass data from bluetooth.  If after three seconds there is no bluetooth data received, the sketch will switch back to receiving data from the radio.

