/* This sketch multiplexes two RTCM serial sources into one.
 * On ESP32,the sources are Radio (Serial2) and Bluetooth SPP.
 *
 * The ESP32 vesion defaults to passing Radio (Serial2) through 
 * to GPS (Serial1) unless data is received on Bluetooth Serial. 
 * In that case, only bluetooth data is passed to GPS (Serial1). 
 * After a timeout of no data received, it switches back to 
 * Radio (Serial2).
 *
 * Speed is usually set to 57600 because that's what I'm using
 * on the Trimble receivers and the radio network.
 *
 * Using pins 16 & 17 for the GPS uart (F9P uart2 or Trimble Port
 * C).  Pins 21 & 22 connect to the radio modem's uart.
 * Pin 5 is a button/switch to enable pass-through mode for
 * connecting XCTU to the Digi receiver via the RS232 port.
 */

#include "whichteensy.h"

#ifndef TEENSY
#include <BluetoothSerial.h>
#endif

//#define I2C

#ifdef I2C
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

SFE_UBLOX_GNSS myGNSS;

#endif

#ifdef TEENSY
#define SerialBT Serial
#define GPS Serial1
#define Radio Serial2
#else
//ESP32
BluetoothSerial SerialBT;
HardwareSerial GPS(1);
HardwareSerial Radio(2);
#endif

unsigned long last_bt_time;
bool use_bluetooth;

#define BT_TIMEOUT 3000 //3 seconds

#define RADIO_RX 21
#define RADIO_TX 22

#define GPS_RX 16
#define GPS_TX 17

#define PASSTHRU_PIN 5

#define TRAFFIC_LED 4

#define RADIO_SPEED 57600
#define GPS_SPEED 57600

#define BLUETOOTH_NAME "Tractor RTK"


/* Set serial_wait create a delay (in milliseconds)
   before data is passed through to the serial port, in
   order to wait for the monitor to boot up.  Set to 0
   if not needed. */
unsigned long serial_wait=0; //ms delay before relaying data

#ifdef I2C
char rtcm_buffer[1024];
int buf_len = 0;
int rtcm_state = 0;
int rtcm_length;

void onI2C_GGA(NMEA_GGA_data_t *nmeaData)
{
	//write out GGA sentence to the bluetooth
	SerialBT.print((const char *)nmeaData->nmea); //includes CRLF
}

void process_rtcm_byte(char b) {
	//rtcm parsing state machine

	switch (rtcm_state) {
	case 0:
		//look for beginning of rtcm packet
		if (b != 0xd3)
			return;
		else {
			//we've got the beginning of a new packet now
			buf_len = 0;
			rtcm_buffer[buf_len++] = b;
			rtcm_state = 1;
		}
		break;
	case 1:
		//looking for length of message
		if (buf_len < 2) {
			rtcm_buffer[buf_len++] = b;
		}

		if (buf_len == 2) {
			//Serial.print("message length is: ");
			//Serial.println(rtcm_length);
			rtcm_state = 2;
			rtcm_length = rtcm_buffer[1] << 8 + rtcm_buffer[2];
			rtcm_length &= 0x03ff; //isolate 10 least significant bits
		}
		break;
	case 2: 
		//look for message type
		if (buf_len < 5) {
			rtcm_buffer[buf_len++] = b;
		}
		if (buf_len == 5) {
			//don't really care about type, but here it is if we want it
			int type =  (rtcm_buffer[3] << 8) + rtcm_buffer[4];
			type >>= 4; //isolate 12 most significant bits
#ifdef TEENSY
			//Serial.print("Got message type: ");
			//Serial.println(type);
#endif
			rtcm_state = 3;
		}
		break;
	case 3:
		//get the message body
		if (buf_len < (rtcm_length + 5) && buf_len < 1023) {
			rtcm_buffer[buf_len++] = b;
		}
		if (buf_len == 1023) {
			//buffer overflow, restart
			rtcm_state = 0;
			return;
		}
		if (buf_len == (rtcm_length + 5)) {
			//we now have a full message including the CRC byte
			//transmit the entire packet to GNSS
			myGNSS.pushRawData(((uint8_t *)rtcm_buffer), buf_len, false);
			rtcm_state = 0;
		}
		break;
	default:
		rtcm_state = 0; //should never happen
	}
}


#endif

void setup() {
	unsigned long start_time;

	Serial.begin(115200);
#ifdef TEENSY
	SerialBT.begin(9600); //fake BT on usb serial for testing
	GPS.begin(GPS_SPEED);
	Radio.begin(RADIO_SPEED);
#else
	Radio.begin(RADIO_SPEED,SERIAL_8N1,RADIO_RX,RADIO_TX); //radio
	GPS.begin(GPS_SPEED,SERIAL_8N1,GPS_RX,GPS_TX); //F9P or Trimble
	SerialBT.begin(BLUETOOTH_NAME); //hard code name
#endif

	Serial.println();
	Serial.println("RTCM switcher.");

	use_bluetooth = false;
	last_bt_time = millis();

	pinMode(TRAFFIC_LED, OUTPUT);
	pinMode(PASSTHRU_PIN, INPUT_PULLUP);

#       ifdef I2C
	Wire.begin();

	if (myGNSS.begin() == false) {
		Serial.println(F("Cannot detect F9P. Halting."));
		while (1)
			digitalWrite(TRAFFIC_LED, HIGH);
			delay(2000);
			digitalWrite(TRAFFIC_LED, LOW);
			delay(500);
	}

	//enable NMEA output on I2C
	myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);

	//enable RTCM input on I2C
	myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_RTCM3);

	//set it to output 1Hz only
	myGNSS.setNavigationFrequency(1);
	// Several of these are on by default so let's disable them

	// Use cfgValset to disable / enable individual NMEA messages
	myGNSS.newCfgValset(VAL_LAYER_RAM_BBR); 
	myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C, 0); 
	myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 0);
	myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 0);
	myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C, 0);
	myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C, 0);
	// Leave only GGA enabled at current navigation rate
	myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 1); 

	if (myGNSS.sendCfgValset()) // Send the configuration VALSET
		Serial.println(F("NMEA messages were configured successfully"));
	else
		Serial.println(F("NMEA message configuration failed!"));

	myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_DEFAULT); 
	myGNSS.setHighPrecisionMode(true);

	// Set up the callback for GNGGA.  data will
	// automatically be transfered to bluetooth by
	// this callback

	myGNSS.setNMEAGNGGAcallbackPtr(&onI2C_GGA);


#endif

	if (serial_wait > 0) {
		start_time = millis();

		while ( (millis() - start_time) < serial_wait) {
			//consume the bytes so buffers don't overflow
			if (Radio.available())
				Radio.read();
			if (SerialBT.available())
				SerialBT.read();
		} 
	}
}

void loop() {
	uint8_t c;
	bool pass_through = false;

	while(1) {
#ifdef I2C
		myGNSS.checkUblox();
		myGNSS.checkCallbacks();
#endif

		if (!digitalRead(PASSTHRU_PIN)) {
			if (!pass_through) {
				Serial.println("switching to pass through");
			}
			pass_through = true;
			use_bluetooth = false;
		} else {
			if (pass_through) {
				Serial.println("back to normal mode.");
			}
			pass_through = false;
		}

		if (Radio.available()) {
			c = Radio.read();
			if (pass_through || !use_bluetooth) {
				digitalWrite(TRAFFIC_LED,HIGH);
				GPS.write(c);
#ifdef I2C
				process_rtcm_byte(c);
#endif
			}
		} else if (!use_bluetooth) {
			digitalWrite(TRAFFIC_LED,LOW);
		}

		if (GPS.available()) {
			//pass GGA data onto bluetooth for VRS NTRIP
			c = GPS.read();
			if (pass_through) {
				Radio.write(c);
			} else {
				SerialBT.write(c);
			}
		}

		if (SerialBT.available()) {
			//Bluetooth data available, switch to bluetooth
			c = SerialBT.read();
			//if (!use_bluetooth) {
			//	Serial.println ("Switching to Bluetooth.");
			//}

			if (!pass_through) {
				use_bluetooth = true;
				last_bt_time = millis();
				GPS.write(c);
#ifdef I2C
				process_rtcm_byte(c);
#endif

				digitalWrite(TRAFFIC_LED,HIGH);
			}
		} else {
			if ((millis() - last_bt_time) > BT_TIMEOUT) {
				//haven't seen any bt data in a while
				//switch back to radio.

				//if (use_bluetooth) {
				//	Serial.println("Switching back to radio.");
				//}
				use_bluetooth = false;
			}
			if (use_bluetooth) {
				digitalWrite(TRAFFIC_LED,LOW);
			}
		}
	}
}
