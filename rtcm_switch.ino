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

#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
HardwareSerial GPS(1);
HardwareSerial Radio(2);

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

void setup() {
	unsigned long start_time;

	Serial.begin(115200);
	Radio.begin(RADIO_SPEED,SERIAL_8N1,RADIO_RX,RADIO_TX); //radio
	GPS.begin(GPS_SPEED,SERIAL_8N1,GPS_RX,GPS_TX); //F9P or Trimble
	SerialBT.begin(BLUETOOTH_NAME); //hard code name

	Serial.println();
	Serial.println("RTCM switcher.");

	use_bluetooth = false;
	last_bt_time = millis();

	pinMode(TRAFFIC_LED, OUTPUT);
	pinMode(PASSTHRU_PIN, INPUT_PULLUP);

	if (serial_wait > 0) {
		start_time = millis();

		while ( (millis() - start_time) > serial_wait) {
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
