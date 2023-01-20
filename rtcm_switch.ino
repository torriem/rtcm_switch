/* This sketch multiplexes two RTCM serial sources into one.
 * On ESP32,the sources are Serial1 (radio) and Bluetooth SPP.
 *
 * The ESP32 vesion defaults to passing Serial1 through to
 * Serial unless data is received on Bluetooth Serial. In
 * that case, only bluetooth data is passed to Serial.  After
 * a timeout of no data received, it switches back to Serial1.
 *
 * Speed is hard coded to 57600 because that's what I'm using
 * on the Trimble receivers and the radio network.
 */

#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
HardwareSerial GPS(1);
HardwareSerial Radio(2);

unsigned long last_bt_time;
bool use_bluetooth;

#define BT_TIMEOUT 3000 //3 seconds

#define RADIO_RX 25
#define RADIO_TX 26

#define TRIMBLE_RX 16
#define TRIMBLE_TX 17

#define TRAFFIC_LED 19

#define RADIO_SPEED 57600
#define GPS_SPEED 57600

void setup() {
	Serial.begin(115200);
	Radio.begin(57600,SERIAL_8N1,RADIO_RX,RADIO_TX); //radio
	GPS.begin(57600,SERIAL_8N1,TRIMBLE_RX,TRIMBLE_TX); //F9P or Trimble
	SerialBT.begin("TractorRTK"); //hard code name

	//Serial.println("Using Radio.");

	use_bluetooth = false;
	last_bt_time = millis();

	pinMode(18, OUTPUT);
}

void loop() {
	uint8_t c;
	while(1) {
		if (Radio.available()) {
			c = Radio.read();
			if (!use_bluetooth) {
				digitalWrite(TRAFFIC_LED,HIGH);
				GPS.write(c);
			}
		} else if (!use_bluetooth) {
			digitalWrite(TRAFFIC_LED,LOW);
		}

		if (GPS.available()) {
			//pass GGA data onto bluetooth for VRS NTRIP
			c = GPS.read();
			SerialBT.write(c);
		}

		if (SerialBT.available()) {
			//Bluetooth data available, switch to bluetooth
			c = SerialBT.read();
			//if (!use_bluetooth) {
			//	Serial.println ("Switching to Bluetooth.");
			//}

			use_bluetooth = true;
			last_bt_time = millis();
			GPS.write(c);
			
			digitalWrite(TRAFFIC_LED,HIGH);
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
