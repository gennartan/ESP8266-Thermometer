#include <OneWire.h>
#include <ArduinoMqttClient.h>
#include <ESP8266WiFi.h>

#include "global_variables.h"

#define DEBUG_LEVEL 3
#define DEBUG_ERROR 1
#define DEBUG_MIN_INFO 2
#define DEBUG_MAX_INFO 3
#define DEBUG_OUT(level, fmt, ...) if(DEBUG_LEVEL>=level) Serial.printf_P( (PGM_P)PSTR(fmt), ## __VA_ARGS__ )

// Time between reads of temperature values (ms)
#define UPDATE_INTERVAL 10000

#define PIN 0

/*
  Network name and password
*/
char ssid[] = SSID;
char pass[] = PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Number of failed Wifi connections that lead to a reset
#define FAILED_WIFI_RESET 10
int failedWifi = 0;
int failedWifiConsecutive = 0;

const char broker[] = "192.168.1.35";
int        port     = 1883;
// TODO: generate topic with board or sensor unique ID
char topic[64];

OneWire ds(PIN);
byte addr[8];
char board_uid[32];

#define N 5
float temperatures[N];
int temp_id = 0;

int connectWifi(bool firstConnect)
{
	unsigned long startwifi=micros();
	unsigned int retry_count = 0;

	if(!firstConnect)
	{
		WiFi.reconnect();
	}
	else
	{
		WiFi.mode(WIFI_STA);
		//A static IP address could be handy if you want to use the webinterface.
		//To use a static IP address uncomment the line below and set the variables staticIP, dns, gateway, subnet
		//WiFi.config(staticIP, dns, gateway, subnet);
		WiFi.begin(ssid, pass); // Start WiFI
	}

	DEBUG_OUT(2, "%sonnecting to %s\n", firstConnect?"C":"Rec", ssid);

	while ((WiFi.status() != WL_CONNECTED) && (retry_count < 40))
	{
		delay(500);
		DEBUG_OUT(2, ".");
		retry_count++;
	}
	int success=WiFi.status();
	if(success==WL_CONNECTED)
	{
		DEBUG_OUT(2, "WiFi connected\nIP address: %s\n", WiFi.localIP().toString().c_str());
	}
	else
	{
		DEBUG_OUT(DEBUG_ERROR, "Failed to connect\n");
	}
	DEBUG_OUT(2, "Number of tries: %d\n", retry_count);
	DEBUG_OUT(2, "Connecting time (microseconds): %lu\n", micros()-startwifi);

	return success;
}

void setupAddressAndBoardUID()
{
	while (!ds.search(addr) )
	{
		Serial.println("No addresses found");
		delay(1000);
	}
	sprintf(board_uid, "%02X%02X%02X%02X%02X%02X%02X%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
	sprintf(topic, "%s/temperature", board_uid);
}
/*
   getTemp
   get temperature for a sensor connected to pin _pin_
*/
float getTemp()
{
	byte i;
	byte present;
	byte data[12];
	// TODO: automatically discover sensor address

	float celsius;

	if (OneWire::crc8(addr, 7) != addr[7])
	{
		Serial.println("CRC is not valid!");
		return 0;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44, 1);        // start conversion, with parasite power on at the end

	delay(1000);     // maybe 750ms is enough, maybe not
					 // we might do a ds.depower() here, but the reset will take care of it.

	present = ds.reset();
	ds.select(addr);
	ds.write(0xBE);         // Read Scratchpad

	for ( i = 0; i < 9; i++)
	{           // we need 9 bytes
		data[i] = ds.read();
	}
	OneWire::crc8(data, 8);


	// Convert the data to actual temperature
	// because the result is a 16 bit signed integer, it should
	// be stored to an "int16_t" type, which is always 16 bits
	// even when compiled on a 32 bit processor.
	int16_t raw = (data[1] << 8) | data[0];
	byte cfg = (data[4] & 0x60);
	// at lower res, the low bits are undefined, so let's zero them
	if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
	else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
	else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
										  //// default is 12 bit resolution, 750 ms conversion time
	celsius = (float)raw / 16.0;
	return celsius;
}

float getTempSmooth()
{
	float curr_temp = getTemp();
	temp_id = (temp_id + 1) % N;
	temperatures[temp_id] = curr_temp;

	float mean_temp = 0;
	for (int i = 0; i < N; i++) {
		mean_temp += temperatures[i];
	}

	mean_temp = mean_temp / N;
	return mean_temp;
}

void setupTemperatureVector()
{
	float temp = getTemp();
	for (int i = 0; i < N; i++) {
		temperatures[i] = temp;
	}
}

void setup()
{
	Serial.begin(115200);

	setupAddressAndBoardUID();
	setupTemperatureVector();

	connectWifi(true);
	mqttClient.connect(broker, port);
}

void loop()
{
	// put your main code here, to run repeatedly:

	static unsigned long previousMillis = 0;
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis >= UPDATE_INTERVAL) {
		previousMillis = currentMillis;

		float temperature = getTemp();
		mqttClient.beginMessage("test/topic");
		mqttClient.print(temperature);
		mqttClient.endMessage();
	}

	wdt_reset();
	yield();

	if(WiFi.status()!=WL_CONNECTED) { // Lost connection
		failedWifi++;
		failedWifiConsecutive++;
		connectWifi(false);
	}

	if (failedWifiConsecutive>FAILED_WIFI_RESET) {
		ESP.restart();
	}

	if(WiFi.status()==WL_CONNECTED) {
		failedWifiConsecutive=0;

		if (!mqttClient.connected()) {
			mqttClient.connect(broker, port);
		}
	}
}
