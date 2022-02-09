//#define DEBUG

#include <OneWire.h>
#include <ArduinoMqttClient.h>
#include <ESP8266WiFi.h>

byte pin = 0;

char ssid[] = "";
char pass[] = "";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "192.168.1.35";
int        port     = 1883;
const char topic[]  = "arduino/simple";

const unsigned long interval = 1000*5;

float getTemp(byte pin)
{
  OneWire ds(0);
  byte i;
  byte present;
  byte data[12];
  byte addr[] = {40, 253, 8, 197, 22, 33, 1, 96};
  float celsius;

  if (OneWire::crc8(addr, 7) != addr[7]) {
    #ifdef DEBUG
      Serial.println("CRC is not valid!");
    #endif
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

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
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



void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    #ifdef DEBUG
    Serial.print(".");
    #endif
    delay(5000);
  }

  if (!mqttClient.connect(broker, port)) {
    #ifdef DEBUG
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    #endif
    while (1);
  }


}

void loop() {
  // put your main code here, to run repeatedly:

  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float temp = getTemp(pin);
    mqttClient.beginMessage(topic);
    mqttClient.print(temp);
    mqttClient.endMessage();
  }
  delay(5000);
}
