#include "Arduino.h"
#include "Average.h"
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/*IoThingsWare - Multiple DS18B20 Temperature Sensors on 1 wire
  Connections:
  DS18B20 Pinout (Left to Right, pins down, flat side toward you)
  - Left   = Ground
  - Center = Signal (Pin 2):  (with 3.3K to 4.7K resistor to +5 or 3.3 )
  - Right  = +5 or +3.3 V

   Questions: terry@yourduino.com
   V1.01  01/17/2013 ...based on examples from Rik Kretzinger

  /*-----( Import needed libraries )-----*/
// Get 1-wire Library here: http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <OneWire.h>

//Get DallasTemperature Library here:  http://milesburton.com/Main_Page?title=Dallas_Temperature_Control_Library
#include <DallasTemperature.h> //you must include library DallasTemperature by Miles Burton, ...

/*-----( Declare Constants and Pin Numbers )-----*/
#define ONE_WIRE_BUS_PIN  14

/*-----( Declare objects )-----*/
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS_PIN); //modified

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

struct {
  DeviceAddress bin;
  char hex[16];
  Adafruit_MQTT_Publish mqttobj= Adafruit_MQTT_Publish((Adafruit_MQTT *)0,"");
  float window[0x10];
} SensorID[20];

byte nSensorsDiscovered;

#include "mdns.h"  //you must include library: esp8266_mdns by mrdunk

/*

#include <DHT.h>

#define DHTTYPE DHT11
#define DHTPIN  4



char temperatureString[6];



DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266
float humidity, temp_f;  // Values read from sensor
*/

char wifi_ssid[] = "IoThingsWareBus";    //  your network SSID (name)
char wifi_password[] = "07B04U1957S";   // your network password
/************************* Adafruit.io Setup *********************************/

//#define AIO_SERVER      "192.168.16.127"
#define AIO_SERVER      "raspberrypi.local"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""
char serveraddress[] = "xxx.xxx.xxx.xxx";
/************ Global State (you don't need to change this!) ******************/



int status = WL_IDLE_STATUS;
WiFiClient espClient;
//PubSubClient client(espClient);
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&espClient, serveraddress, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temperature_surrounding = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/Sensor/SurroundingTemperature");
Adafruit_MQTT_Publish humidity_surrounding = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/Sensor/SurroundingHumidity");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton_obj = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/Actuator/onoff");

/*************************** Sketch Code ************************************/
static int mqttServerDiscovered;

//#define humidity_topic "/Sensor/humidity"

//#define temperature_topic "/Sensor/temperature"
// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).

void MQTT_connect();

// When an mDNS packet gets parsed this optional callback gets called once per Query.
// See mdns.h for definition of mdns::Answer.
void answerCallback(const mdns::Answer* answer) {
  if (!strcmp(answer->name_buffer, AIO_SERVER) && answer->rrtype == MDNS_TYPE_A)
  {
    Serial.println(answer->rdata_buffer);
    strcpy(serveraddress, answer->rdata_buffer);
    answer->Display();
    /*
      Serial.println(mqtt.servername);
      mqtt.servername=serveraddress;
      Serial.println(mqtt.servername);
      Adafruit_MQTT_Client mqtt(&espClient, serveraddress, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
    */
    // Setup MQTT subscription for onoff feed.
    mqtt.subscribe(&onoffbutton_obj);
    mqttServerDiscovered = true;
  }
}

// Initialise MDns.
// If you don't want the optional callbacks, just provide a NULL pointer as the callback.
//mdns::MDns my_mdns(packetCallback, queryCallback, answerCallback);
mdns::MDns my_mdns(NULL, NULL, answerCallback);


void discoverOneWireDevices(void) {
  byte i, index = 0;
  byte present = 0;
  byte data[12];
  byte addr[8];
  String hex;
  Serial.print("Looking for 1-Wire devices...\n\r");
  oneWire.reset();
  delay(1000);
  oneWire.reset_search();
  while (oneWire.search(addr)) {
    hex = "";
    Serial.print("\n\rFound \'1-Wire\' device with address:\n\r");
    for ( i = 0; i < 8; i++) {
      Serial.print("0x");
      SensorID[index].bin[i] = addr[i];
      if (addr[i] < 16) {
        hex += '0';
        Serial.print('0');
      }
      hex += String(addr[i], HEX).c_str();
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    if ( oneWire.crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      return;
    }
    strcpy(SensorID[index].hex, hex.c_str());
    index++;
  }
  Serial.print("\n\r\n\rThat's it.\r\n");
  oneWire.reset_search();
  nSensorsDiscovered = index;
  return;
}




void setup() {
  mqttServerDiscovered = false;
  nSensorsDiscovered = 0;
  Serial.begin(115200);
  delay(10);
  // Initialize the Temperature measurement library
//  sensors.begin();
//  delay(1000);

  discoverOneWireDevices();
  Serial.println(nSensorsDiscovered);
  // set the resolution to 10 bit (Can be 9 to 12 bits .. lower is faster)
  for (int i = 0; i < nSensorsDiscovered; i++)
  {
     sensors.setResolution(SensorID[i].bin, 10);
  }


/*
  dht.begin();           // initialize temperature sensor
*/
  setup_wifi();
  // Query for all host information for a paticular name. ("raspberrypi.local" in this case.)
  my_mdns.Clear();
  struct mdns::Query query_server;
  strncpy(query_server.qname_buffer, AIO_SERVER, MAX_MDNS_NAME_LEN);
  query_server.qtype = MDNS_TYPE_A;
  query_server.qclass = 1;    // "INternet"
  query_server.unicast_response = 0;
  my_mdns.AddQuery(query_server);
  my_mdns.Send();
  // Discover Sensor on 1Wire Bus
  Serial.println();
  Serial.print("Number of Devices found on bus = ");
  Serial.println(sensors.getDeviceCount());
  Serial.print("Reading sensors... ");
  Serial.println();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(2000);  // wait 2 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      ESP.restart();  //manually reset after serial flashing, the latest work around
    }
  }
  Serial.println("MQTT Connected!");
}

void getValue(byte i)
{
  float tempC = sensors.getTempC(SensorID[i].bin);
  if (tempC == -127.00)
  {
    Serial.print("Error getting temperature  ");
  }
  else
  {
    Serial.print("C: ");
    Serial.println(tempC);
    //   Serial.print(" F: ");
    //   Serial.print(DallasTemperature::toFahrenheit(tempC));
    Adafruit_MQTT_Publish obj = Adafruit_MQTT_Publish(&mqtt, (const char *)SensorID[i].hex);
//    Adafruit_MQTT_Publish obj = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME SensorID[i].hex);
//    Adafruit_MQTT_Publish obj = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "jghj");
    if (! obj.publish(String(tempC).c_str())) {
      Serial.println(F("Failed"));
      ESP.restart();  //manually reset after serial flashing, the latest work around
    } else {
      Serial.println(F("OK!"));
    }
  }
}



void readSensors() {
  if(!nSensorsDiscovered) {
  discoverOneWireDevices();
  } else {
  // Command all devices on bus to read temperature
  sensors.requestTemperatures();
  for (int i = 0; i < nSensorsDiscovered; i++)
  {
   getValue(i);
  }
  }
}




void loop() {
  if (!mqttServerDiscovered)
  {
    my_mdns.Check();
    return;
  }
  // get sensors falue and write on channel 168044
  MQTT_connect();
  /*
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton_obj) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton_obj.lastread);
    }
  }
  */
  
//  getDHTSensorsValue();
  readSensors();
  delay(2000); // Note that the weather station only updates once a minute
}

/*
void getDHTSensorsValue() {
  humidity = dht.readHumidity();          // Read humidity (percent)
  temp_f = dht.readTemperature(true);     // Read temperature as Fahrenheit
  // Check if any reads failed and exit early (to try again).

  if (isnan(humidity) || isnan(temp_f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("Current temperature is: ");
  Serial.print((temp_f - 32) * 5 / 9);
  Serial.println(" °C");
  Serial.print("Current humidity is: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print(F("temperature publisher "));
  if (! temperature_surrounding.publish(String((temp_f - 32) * 5 / 9).c_str())) {
    Serial.println(F("Failed"));
    mqtt.disconnect();
    return;

  } else {
    Serial.println(F("OK!"));
  }
  Serial.print(F("humidity publisher "));
  if (! humidity_surrounding.publish(String(humidity).c_str())) {
    Serial.println(F("Failed"));
    mqtt.disconnect();
    return;
  } else {
    Serial.println(F("OK!"));
  }
}
*/
