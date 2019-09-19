/*
   Connections:
   WeMos D1 Mini   Nokia 5110    Description
   (ESP8266)       PCD8544 LCD

   D2 (GPIO4)      0 RST         Output from ESP to reset display
   D1 (GPIO5)      1 CE          Output from ESP to chip select/enable display
   D6 (GPIO12)     2 DC          Output from display data/command to ESP
   D7 (GPIO13)     3 Din         Output from ESP SPI MOSI to display data input
   D5 (GPIO14)     4 Clk         Output from ESP SPI clock
   3V3             5 Vcc         3.3V from ESP to display
   D0 (GPIO16)     6 BL          3.3V to turn backlight on, or PWM
   G               7 Gnd         Ground

   Dependencies:
   https://github.com/adafruit/Adafruit-GFX-Library
   https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library
   - This pull request adds ESP8266 support:
   - https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library/pull/27
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <WiFiUdp.h>
#include "DHT.h"

// time
#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include <coredecls.h>                  // settimeofday_cb()

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <FS.h>
#include <EEPROM.h>
#include "credentials.h"

#include <JsonListener.h>//

//#include "fonts.h";
//#include <Ticker.h>
//#include "TimeClient.h"

// Pins
const int8_t RST_PIN = D2;
const int8_t CE_PIN = D1;
const int8_t DC_PIN = D6;
//const int8_t DIN_PIN = D7;  // Uncomment for Software SPI
//const int8_t CLK_PIN = D5;  // Uncomment for Software SPI
const int8_t BL_PIN = D0;
#define DHTPIN D3     // what digital pin we're connected to
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// Graph the RSSI of this WiFi
//const char* myssid = "myhome";
//const char* mypass = "noplay123";
int count = 0;
//long rssi;
//int8_t graph[83];
//uint8_t i, col, pos = 0;
//bool scroll = false;


// Set to false, if you prefere imperial/inches, Fahrenheit

// flag changed in the ticker function every 10 minutes
bool readyForWeatherUpdate = false;
float utcOffset = 0; // enter your UTC
unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead .
    I know but its not working :(
*/
IPAddress timeServerIP(128, 138, 141, 172); // time.nist.gov NTP server
//IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";

// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
const unsigned long seventyYears = 2208988800UL;
unsigned    second;
unsigned    minute;
unsigned    hour;
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

// Hardware SPI based on hardware controlled SCK (SCLK) and MOSI (DIN) pins. CS is still controlled by any IO pin.
// NOTE: MISO and SS will be set as an input and output respectively, so be careful sharing those pins!
Adafruit_PCD8544 display = Adafruit_PCD8544(DC_PIN, CE_PIN, RST_PIN);
int addr = 0;
byte value;
float avgTemp;
unsigned int tempCount;

void setup(void) {

  Serial.begin(115200);
  Serial.println("\nESP8266 ");
  Serial.println("WeMos D1 Mini + Nokia 5110 Weather\n");
  EEPROM.begin(512);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Turn LCD backlight on by setting pin LOW
  pinMode(BL_PIN, OUTPUT);
  analogWrite(BL_PIN, 25);
  //digitalWrite(BL_PIN, LOW);

  // Configure LCD
  display.begin();
  display.setContrast(60);  // Adjust for your display
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.clearDisplay();

  WiFi.begin(myssid, mypass);
  Serial.print("Connecting");
  display.print("Connecting");
  display.display();

  // Wait for successful connection
  while (count < 20 && WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    display.print(".");
    display.display();
    count++;
  }
  if ( count <= 20) {
    Serial.print("\nConnected to: ");
    Serial.println(myssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("");

    display.clearDisplay();
    display.println("Connected");

    display.println (WiFi.localIP().toString());
    display.display();

    Serial.println("Starting UDP");
    udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
  }
  delay(1000);
}


void loop(void) {

  display.clearDisplay();
  //get a random server from the pool
  //WiFi.hostByName(ntpServerName, timeServerIP);
  Serial.print( "ntpserver: " );
  Serial.println(  ntpServerName );
  Serial.print( "ntp ip: " );
  Serial.println(  timeServerIP );
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(5000);

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    int timeZone = 0;
    // print the hour, minute and second:
    second = epoch % 60;
    epoch /= 60;
    minute = epoch % 60;
    epoch /= 60;
    hour = ((epoch % 24) + timeZone) % 24;
  }
  display.printf("%02u:%02u:%02u UTC\n", hour, minute, second);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  delay(2000);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  //update count and sum of temps
  avgTemp += f;
  tempCount++;

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  //  Serial.print("Humidity: ");
  //  Serial.print(h);
  //  Serial.print(" %\t");
  //  Serial.print("Temperature: ");
  //  Serial.print(t);
  //  Serial.print(" *C ");
  //  Serial.print(f);
  //  Serial.print(" *F\t");
  //  Serial.print("Heat index: ");
  //  Serial.print(hic);
  //  Serial.print(" *C ");
  //  Serial.print(hif);
  //  Serial.println(" *F");
  display.printf("%4.2f *F\n", f);
  display.printf("%4.2f *F Heat\n", hif);
  display.printf("%4.2f  %% Hum\n", h);
  display.printf("%4.2f *F Avg\n", avgTemp / (float)tempCount);
  display.printf("%5d sec SSS\n", millis() / 1000); // total # of Seconds Since Start . (visual indication of time passed)

  //display.println("Norwalk,CT:");
 
  Serial.print("curr temp:");
  // Serial.println(wunderground.getCurrentTemp());
  // display.print(wunderground.getCurrentTemp() );
  //display.println (" *F");
  //getCurrentTemp());
  display.display();
  delay(5000);



}
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
