/*

Release new firmware version:
1. Increase REV value in this .ino
2. Arduino IDE 1.8.19 menu: Sketch/Export compiled Binary
   (HARDWARE ESP32-POE; the sketch-local partitions.csv is applied automatically)
3. generate all .bin and publish to GitHub Pages: $ ./tools/all.sh --publish
4. git commit with the Release number as comment and push

Partial update (without a release):
1. Arduino IDE 1.8.19 menu: Sketch/Export compiled Binary (HARDWARE ESP32-POE)
2. build SPIFFS image: $ tools/build_spiffs_image.sh
3. upload firmware.bin and build/spiffs.bin via EasyOTA at http://[ip]:82/update
4. git commit and push

Firmware download page: https://ok1hra.github.io/3D-print-WX-station/
One-time setup: GitHub repo Settings -> Pages -> Branch: gh-pages, folder /(root)

3D printed WX station
----------------------
https://remoteqth.com/w/doku.php?id=3d_print_wx_station
TNX OK1IAK for code help

 ___               _        ___ _____ _  _
| _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
|   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
|_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Send test packet
  echo -n -e '\x00ms:bro;' | nc -u -w1 192.168.1.23 88 | hexdump -C
Remote USB access
  screen /dev/ttyUSB0 115200


mosquitto_sub -h 54.38.157.134 -p 1883 -t "OK1HRA-8/WX/FreeHeap"



HARDWARE ESP32-POE

Changelog:
20260626 - windy.com Station ID ('I' menu cmd, EEPROM 373-404) + WINDY link on main web page next to APRS
20260626 - finish windy.com upload (API key in EEPROM 244-367 + 'K' menu cmd, fix units, local TLS client + setInsecure to save heap)
20260626 - AI bugfix
20231012 - mDNS support
20230917 - windy.com (not work, probably low memory)
20230916 - fix key login, http temperature
20230825 - add dew point, fix shift register, fix MQTT topic
20230815 - recalibrate rain
20221104 - calibrate rain
20220910 - HW rev5, DS18B20 autodetect,
20220429 - add html preview page on port 88
20220307 - add wind direction shift settings for non North orientation, update for new lib
20210513 - add enable support, RF module support by https://github.com/jarodan
20210407 - disable local CLI
20210322 - get actual data on request, via MQTT topic /get (any message)
20210321 - used inaccurate internal temperature sensor, if external DS18B20 disable
20210316 - web firmware upload
20210225 - eeprom bugfix
20210131 - add to menu erase windspeed max memory, fix max speed bug, disable internal temperature sensor
20210123 - calculate pressure with altitude TNX OK1IRG, add altitude settings in CLI
20210122 - bugfix DS18B20 eeprom set
20210116 - WDT bug fix, command listing after telnet login
20201211 - external sensor enable from CLI
20200815 - js url fix
20200814 - add WatchdogTimer for reset
20200620 - addd frenetic mode, calibrate rain sensor
20200509 - first function for calibrate wind sensor
20200424 - add support for external 1-wire temperature ensor (DS18B20)

ToDo
- web setup
- web selfcheck
- Windy
  https://community.windy.com/topic/8168/report-your-weather-station-data-to-windy
  https://github.com/zpukr/esp8266-WindStation/blob/master/esp8266-WindStation.ino
- Sunset https://github.com/buelowp/sunset/blob/master/examples/esp/example.ino
- telnet inactivity to close
- clear code
- https://github.com/Sensirion/arduino-sht
APRS-lora
https://how2electronics.com/esp32-lora-thingspeak-gateway-sensor-node/
https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
https://github.com/lora-aprs/LoRa_APRS_Tracker/blob/master/src/configuration.cpp#L73
https://github.com/josefmtd/lora-aprs
https://on5vl.org/lora-aprs-le-guide-pratique/


Použití knihovny WiFi ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/WiFi
Použití knihovny EEPROM ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/EEPROM
Použití knihovny Ethernet ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/Ethernet
Použití knihovny ESPmDNS ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/ESPmDNS
Použití knihovny ArduinoOTA ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/ArduinoOTA
Použití knihovny Update ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/Update
Použití knihovny AsyncTCP ve verzi 1.1.1 v adresáři: /home/dan/Arduino/libraries/AsyncTCP
Použití knihovny ESPAsyncWebServer ve verzi 1.2.3 v adresáři: /home/dan/Arduino/libraries/ESPAsyncWebServer
Použití knihovny FS ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/FS
Použití knihovny AsyncElegantOTA ve verzi 2.2.5 v adresáři: /home/dan/Arduino/libraries/AsyncElegantOTA
Použití knihovny PubSubClient ve verzi 2.8 v adresáři: /home/dan/Arduino/libraries/PubSubClient
Použití knihovny Wire ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/Wire
Použití knihovny Adafruit_Unified_Sensor ve verzi 1.1.2 v adresáři: /home/dan/Arduino/libraries/Adafruit_Unified_Sensor
Použití knihovny Adafruit_BMP280_Library ve verzi 2.5.0 v adresáři: /home/dan/Arduino/libraries/Adafruit_BMP280_Library
Použití knihovny Adafruit_BusIO ve verzi 1.9.8 v adresáři: /home/dan/Arduino/libraries/Adafruit_BusIO
Použití knihovny SPI ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/SPI
Použití knihovny Adafruit_HTU21DF_Library ve verzi 1.0.4 v adresáři: /home/dan/Arduino/libraries/Adafruit_HTU21DF_Library
Použití knihovny SD_MMC ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/SD_MMC
Použití knihovny OneWire ve verzi 2.3.6 v adresáři: /home/dan/Arduino/libraries/OneWire
Použití knihovny DallasTemperature ve verzi 3.9.0 v adresáři: /home/dan/Arduino/libraries/DallasTemperature

*/
//-------------------------------------------------------------------------------------------------------
const char* REV = "20260626";
#define HWREVsw 8                   // software PCB version [7-8]
// #define AJAX                        // enable ajax web server
#define WINDY                      // upload to windy.com PWS (set station API key with 'K' menu command). TLS client is local + setInsecure() to save heap
// #define M_DNS                        // not work, but ArduinoOTA.setHostname() rewrite hostname
#define OTAWEB                      // enable upload firmware via web
#define DS18B20                     // external 1wire Temperature sensor
#define BMP280                      // pressure I2C sensor
#define HTU21D                      // humidity I2C sensor
// #define SHT21                       // humidity I2C sensor
// #define SHT                         // general SHT sensors I2C sensor https://github.com/Sensirion/arduino-sht/blob/master/examples/multiple-sht-sensors/multiple-sht-sensors.ino
// #define RF69_EXTERNAL_SENSOR        // RX temp and humidity radio sensor RF69
#define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
#define ETH_ADDR 0
#define ETH_TYPE ETH_PHY_LAN8720
#if HWREVsw==8
  #define ETH_POWER 0                // #define ETH_PHY_POWER 0 ./Arduino/hardware/espressif/esp32/variants/esp32-poe/pins_arduino.h
#endif
#if HWREVsw==7
  #define ETH_POWER 12                // mosfet on VDDIO
#endif
#define ETH_MDC 23                  // MDC pin17
#define ETH_MDIO 18                 // MDIO pin16
#define ETH_CLK ETH_CLOCK_GPIO17_OUT    // CLKIN pin5 | settings for ESP32 GATEWAY rev f-g
String MACString;
char MACchar[18];
// #define ETH_CLK ETH_CLOCK_GPIO0_OUT    // settings for ESP32 GATEWAY rev c and older
// ETH.begin(ETH_ADDR, ETH_POWER, ETH_MDC, ETH_MDIO, ETH_TYPE, ETH_CLK);
// #define WIFI                     // Enable ESP32 WIFI (DHCP IPv4) - NOT TESTED
const char* ssid     = "";
const char* password = "";
const float FunelDiaInCM = 10.0; // cm funnel diameter
unsigned char HWREVpcb = 0;          // PCB version must be compatible with HWREVsw
//-------------------------------------------------------------------------------------------------------
// unsigned long TimerTemp;

// interrupts
#include "esp_attr.h"

// values
const int keyNumber = 1;
char key[100];
String YOUR_CALL = "";
/*
1mm rain = 15,7cm^2/10 = 1,57ml     <- by rain funnel radius
10ml = 11,5 pulses = 0,87ml/pulse   <- constanta tilting measuring cup
*/
// float mmInPulse = 0.87/(3.14*(FunelDiaInCM/2)*(FunelDiaInCM/2)/10); // calculate rain mm, in one pulse
// float mmInPulse = 0.2 ; // callibration rain 17,7-20,2 mm with 95 pulse
float mmInPulse = 0; // 0.65 ; // measure 3.8mm, reference 12.2mm

long MeasureTimer[2]={2800000,300000};   //  millis,timer (5 min)
long RainTimer[2]={0,5000};   //  <---------------- rain timing
int RainCount;
String RainCountDayOfMonth;
bool RainStatus;
int WindDir = 0;
int WindDirShift = 0;
float RainTodayMM = 0;
float WindSpeedAvgMPS = 0;
float WindSpeedMaxPeriodMPS = 0;

float PressureHPA = 0;
float TemperatureCelsius = 0;
float HumidityRelPercent = 0;
float DewPointCelsius = 0;

  float PressureHPaBMP280 = 0;
  float TemperatureCelsiusBMP280 = 0;
  float HumidityRelPercentHTU21D = 0;
  float DewPointCelsiusHTU21D = 0;
  float TemperatureCelsiusDS18B20 = 0;

volatile long RpmTimer[2]={0,3000};
volatile long RpmPulse = 987654321;
volatile long PeriodMinRpmPulse = 987654321;
String WindSpeedMaxPeriodUTC;
long MinRpmPulse;
String MinRpmPulseTimestamp;
// unsigned int RpmSpeed = 0;
volatile unsigned long RpmAverage[2]={0,0};  // counter,sum time
volatile bool RpmInterrupt = false;
volatile bool GustEvent = false;    // ISR flagged a new period-max (gust); loop timestamps + persists it
volatile long RollBucketMin = 987654321;   // shortest pulse since WindTask last folded it into a bucket
// All RPM state above is accumulated directly inside the IRAM-safe RPMcount() ISR and shared with
// loop() (core 1) + WindTask (core 0). rpmMux serialises access; use *_ISR variants inside the ISR.
portMUX_TYPE rpmMux = portMUX_INITIALIZER_UNLOCKED;
// Rolling "last 5 min" wind max for the dashboard, kept by WindTask in 30s buckets. Deliberately
// SEPARATE from WindSpeedMaxPeriodMPS (the publish-period max sent to MQTT/Windy/APRS), so the web
// shows a true trailing-5-min peak independent of the 5-min publish reset.
float WindRollMaxMps = 0;
String WindRollMaxUtc = "";
int InterruptDepth = 0;             // nesting counter for Interrupts(false/true)
float TempCal=0;

int SpeedAlertLimit_ms = 0;
int SpeedAlert_ms = 3000;
// bool NeedSpeedAlert_ms = false;
long AlertTimer[2]={0,60000};
//  |alert...........|alert........... everry max 1 minutes and with publish max value in period

// 73 seconds WDT (WatchDogTimer)
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 73
long WdtTimer=0;

// Software watchdog - recovers from failures the HW task-WDT cannot catch
// (network stack death / socket- or heap-exhaustion while loop() keeps running and feeding the WDT).
#define CONN_LOST_REBOOT_MS 900000UL   // 15 min without ETH connectivity -> restart (MQTT may be absent on an island LAN)
#define HEAP_MIN_FREE 20000            // bytes; if free heap drops below this -> restart
unsigned long ConnOkTimer=0;           // millis() of last healthy ETH state

byte InputByte[21];
// #define Ser2net                  // Serial to ip proxy - DISABLE if board revision 0.3 or lower
#define EnableOTA                // Enable flashing ESP32 Over The Air
// int NumberOfEncoderOutputs = 8;  // 2-16
int EnableSerialDebug     = 0;
long FreneticModeTimer ;
#define HTTP_SERVER_PORT  80     // Web server port

const int SERIAL_BAUDRATE = 115200; // serial debug baudrate
int SERIAL1_BAUDRATE; // serial1 to IP baudrate
int incomingByte = 0;   // for incoming serial data

int i = 0;
#include <WiFi.h>
// mDNS
#if defined(M_DNS) && defined(ETHERNET)
  #include <ESPmDNS.h>
#endif
#include "EEPROM.h"
#define EEPROM_SIZE 470   /* up to 512
0|Byte    1|128
1|Char    1|A
2|UChar   1|255
3|Short   2|-32768
5|UShort  2|65535
7|Int     4|-2147483648
11|Uint    4|4294967295
15|Long    4|-2147483648
19|Ulong   4|4294967295
23|Long64  8|0x00FFFF8000FF4180
31|Ulong64 8|0x00FFFF8000FF4180
39|Float   4|1234.1234
43|Double  8|123456789.12345679
51|Bool    1|1

0    - (free, was: listen source)
1    - (free, was: net ID)
2-3  - TempCal Short
4    - HWREVpcb UChar
5    - mmInPulse Short

-13
14-17 - SERIAL1_BAUDRATE
18-21 - SerialServerIPport
22-25 - (free, was: IncomingSwitchUdpPort)
26-29 - (free, was: RebootWatchdog)
30-33 - (free, was: OutputWatchdog)
34    - (free, was: Bank0 storage)
35    - (free, was: Bank1 storage)
36    - (free, was: Bank2 storage)
37-40 - Authorised telnet client IP
41-140 - Authorised telnet client key
141-160 - YOUR_CALL
161-164 - MQTT broker IP
165-168? - MQTT broker port 4
169-172  - MinRpmPulse 4
173-198  - MinRpmPulseTimestamp
199   - APRS ON/OFF;
200-203 - APRS server IP
204-207 - APRS server port 4
208-212 - APRS password
213-230 - APRS coordinate
231-234 - Altitude 4
// 232 - SpeedAlert 4
235-238 - SpeedAlert 4
// 233 - DS18B20 on/off 1
239 - DS18B20 on/off 1
240-243 - WindDirShift 4
244-367 - 123 char API key windy.com
368   - RainCount day-of-month (byte, 1-31; 255=unset)
369-372 - RainCount (int 4) - persisted so a reboot does not lose today's rain
373-404 - 32 char windy.com public Station ID (e.g. pws-f06ea43a) for the web-page link
405-468 - 64 char web Basic-auth password (0xff/0x00 terminated, empty = auth OFF)
469     - WebAuthEnabled (1=on, set automatically when password is non-empty)

!! Increment EEPROM_SIZE #define !!

*/
int Altitude = 0;
bool needEEPROMcommit = false;

#if defined(WINDY)
  // #include <HTTPClient.h>
  #include <WiFiClientSecure.h>
  const char*  windyServer = "stations.windy.com";  // Server URL
  String WindyApiKey = "";                           // windy.com PWS station API key, stored in EEPROM 244-367 (empty = upload disabled)
  String WindyStationId = "";                         // windy.com public Station ID, e.g. "pws-f06ea43a" (EEPROM 373-404), only for the web-page link
  // The WiFiClientSecure (TLS) object is created locally inside GetHttpsWindy() so its
  // ~30-40 kB of handshake buffers are released after every upload instead of being held
  // permanently in RAM (the original global object was the main cause of "low memory").

  // #define WINDY_VERIFY_CERT     // verify server certificate chain; needs several kB more heap during handshake.
                                    // Default OFF (client.setInsecure()) to fit the limited ESP32 heap - weather telemetry is not security sensitive.
  #if defined(WINDY_VERIFY_CERT)
  // ISRG Root X1 root .pem certificate for windy.com valid to Mon, 04 Jun 2035 11:04:38 GMT
  const char* rootCACertificate = \
  "-----BEGIN CERTIFICATE-----\n" \
  "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n" \
  "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
  "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n" \
  "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n" \
  "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n" \
  "MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n" \
  "h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n" \
  "0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n" \
  "A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n" \
  "T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n" \
  "B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n" \
  "B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n" \
  "KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n" \
  "OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n" \
  "jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n" \
  "qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n" \
  "rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n" \
  "HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n" \
  "hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n" \
  "ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n" \
  "3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n" \
  "NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n" \
  "ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n" \
  "TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n" \
  "jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n" \
  "oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n" \
  "4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n" \
  "mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n" \
  "emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n" \
  "-----END CERTIFICATE-----\n";
  #endif
#endif

#include <WebServer.h>
WebServer webserver(HTTP_SERVER_PORT);   // :80 - new web UI: root dashboard, /setup, /api/*, static SPIFFS files
bool DHCP_ENABLE = 1;
// Client variables
char linebuf[80];
int charcount=0;
//Are we currently connected?
boolean connected = false;
uint8_t buffer[50] = "";
#include <ETH.h>
static bool eth_connected = false;
String HTTP_req;
#if defined(EnableOTA)
  #include <ESPmDNS.h>
  #include <ArduinoOTA.h>
#endif
#if defined(OTAWEB)
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include "src/AsyncElegantOTA_IPR/AsyncElegantOTA_IPR.h"   // vendored OTA: firmware + filesystem (SPIFFS) upload
  AsyncWebServer OTAserver(82);
  // Server-Sent Events stream for the dashboard's live wind needle. Pushed from WindTask (see below),
  // so the gauge tracks gusts in real time instead of polling. Runs on the existing async server.
  AsyncEventSource events("/events");
  #define WEB_AUTH_USER "admin"          // fixed Basic-auth user for OTA + /setup writes
  bool WebAuthEnabled = false;           // enabled when WebAuthPassword is set (phase 5)
  String WebAuthPassword = "";           // stored in EEPROM (phase 5); empty = auth OFF
#endif

#define MQTT               // Enable MQTT debug
#if defined(MQTT)
  #include <PubSubClient.h>
  // #include "PubSubClient.h" // lokalni verze s upravou #define MQTT_MAX_PACKET_SIZE 128
  // WiFiClient esp32Client;
  // PubSubClient mqttClient(esp32Client);
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);
  // PubSubClient mqttClient(ethClient);
   // PubSubClient mqttClient(server, 1883, callback, ethClient);
   long lastMqttReconnectAttempt = 0;
#endif
boolean MQTT_ENABLE     = 1;          // enable public to MQTT broker
IPAddress mqtt_server_ip(0, 0, 0, 0);
// byte BrokerIpArray[2][4]{
//   // {192,168,1,200},   // MQTT broker remoteqth.com
//   {54,38,157,134},   // MQTT broker remoteqth.com
// };
// IPAddress server(10, 24, 213, 92);    // MQTT broker
int MQTT_PORT;       // MQTT broker PORT
// int MQTT_PORT_Array[2] = {
//   1883,
//   1883
// };       // MQTT broker PORT
boolean MQTT_LOGIN      = 0;          // enable MQTT broker login
// char MQTT_USER= 'login';    // MQTT broker user login
// char MQTT_PASS= 'passwd';   // MQTT broker password
const int MqttBuferSize = 1000; // 1000
char mqttTX[MqttBuferSize];
char mqttPath[MqttBuferSize];
// char mqttTX[100];
// char mqttPath[100];
long MqttStatusTimer[2]{1500,1000};
long HeartBeatTimer[2]={0,1000};

// WX pinout
const int ShiftInDataPin = 34;  // to rev0.3 13
const int ShiftInLatchPin = 4;
const int ShiftInClockPin = 5;
bool rxShiftInRead;

// https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
#include <Wire.h>
#define I2C_SDA 33
#define I2C_SCL 32

#if defined(BMP280)||defined(HTU21D)||defined(SHT21)
  // #include <SPI.h>
  #include <Adafruit_Sensor.h>
  TwoWire I2Cone = TwoWire(0);
#endif

#if defined(BMP280)
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp(&I2Cone); // use I2C interface
  Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
  bool BMP280enable;
#endif

#if defined(HTU21D)
  #include "Adafruit_HTU21DF.h"
  Adafruit_HTU21DF htu = Adafruit_HTU21DF();
  bool HTU21Denable;
#endif

#if defined(SHT21)
  #include "SHT2x.h"
  SHT2x internal;
  // SHT2x external;
#endif

#if defined(SHT)
  #include "SHTSensor.h"
  // Sensor with normal i2c address
  // Sensor 1 with address pin pulled to GND
  SHTSensor sht1(SHTSensor::SHT3X);
  // Sensor with alternative i2c address
  // Sensor 2 with address pin pulled to Vdd
  // SHTSensor sht2(SHTSensor::SHT3X_ALT);
#endif

// https://github.com/PaulStoffregen/RadioHead
#if defined(RF69_EXTERNAL_SENSOR)
  bool RF69enable;
  #include <SPI.h>
  #include <RH_RF69.h>
  // Change to 434.0 or other frequency, must match RX's freq!
  #define RF69_FREQ 434.0
  #define RFM69_RST     -1   // same as LED
  #define RFM69_CS      0   // "B"
  #define RFM69_INT     16   // "A"

  // Singleton instance of the radio driver
  RH_RF69 rf69(RFM69_CS, RFM69_INT);

  int16_t packetnum = 0;  // packet counter, we increment per xmission
  String received_data;
  float humidity, temp_f, temp_f2, temp_PT100, temp_dallas;
  String temp_radio;
  String humidity_radio;
  String vbat_radio;

  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
#endif

const int RpmPin = 39;
#if HWREVsw==8
  const int RainPin = 36;
#endif
#if HWREVsw==7
  const int Rain1Pin = 36;
  const int Rain2Pin = 35;
#endif
// const int EnablePin = 13;
// const int ButtonPin = 34;

#if defined(Ser2net)
  #define RX1 3
  #define TX1 1
  HardwareSerial Serial_one(1);
#endif

const int MappingRow = 5;
const long mapping[MappingRow][2] = { // ms > m/s
  {987654321,0},
  {120,1},
  {50,2},
  {2,4},
  {1,200},
};
// WX end

// SD
// #define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// #define ETH_PHY_POWER 12
// #include "FS.h"
// #include "SD_MMC.h"
#include "SPIFFS.h"                  // web pages live here (gzipped), uploaded via OTA filesystem image
#include "esp_partition.h"           // read the on-flash SPIFFS partition geometry (size for the OTA image)
bool FsMounted = false;             // true after a successful SPIFFS.begin()

// ntp
#include "time.h"
const char* ntpServer = "pool.ntp.org";
// const char* ntpServer = "tik.cesnet.cz";
// const char* ntpServer = "time.google.com";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

#define MAX_SRV_CLIENTS 1
int SerialServerIPport;
// WiFiServer SerialServer(SerialServerIPport);
WiFiServer SerialServer;
WiFiClient SerialServerClients[MAX_SRV_CLIENTS];

int TelnetServerIPport = 23;
WiFiServer TelnetServer;
WiFiClient TelnetServerClients[MAX_SRV_CLIENTS];
IPAddress TelnetServerClientAuth;
bool TelnetAuthorized = false;
int TelnetAuthStep=0;
int TelnetAuthStepFails=0;
int TelnetLoginFails=0;
long TelnetLoginFailsBanTimer[2]={0,600000};
int RandomNumber;
bool FirstListCommands=true;

int CompareInt;

// APRS
WiFiClient AprsClient;
boolean AprsON = false;
uint16_t AprsPort;
IPAddress aprs_server_ip(0, 0, 0, 0);
String AprsPassword;
String AprsCoordinates;

// DS18B20
#if defined(DS18B20)
  // #include <OneWire.h>
  // #include <DallasTemperature.h>
  // // const int DsPin = 3;
  // // OneWire ds(DsPin);
  // // DallasTemperature sensors(&ds);
  // const int oneWireBus = 13;
  // OneWire oneWire(oneWireBus);
  // DallasTemperature sensors(&oneWire);
  bool ExtTemp = true;
  bool ExtTempValid = false;   // last DS18B20 read succeeded; kept across fast cache refreshes

  // Include the libraries we need
  #include <OneWire.h>
  #include <DallasTemperature.h>
  // Data wire is plugged into port 2 on the Arduino
  #if HWREVsw==8
    #define ONE_WIRE_BUS 2
  #endif
  #if HWREVsw==7
    #define ONE_WIRE_BUS 13
  #endif
  #define TEMPERATURE_PRECISION 10 // 9: ±0,5°C | 10: ±0,25°C | 11: ±0,125°C
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS);
  // Pass our oneWire reference to Dallas Temperature.
  DallasTemperature sensors(&oneWire);
  // arrays to hold device addresses
  DeviceAddress insideThermometer, outsideThermometer;
  // forward declarations (Arduino auto-prototype skips functions using DeviceAddress)
  void printAddress(DeviceAddress deviceAddress);
  void printTemperature(DeviceAddress deviceAddress);
  void printResolution(DeviceAddress deviceAddress);
  void printData(DeviceAddress deviceAddress);
  // Assign address manually. The addresses below will need to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // DeviceAddress insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  // DeviceAddress outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };
#endif

//-------------------------------------------------------------------------------------------------------

// ----- Forward declarations -------------------------------------------------
// Arduino IDE 1.8.x (arduino-builder + ctags 5.8) fails to auto-generate
// function prototypes for this sketch (the prototype preprocessing aborts on
// the very long line in AsyncElegantOTA's elegantWebpage.h), so every function
// called before its definition would error "was not declared in this scope".
// Declaring them explicitly makes the build independent of auto-prototyping.
void IRAM_ATTR RPMcount();
void Interrupts(boolean ON);
void WindTask(void *pv);            // dedicated wind-speed cache refresh, pinned to core 0
void EEPROMcommit();
void Watchdog();
void RainCountStore();
void RainCountRestore();
void GetValue(bool readSlow = true);   // readSlow=false: fast web-cache refresh (skip DS18B20, no accumulator reset)
unsigned long LiveCacheTimer = 0;      // drives the frequent dashboard cache refresh
unsigned long WindDirTimer = 0;        // drives the 1s wind-direction refresh (kept fresh for SSE push)
const unsigned long LiveCacheInterval = 30000;  // ms between slow-sensor cache refreshes (temp/hum/press,
                                                // ~2x/min). Wind dir+speed update far faster via /api/wind.
void MqttPubValue();
void check_radio();
void GetHttpsWindy();
double Babinet(double Pz, double T);
int MpsToMs(int MPS);
float PulseToMetterBySecond(long PULSE);
long AvgRpmPulse();
float WindNowMps();                 // instantaneous wind speed, decays to 0 after >2s without a pulse
byte Azimuth();
void AzShift(int AZ);
void AprsWxIgate();
String LeadingZero(int NumberOfZero, int NR);
byte HiByte(int ID);
byte LowByte(int ID);
void CLI();
void Enter();
void EnterChar(int OUT);
void EnterInt(int OUT);
void EnterIntOld(int OUT);
void Prn(int OUT, int LN, String STR);
void ListCommands(int OUT);
float RainPulseToMM(int PULSE);
char RandomChar();
String getValue(String data, char separator, int index);
void EthEvent(WiFiEvent_t event);
void Mqtt();
bool mqttReconnect();
void MqttRx(char *topic, byte *payload, unsigned int length);
void AfterMQTTconnect();
void MqttPubString(String TOPIC, String DATA, bool RETAIN);
void testClient(const char * host, uint16_t port);
void TelnetAuth();
void AuthQ(int NR, bool BAD);
void Telnet();
void SerialToIp();
void I2cScanner();
String UtcTime(int format);
String Timestamp();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);
void handleRoot();                       // GET / - meteo dashboard (SPIFFS index.html, else PROGMEM fallback)
void handleStatic();                     // serve any other SPIFFS file (gzip-aware), else 404
void handleApiLive();                    // GET /api/live - cached meteo data as JSON (root polls this)
void handleApiWind();                    // GET /api/wind - tiny fast-poll endpoint: live dir + speed
void handleApiWallCfg();                 // GET /api/wallcfg - broker ws URI + topic for the MQTT wall page
void handleSetupPage();                  // GET /setup - the settings page (auth-gated so the browser prompts)
void handleApiStatus();                  // GET /api/status - full sensor + raw debug dump for Status tab
void handleApiDebug();                   // GET /api/debug - debug level + recent debug lines (ring buffer)
void handleApiConfig();                  // GET /api/config - all /setup settings + diagnostics (secrets masked)
void handleApiConfigSave();              // POST /api/config - apply+persist settings, reboot if a network field changed
String jsonEsc(const String& s);
void eepromWriteString(int start, int maxlen, const String& s);
bool webAuthOK();                        // Basic-auth gate for write/secret endpoints (when enabled)
String webContentType(const String& path);
bool streamSpiffsFile(const String& path);
// ----------------------------------------------------------------------------

void setup() {
  // for (int i = 0; i < 8; i++) {
  //   pinMode(TestPin[i], INPUT);
  // }
  #if HWREVsw==8
    pinMode(RainPin, INPUT);
  #endif
  #if HWREVsw==7
    pinMode(Rain1Pin, INPUT);
    pinMode(Rain2Pin, INPUT);
  #endif

  pinMode(RpmPin, INPUT);
  // pinMode(EnablePin, OUTPUT);
  // digitalWrite(EnablePin,1);

  // pinMode(ButtonPin, INPUT);
  // SHIFT IN
  pinMode(ShiftInLatchPin, OUTPUT);
    digitalWrite(ShiftInLatchPin,1);
  pinMode(ShiftInClockPin, OUTPUT);
  pinMode(ShiftInDataPin, INPUT);

  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  randomSeed(esp_random());   // seed PRNG from hardware RNG (telnet auth challenge)

  #if defined(DS18B20)
    sensors.begin();

    // locate devices on the bus
    Serial.print("DS18B20 found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    // report parasite power requirements
    //  Serial.print("Parasite power is: ");
    //  if (sensors.isParasitePowerMode()) Serial.println("ON");
    //  else Serial.println("OFF");

    // Search for devices on the bus and assign based on an index. Ideally,
    // you would do this to initially discover addresses on the bus and then
    // use those addresses and manually assign them (see above) once you know
    // the devices on your bus (and assuming they don't change).
    //
    // method 1: by index
    if (!sensors.getAddress(insideThermometer, 0)){
      ExtTemp = false;
      Serial.println("DS18B20 unable to find address for Device 0");
    }
    //  if (!sensors.getAddress(outsideThermometer, 1)) Serial.println("Unable to find address for Device 1");

    // method 2: search()
    // search() looks for the next device. Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are no devices,
    // or you have already retrieved all of them. It might be a good idea to
    // check the CRC to make sure you didn't get garbage. The order is
    // deterministic. You will always get the same devices in the same order
    //
    // Must be called before search()
    //oneWire.reset_search();
    // assigns the first address found to insideThermometer
    //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
    // assigns the seconds address found to outsideThermometer
    //if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

    // show the addresses we found on the bus
    Serial.print("DS18B20 device 0 Address: ");
    printAddress(insideThermometer);
    Serial.println();

    //  Serial.print("Device 1 Address: ");
    //  printAddress(outsideThermometer);
    //  Serial.println();

    // set the resolution to 9 bit per device
    sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
    //  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

    Serial.print("DS18B20 device 0 Resolution: ");
    Serial.print(sensors.getResolution(insideThermometer), DEC);
    Serial.println();

    //  Serial.print("Device 1 Resolution: ");
    //  Serial.print(sensors.getResolution(outsideThermometer), DEC);
    //  Serial.println();
  #endif

  // single shared I2C bus (TwoWire port 0) for all on-board I2C sensors
  #if defined(BMP280)||defined(HTU21D)||defined(SHT21)
    // I2Cone.begin(0x76, I2C_SDA, I2C_SCL, 100000); // SDA pin, SCL pin, 100kHz frequency
    I2Cone.begin(I2C_SDA, I2C_SCL, (uint32_t)100000); // SDA pin, SCL pin, 100kHz frequency
  #endif

  #if defined(BMP280)
    Serial.print("BMP280 sensor init ");
    if(!bmp.begin(0x76)){
      Serial.println("failed!");
      // while (1) delay(10);
      BMP280enable=false;
    }else{
      Serial.println("OK");
      BMP280enable=true;
      /* Default settings from datasheet. */
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        bmp_temp->printSensorDetails();
    }
  #endif

  #if defined(SHT)
    Wire.begin(I2C_SDA, I2C_SCL);
  #endif
  #if defined(HTU21D)
    Serial.print("HTU21D sensor init ");
    if(!htu.begin(&I2Cone)){   // share the same bus instance as BMP280
      Serial.println("failed!");
      // while (1);
      HTU21Denable=false;
    }else{
      Serial.println("OK");
      HTU21Denable=true;
    }
  #endif

  #if defined(SHT21)
    Serial.println(__FILE__);
    Serial.print("SHT2x_LIB_VERSION: \t");
    Serial.println(SHT2x_LIB_VERSION);

    internal.begin(&I2Cone);
    // external.begin(&I2Ctwo);

    uint8_t stat = internal.getStatus();
    Serial.print(stat, HEX);
    Serial.println();
    // stat = external.getStatus();
    // Serial.print(stat, HEX);
    // Serial.println();
    Serial.println();
  #endif

  #if !defined(BMP280) && !defined(HTU21D)
    Wire.begin(I2C_SDA, I2C_SCL);
  #endif

  // // SD
  // if(!SD_MMC.begin()){
  //   Serial.println("SD card Mount Failed");
  //   // return;
  // }

  // Listen source
  if (!EEPROM.begin(EEPROM_SIZE)){
    if(EnableSerialDebug>0){
      Serial.println("failed to initialise EEPROM"); delay(1);
    }
  }
  // 2-3 TempCal Short
  if(EEPROM.readByte(2)!=255){
    TempCal = (float)EEPROM.readShort(2)/100.0;
  }else{
    TempCal = 0;
  }

  // 4 HWREVpcb UChar
  if(EEPROM.readByte(4)!=255){
    HWREVpcb = EEPROM.readUChar(4);
  }

  // 5 mmInPulse Short
  if(EEPROM.readByte(5)!=255){
    mmInPulse = (float)EEPROM.readShort(5)/100.0;
  }else{
    mmInPulse = 0.13;
  }

  SERIAL1_BAUDRATE=EEPROM.readInt(14);
  SerialServerIPport=EEPROM.readInt(18);
  TelnetServerClientAuth[0]=EEPROM.readByte(37);
  TelnetServerClientAuth[1]=EEPROM.readByte(38);
  TelnetServerClientAuth[2]=EEPROM.readByte(39);
  TelnetServerClientAuth[3]=EEPROM.readByte(40);

  // 41-140 key
  // if clear, generate
  if(EEPROM.readByte(41)==255 && EEPROM.readByte(140)==255){
    Serial.println();
    Serial.println("  ** GENERATE KEY **");
    for(int i=41; i<141; i++){
      EEPROM.writeChar(i, RandomChar());
      Serial.print("*");
    }
    EEPROM.commit();
    Serial.println();
  }
  // read
  for(int i=41; i<141; i++){
    key[i-41] = EEPROM.readChar(i);
  }

  // YOUR_CALL
  // move after ETH init

  // MQTT broker IP
  for(int i=0; i<4; i++){
    mqtt_server_ip[i]=EEPROM.readByte(i+161);
  }
  MQTT_PORT = EEPROM.readInt(165);
  if(mqtt_server_ip[0]==255 && mqtt_server_ip[1]==255 && mqtt_server_ip[2]==255 && mqtt_server_ip[3]==255 && MQTT_PORT==-1){
    mqtt_server_ip[0]=54;
    mqtt_server_ip[1]=38;
    mqtt_server_ip[2]=157;
    mqtt_server_ip[3]=134;
    MQTT_PORT=1883;
  }

  // RPM
  MinRpmPulse = EEPROM.readLong(169);
  if(MinRpmPulse<=0){
    MinRpmPulse=987654321;
  }

  // 173
  if(EEPROM.readByte(173)!=255){
    MinRpmPulseTimestamp = EEPROM.readString(173);
  }

  // 199   - APRS ON/OFF;
  if(EEPROM.read(199)<2){
    AprsON=EEPROM.read(199);
  }

  // 200-203 - APRS server IP
  // 204-207 - APRS server port
  for(int i=0; i<4; i++){
    aprs_server_ip[i]=EEPROM.readByte(i+200);
  }
  AprsPort = EEPROM.readInt(204);

  // 208-212 - APRS password
  for (int i=208; i<213; i++){
    if(EEPROM.read(i)!=0xff){
      AprsPassword=AprsPassword+char(EEPROM.read(i));
    }
  }

  // 213-230 - APRS coordinate
  for (int i=213; i<231; i++){
    if(EEPROM.read(i)!=0xff){
      AprsCoordinates=AprsCoordinates+char(EEPROM.read(i));
    }
  }

  // 231-234 Altitude
  if(EEPROM.readByte(231)!=255){
    Altitude = EEPROM.readInt(231);
  }

  // 235-238 AlertLimit
  if(EEPROM.readByte(235)!=255){
    SpeedAlertLimit_ms = EEPROM.readInt(235);
  }

  #if defined(DS18B20)
    // 239   - ExtTemp ON/OFF;
    if(EEPROM.read(239)<2){
      ExtTemp=EEPROM.read(239);
    }
  #endif

  // 240-243 WindDirShift
  if(EEPROM.readByte(240)!=255){
    WindDirShift = EEPROM.readInt(240);
  }

  // 244-367 - windy.com API key (123 chars max, 0xff terminated/unset)
  // 373-404 - windy.com public Station ID (32 chars max, 0xff terminated/unset)
  #if defined(WINDY)
    WindyApiKey="";
    for (int i=244; i<367; i++){
      byte b = EEPROM.read(i);
      if(b==0xff || b==0x00){
        break;
      }
      WindyApiKey += char(b);
    }
    WindyStationId="";
    for (int i=373; i<405; i++){
      byte b = EEPROM.read(i);
      if(b==0xff || b==0x00){
        break;
      }
      WindyStationId += char(b);
    }
  #endif

  // 405-468 web Basic-auth password, 469 enabled flag
  #if defined(OTAWEB)
    WebAuthPassword="";
    for (int i=405; i<469; i++){
      byte b = EEPROM.read(i);
      if(b==0xff || b==0x00){ break; }
      WebAuthPassword += char(b);
    }
    WebAuthEnabled = (WebAuthPassword.length() > 0);   // auth follows whether a password is set
  #endif

  #if defined(WIFI)
    if(EnableSerialDebug>0){
      Serial.print("WIFI Connecting to ");
      Serial.print(ssid);
    }
    WiFi.begin(ssid, password);
    // attempt to connect to Wifi network:
    while(WiFi.status() != WL_CONNECTED) {
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      delay(500);
      if(EnableSerialDebug>0){
        Serial.print(".");
      }
    }
    // LED1status = !LED1status;
    // digitalWrite(LED1, LED1status);           // signalize wifi connected
    if(EnableSerialDebug>0){
      Serial.println("");
      Serial.println("WIFI connected");
      Serial.print("WIFI IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("WIFI dBm: ");
      Serial.println(WiFi.RSSI());
    }
  #endif

  #if defined(ETHERNET)
    // mqtt_server_ip=BrokerIpArray[0];
    // MQTT_PORT = MQTT_PORT_Array[0];

    WiFi.onEvent(EthEvent);
    // ETH.begin();
    ETH.begin(ETH_ADDR, ETH_POWER, ETH_MDC, ETH_MDIO, ETH_TYPE, ETH_CLK);
    if(DHCP_ENABLE==false){
      ETH.config(IPAddress(192, 168, 1, 188), IPAddress(192, 168, 1, 255),IPAddress(255, 255, 255, 0),IPAddress(8, 8, 8, 8));
      //config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = (uint32_t)0x00000000, IPAddress dns2 = (uint32_t)0x00000000);
    }
    // mDNS
    #if defined(M_DNS)
        // Set up mDNS responder:
        // - first argument is the domain name, in this example
        //   the fully-qualified domain name is "esp32.local"
        // - second argument is the IP address to advertise
        //   we send our IP address on the WiFi network
      if (MDNS.begin("wx")) {
        Serial.println("mDNS server run");
        MDNS.addService("http", "tcp", 80);
      } else {
        Serial.println("Error start mDNS server");
      }
    #endif

  #endif
    // chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    //   unsigned long long1 = (unsigned long)((chipid & 0xFFFF0000) >> 16 );
    //   unsigned long long2 = (unsigned long)((chipid & 0x0000FFFF));
    //   ChipidHex = String(long1, HEX) + String(long2, HEX); // six octets
    //   YOUR_CALL=ChipidHex;

  #if defined(EnableOTA)
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);
    // Hostname defaults to esp3232-[MAC]

    // String StringHostname = "WX-station-"+String(NET_ID, HEX);
    String StringHostname = "WX-"+String(YOUR_CALL);
    char copy[13];
    StringHostname.toCharArray(copy, 13);

    ArduinoOTA.setHostname(copy);
    ArduinoOTA.setPassword("remoteqth");
    // $ echo password | md5sum
    // ArduinoOTA.setPasswordHash("5587ba7a03b12a409ee5830cea97e079");
    ArduinoOTA
      .onStart([]() {
        esp_task_wdt_reset();
        WdtTimer=millis();

        Interrupts(false);
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
        Interrupts(true);
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        Interrupts(true);
      });

    ArduinoOTA.begin();
  #endif

  // mount SPIFFS (web pages). Try without formatting first so an already-good FS is kept as-is.
  // If that fails (e.g. the on-flash partition table predates this build, so a pre-built spiffs.bin
  // was sized for the wrong partition and its SPIFFS_USE_MAGIC_LENGTH magic no longer matches),
  // format to the partition's ACTUAL geometry and mount empty. The PROGMEM fallback page then
  // tells the operator to (re)upload spiffs.bin built for the real partition size reported by /api/config.
  FsMounted = SPIFFS.begin(false);
  if(!FsMounted){
    Serial.println("SPIFFS mount failed - formatting to the partition's real geometry");
    FsMounted = SPIFFS.begin(true);
  }
  if(FsMounted){
    Serial.print("SPIFFS mounted: total ");
    Serial.print(SPIFFS.totalBytes());
    Serial.print(" B, used ");
    Serial.print(SPIFFS.usedBytes());
    Serial.println(" B");
  }else{
    Serial.println("SPIFFS mount failed - upload spiffs.bin via OTA filesystem image");
  }

  #if defined(OTAWEB)
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if(WebAuthEnabled && !request->authenticate(WEB_AUTH_USER, WebAuthPassword.c_str())){
          return request->requestAuthentication();
        }
        request->send(200, "text/plain", "PSE QSY to /update");
    });
    AsyncElegantOTA_IPR.begin(&OTAserver, WEB_AUTH_USER, WebAuthPassword.c_str(), &WebAuthEnabled, REV);    // firmware + filesystem upload
    // The dashboard is served from the sync server on :80, the SSE stream lives here on :82 (different
    // origin), so allow cross-origin for the EventSource. The stream is public read-only wind data.
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    OTAserver.addHandler(&events);
    OTAserver.begin();
  #endif

  #if defined(Ser2net)
    Serial_one.begin(SERIAL1_BAUDRATE, SERIAL_8N1, RX1, TX1);
  // Serial2.begin(9600);
  SerialServer.begin(SerialServerIPport);
  SerialServer.setNoDelay(true);
  #endif

  TelnetServer.begin(TelnetServerIPport);
  // TelnetlServer.setNoDelay(true);


  #if HWREVsw==8
    int intBuf = analogRead(RainPin);
    if(intBuf<1000){
      RainStatus=false;
    }else if(intBuf>=1000 && intBuf<=2000){
      RainStatus=true;
    }
  #endif
  #if HWREVsw==7
    if(digitalRead(Rain1Pin)==0 && digitalRead(Rain2Pin)==1){
      RainStatus=false;
    }else if(digitalRead(Rain1Pin)==1 && digitalRead(Rain2Pin)==0){
      RainStatus=true;
    }
  #endif

  #if defined(RF69_EXTERNAL_SENSOR)
   // clock, miso,mosi, ss
    SPI.begin(14, 15, 2, 0);
    //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

    Serial.println();
    Serial.print("RFM69 radio init ");
    if (!rf69.init()) {
      Serial.println("failed!");
      RF69enable = false;
      // while (1);
    }else{
      Serial.println("OK");
      RF69enable = true;

      // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
      // No encryption
      Serial.print("RFM69 setFrequency ");
      if (!rf69.setFrequency(RF69_FREQ)) {
        Serial.println("failed!");
      }else{
        Serial.println("OK");
        // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
        // ishighpowermodule flag set like this:
        rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

        // The encryption key has to be the same as the one in the server
        uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
          0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
        };
        rf69.setEncryptionKey(key);
        Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
      }
    }

  #endif
  //------------------------------------------------

  // digitalWrite(EnablePin,0);

  // WDT
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  WdtTimer=millis();
  ConnOkTimer=millis();   // software-watchdog connectivity timer counts from boot

  //init and get the time
   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
   RainCountRestore();   // restore today's rain counter from EEPROM (survive reboot)
   Interrupts(true);

   // new web UI on :80. Authorization header must be collected so webserver.authenticate() works.
   {
     static const char* webAuthHeaders[] = { "Authorization" };
     webserver.collectHeaders(webAuthHeaders, 1);
   }
   webserver.on("/", HTTP_GET, handleRoot);          // responsive meteo dashboard (SPIFFS or PROGMEM fallback)
   webserver.on("/api/live", HTTP_GET, handleApiLive); // cached meteo JSON for the dashboard
   webserver.on("/api/wind", HTTP_GET, handleApiWind); // tiny fast-poll: live wind dir + speed
   webserver.on("/api/wallcfg", HTTP_GET, handleApiWallCfg); // broker ws URI + topic for /wall
   webserver.on("/setup", HTTP_GET, handleSetupPage);        // settings page (auth-gated)
   webserver.on("/api/config", HTTP_GET, handleApiConfig);   // /setup form data + diagnostics
   webserver.on("/api/config", HTTP_POST, handleApiConfigSave); // /setup global Save
   webserver.on("/api/status", HTTP_GET, handleApiStatus);   // Status tab: full sensor + raw debug dump
   webserver.on("/api/debug", HTTP_GET, handleApiDebug);     // Debug tab: level + recent log lines
   webserver.onNotFound(handleStatic);               // static SPIFFS files (gzip-aware), else 404
   webserver.begin();
   Serial.println("HTTP web server started on :80");

   // Spawn the wind-speed cache task on core 0 (loop() runs on core 1). Keeps /api/live wind values
   // fresh independently of how long loop() blocks in the publish cycle. Low priority, 4 kB stack.
   xTaskCreatePinnedToCore(WindTask, "WindTask", 4096, NULL, 1, NULL, 0);
   Serial.println("WindTask started on core 0");

}

//-------------------------------------------------------------------------------------------------------

void loop() {
  webserver.handleClient();
  Mqtt();
  CLI();
  Telnet();
  Watchdog();

  // check_radio();

  // blank loop 80us
  // SerialToIp();
  // if(millis()-TimerTemp > 60000){
  //   String StringUtc1 = UtcTime(1);
  //   // MqttPubString("tmp/utc", StringUtc1, false);
  //   sensors.requestTemperatures();
  //   float temperatureC = sensors.getTempCByIndex(0);
  //   MqttPubString("tmp/c", StringUtc1+" "+String(temperatureC), false);
  //   TimerTemp=millis();
  // }

  /*
  Scanning...
  I2C device found at address 0x40 - HTU21D - Humidity + Temp sensor
  I2C device found at address 0x76 - BMP280 - Pressure sensor
  */
  #if defined(EnableOTA)
   ArduinoOTA.handle();
  #endif

  #if defined(OTAWEB)
   // OTAserver.on("/printIp", HTTP_GET, [](AsyncWebServerRequest *request){
   //     request->send(200, "text/plain", "ok");
   //     Serial.println(request->client()->remoteIP());
   // });
   AsyncElegantOTA_IPR.loop();
  #endif
}
// SUBROUTINES -------------------------------------------------------------------------------------------------------

void IRAM_ATTR RPMcount(){    // must be before attachInterrupt ;)
  // IRAM-safe ISR: reads the pin straight from the GPIO register (digitalRead is not guaranteed to
  // stay in IRAM, which would crash if the ISR fired during a flash write) and does ALL min/max/avg
  // accumulation here under rpmMux. Because every pulse is counted at the instant it arrives, gusts
  // are captured even while loop() is busy (web/MQTT/TLS) and even during EEPROM commits - we no
  // longer detach the interrupt for those. Only integer math + millis() (both IRAM-safe) run here;
  // the timestamp + EEPROM persistence of the lifetime max stay in loop()/Watchdog (see GustEvent).
  // RpmPin=39 lives in the GPIO 32-39 bank, read via GPIO_IN1_REG bit (pin-32).
  if(((REG_READ(GPIO_IN1_REG) >> (RpmPin - 32)) & 0x1) != 0) return;   // pin not low -> ignore (FALLING confirm)

  portENTER_CRITICAL_ISR(&rpmMux);
  unsigned long now = millis();
  long pulse = (long)(now - RpmTimer[0]);
  RpmTimer[0] = now;
  if(pulse < 10){                       // <10ms => >300km/h => electrical noise, treat as "no reading"
    RpmPulse = 987654321;
  }else{
    RpmPulse = pulse;
    if(pulse < PeriodMinRpmPulse){      // shorter period = faster wind = new period max (gust)
      PeriodMinRpmPulse = pulse;
      GustEvent = true;                 // loop() will timestamp it and persist a new lifetime max
    }
    if(pulse < RollBucketMin){          // feed the rolling 5-min window (WindTask buckets this)
      RollBucketMin = pulse;
    }
    if(pulse < RpmTimer[1]){            // only pulses < 3s count into the running average (ignore calm)
      RpmAverage[1] += pulse;
      RpmAverage[0]++;
    }
  }
  RpmInterrupt = true;
  portEXIT_CRITICAL_ISR(&rpmMux);
}
//-------------------------------------------------------------------------------------------------------
void Interrupts(boolean ON){
  // nesting safe - inner Interrupts(true) must not re-enable prematurely
  if(ON==true){
    if(InterruptDepth>0){
      InterruptDepth--;
    }
    if(InterruptDepth==0){
      attachInterrupt(RpmPin, RPMcount, FALLING);
    }
    // attachInterrupt(digitalPinToInterrupt(RpmPin), RPMcount, FALLING);
    // attachInterrupt(digitalPinToInterrupt(RpmPin), RPMcount, RISING);
  }else{
    if(InterruptDepth==0){
      detachInterrupt(digitalPinToInterrupt(RpmPin));
    }
    InterruptDepth++;
  }
}
//-------------------------------------------------------------------------------------------------------
// The RPM ISR is now fully IRAM-safe (register read + integer math, no flash-resident calls), so it
// may keep firing while the flash is being written - no need to detach it around the commit anymore.
void EEPROMcommit(){ EEPROM.commit(); }

// Persist today's rain counter to EEPROM so a reboot (recovery restart / power loss)
// does not lose the daily rain total. Writes only when the value changed (EEPROM wear).
void RainCountStore(){
  int d = RainCountDayOfMonth.toInt();   // "26"->26 ; "n/a"->0
  if(d>=1 && d<=31){
    if(EEPROM.readByte(368)!=(byte)d || EEPROM.readInt(369)!=RainCount){
      EEPROM.writeByte(368, (byte)d);
      EEPROM.writeInt(369, RainCount);
      EEPROMcommit();
    }
  }
}

// Restore today's rain counter from EEPROM at boot. The day-change logic in GetValue()
// (UtcTime(2)!=RainCountDayOfMonth) then keeps it for the same day or zeroes it across midnight.
void RainCountRestore(){
  byte d = EEPROM.readByte(368);
  if(d>=1 && d<=31){
    int c = EEPROM.readInt(369);
    if(c<0 || c>100000){ c=0; }   // guard against unset/corrupt value
    RainCount = c;
    RainCountDayOfMonth = (d<10?"0":"") + String(d);   // zero-padded to match UtcTime(2) "%d"
  }else{
    RainCount = 0;
    RainCountDayOfMonth = UtcTime(2);
  }
}
//-------------------------------------------------------------------------------------------------------
void Watchdog(){

  // RPM - the min/max/avg accumulation now happens inside the IRAM-safe RPMcount() ISR (so no pulse
  // is lost while loop() is busy). Here we only react to a gust flagged by the ISR: stamp the time of
  // the new period max and, if it also beats the lifetime record, persist it to EEPROM. The ISR can
  // keep firing during EEPROMcommit() because it is fully IRAM-safe, so we no longer detach it.
  if(GustEvent==true){
    portENTER_CRITICAL(&rpmMux);
    GustEvent = false;
    long periodMin = PeriodMinRpmPulse;   // shortest period seen this window = the gust
    portEXIT_CRITICAL(&rpmMux);

    WindSpeedMaxPeriodUTC = UtcTime(1);
    if(periodMin < MinRpmPulse && needEEPROMcommit==false){
      MinRpmPulseTimestamp = WindSpeedMaxPeriodUTC;
      MinRpmPulse = periodMin;
      EEPROM.writeLong(169, MinRpmPulse);
      EEPROM.writeString(173, MinRpmPulseTimestamp);
      needEEPROMcommit = true;
    }
    if(needEEPROMcommit==true){
      needEEPROMcommit = false;
      EEPROMcommit();
      MqttPubString("WindSpeedMax-mps", String(PulseToMetterBySecond(MinRpmPulse)), true);
      MqttPubString("WindSpeedMax-utc", String(MinRpmPulseTimestamp), true);
    }
  }


  // WDT
  if(millis()-WdtTimer > 60000){
    esp_task_wdt_reset();
    WdtTimer=millis();
    if(EnableSerialDebug>0){
      Prn(3, 0,"WDT reset ");
      Prn(3, 1, UtcTime(1));
    }
  }

  // Software watchdog - recover from failures the HW task-WDT can't catch.
  // The HW WDT only detects a frozen loop()/CPU. The real long-term failure mode is the
  // network stack dying (socket/heap exhaustion) while loop() keeps running and feeding the WDT.
  // Here we restart if ETH connectivity has been lost too long, or free heap is critically low.
  // (MQTT loss alone does NOT trigger a restart - the station must keep running on an island LAN.)
  {
    static unsigned long ConnLastCheck=0;
    if(millis()-ConnLastCheck > 5000){   // check every 5s
      ConnLastCheck=millis();
      if(eth_connected==true){
        ConnOkTimer=millis();   // healthy - ETH link/IP is what we watch (MQTT may be intentionally absent on an island LAN)
      }
      bool recoveryReboot=false;
      if(ConnOkTimer>0 && millis()-ConnOkTimer > CONN_LOST_REBOOT_MS){
        Prn(3, 1,"** Software watchdog: ETH connectivity lost too long - restart **");
        recoveryReboot=true;
      }
      if(ESP.getFreeHeap() < HEAP_MIN_FREE){
        Prn(3, 1,"** Software watchdog: free heap critically low ("+String(ESP.getFreeHeap())+" B) - restart **");
        recoveryReboot=true;
      }
      if(recoveryReboot==true){
        RainCountStore();   // persist today's rain so the recovery reboot does not lose it
        delay(200);
        ESP.restart();
      }
    }
  }

  // frenetic mode
  if(EnableSerialDebug>1 && millis()-FreneticModeTimer > 1000){
    Azimuth();
    Prn(3, 1, "  Wind speed az/last/avg/max "+String(WindDir)+"° "+String(PulseToMetterBySecond(RpmPulse)*3.6)+"/"+String(PulseToMetterBySecond(AvgRpmPulse())*3.6)+"/"+String(PulseToMetterBySecond(PeriodMinRpmPulse)*3.6)+" km/h");
    if(eth_connected==true && mqttClient.connected()==true){
      MqttPubString("WindSpeed_az/last/avg/max", String(WindDir)+"° "+String(PulseToMetterBySecond(RpmPulse)*3.6)+"/"+String(PulseToMetterBySecond(AvgRpmPulse())*3.6)+"/"+String(PulseToMetterBySecond(PeriodMinRpmPulse)*3.6)+" km/h", false);
    }
    FreneticModeTimer=millis();
  }

  // if(NeedSpeedAlert_ms==true){
  //   NeedSpeedAlert_ms=false;
  //   MqttPubString("SpeedAlert-mps", String(PulseToMetterBySecond(RpmPulse)), false);
  //   Azimuth();
  //   MqttPubString("SpeedAlert-az", String(WindDir), false);
  // }


  // if(millis()-HeartBeatTimer[0]>HeartBeatTimer[1]){
  //   MqttPubString("HeartBeat", String(UtcTime(1)), false);
  //   // MqttPubString("WindDir", String(WindDir), false);
  //   HeartBeatTimer[0]=millis();
  // }

  // Rain counter 5sec
  if(millis()-RainTimer[0]>RainTimer[1]){
    #if HWREVsw==8
      if(EnableSerialDebug>1){Prn(3, 1,"*RainStatus "+String(RainStatus)+"|RainCount "+String(RainCount)+"|"+String(UtcTime(1)));}
      int intBuf = analogRead(RainPin);
      if(intBuf<1000){
        if(RainStatus==true){
          RainCount++;
          MqttPubString("RainCount", String(RainCount)+" (day "+String(UtcTime(2))+")", false);
          MqttPubString("RainToday-mm", String(RainPulseToMM(RainCount)), false);
          RainStatus=false;
        }
      }else if(intBuf>=1000 && intBuf<=2000){
        if(RainStatus==false){
          RainCount++;
          MqttPubString("RainCount", String(RainCount)+" (day "+String(UtcTime(2))+")", false);
          MqttPubString("RainToday-mm", String(RainPulseToMM(RainCount)), false);
          RainStatus=true;
        }
      }
      RainTimer[0]=millis();
      if(RainCountDayOfMonth=="n/a"){
        RainCountDayOfMonth=UtcTime(2);
      }
    #endif

    #if HWREVsw==7
      if(EnableSerialDebug>1){Prn(3, 1,"*RainStatus "+String(RainStatus)+"|RainCount "+String(RainCount)+"|"+String(UtcTime(1)));}
      // digitalWrite(EnablePin,1);
      if(digitalRead(Rain1Pin)==0 && digitalRead(Rain2Pin)==1){
        if(RainStatus==true){
          // #if defined(HTU21D)
          //   if(HTU21Denable==true){
          //     // debouncing
          //     if(htu.readHumidity()>90){
                RainCount++;
                MqttPubString("RainCount", String(RainCount)+" (day "+String(UtcTime(2))+")", false);
                MqttPubString("RainToday-mm", String(RainPulseToMM(RainCount)), false);
          //     }
          //   }
          // #endif
          RainStatus=false;
        }
      }else if(digitalRead(Rain1Pin)==1 && digitalRead(Rain2Pin)==0){
        if(RainStatus==false){
          // #if defined(HTU21D)
          //   if(HTU21Denable==true){
          //     // debouncing
          //     if(htu.readHumidity()>90){
                RainCount++;
                MqttPubString("RainCount", String(RainCount)+" (day "+String(UtcTime(2))+")", false);
                MqttPubString("RainToday-mm", String(RainPulseToMM(RainCount)), false);
          //     }
          //   }
          // #endif
          RainStatus=true;
        }
      }
      RainTimer[0]=millis();

      if(RainCountDayOfMonth=="n/a"){
        RainCountDayOfMonth=UtcTime(2);
      }
      // digitalWrite(EnablePin,0);
    #endif
  }

  // Measurement cycle (every MeasureTimer[1], default 5 min): full measurement incl. DS18B20, then
  // reset the wind accumulators for the next period. The cycle now runs on ETH alone (no longer gated
  // on MQTT), so the external DS18B20 temperature refresh, the today's-rain EEPROM persistence and the
  // period-max reset all keep working in an offline / island LAN. Only the network uploads (MQTT/APRS/
  // windy) stay gated on the MQTT connection - used here as an "internet is up" proxy so AprsWxIgate()/
  // GetHttpsWindy() don't stall the loop trying to open a socket that can't connect.
  if(millis()-MeasureTimer[0]>MeasureTimer[1] && eth_connected==true){
    // No more Interrupts(false) around this block: the IRAM-safe ISR keeps accumulating gusts through
    // the (multi-second) windy.com TLS upload and the DS18B20 read, so strong-wind peaks are no
    // longer lost during the cycle.
    // digitalWrite(EnablePin,1);

    GetValue(true);   // full read incl. DS18B20 - runs every cycle regardless of MQTT

    if(mqttClient.connected()==true){   // network uploads only when connected (internet up)
      MqttPubValue();
      AprsWxIgate();
      GetHttpsWindy();
      MqttPubString("FreeHeap", String(ESP.getFreeHeap()), false);   // monitor heap trend (leak/fragmentation)
    }

    RainCountStore();   // persist today's rain counter (only writes EEPROM when it changed)

    // reset wind accumulators AFTER publishing so the period average/max span the whole interval
    portENTER_CRITICAL(&rpmMux);
    RpmAverage[0]=0;
    RpmAverage[1]=0;
    PeriodMinRpmPulse=987654321;
    portEXIT_CRITICAL(&rpmMux);
    WindSpeedMaxPeriodUTC="";
    MeasureTimer[0]=millis();
    LiveCacheTimer=millis();   // fresh cache, no need for a fast refresh right after a full one
    // digitalWrite(EnablePin,0);
  }

  // Fast cache refresh for the web dashboard: samples quick sensors only (no DS18B20, interrupts
  // stay on), independent of MQTT, so /api/live reflects current wind/temp/humidity within seconds.
  if(millis()-LiveCacheTimer > LiveCacheInterval && eth_connected==true){
    GetValue(false);
    LiveCacheTimer=millis();
  }

  // Wind direction refresh (1s). Azimuth() bit-bangs the shift register on core 1 only, so WindTask
  // (core 0) never touches it - it just reads the resulting atomic int WindDir for the SSE push. A
  // mechanical vane moves slowly, so 1s is plenty and the value stays current for the live needle.
  if(millis()-WindDirTimer > 1000){
    Azimuth();
    WindDirTimer=millis();
    // Timestamp the rolling 5-min max whenever it reaches a new high (UtcTime is safe here on core 1,
    // not in WindTask). When the peak ages out of the window the value drops and a later gust re-stamps.
    static float lastRoll = -1;
    if(WindRollMaxMps > lastRoll + 0.05){
      WindRollMaxUtc = UtcTime(1);
    }
    lastRoll = WindRollMaxMps;
  }

  if(!TelnetServerClients[0].connected() && FirstListCommands==false){
    FirstListCommands=true;
  }

}


//-------------------------------------------------------------------------------------------------------

// readSlow=true  : full measurement (incl. DS18B20 OneWire read); call from the 5-min publish cycle
//                  with interrupts disabled. Does NOT reset wind accumulators (the caller does that
//                  after publishing, so averages span the whole period).
// readSlow=false : fast refresh for the web dashboard - samples the quick sensors (HTU/BMP/wind)
//                  only, keeps the last DS18B20 temperature, leaves interrupts running, so it can
//                  run every few seconds without losing wind pulses or shrinking the publish period.
void GetValue(bool readSlow){

  if(UtcTime(2)!=RainCountDayOfMonth){
    RainCount=0;
    RainCountDayOfMonth=UtcTime(2);
  }

  WindDir = 0;
  // RainCount = 0;
  RainTodayMM = 0;
  WindSpeedAvgMPS = 0;
  // WindSpeedMaxPeriodMPS = 0;
  PressureHPA = 0;
  TemperatureCelsius = 0;
  HumidityRelPercent = 0;
  DewPointCelsius = 0;
    PressureHPaBMP280 = 0;
    TemperatureCelsiusBMP280 = 0;
    HumidityRelPercentHTU21D = 0;
    DewPointCelsiusHTU21D = 0;
    // TemperatureCelsiusDS18B20 kept across fast refreshes (only re-read when readSlow)

  Azimuth();
  RainTodayMM = RainPulseToMM(RainCount);
  WindSpeedAvgMPS = PulseToMetterBySecond(AvgRpmPulse());
  if(PeriodMinRpmPulse<987654321){
    WindSpeedMaxPeriodMPS = PulseToMetterBySecond(PeriodMinRpmPulse);
  }else{
    WindSpeedMaxPeriodMPS = 0;
  }
  float htuTemp = 0;

  #if defined(DS18B20)
    if(ExtTemp==true && readSlow==true){
      sensors.requestTemperatures();
      float dsTemp = sensors.getTempC(insideThermometer);
      if(dsTemp != DEVICE_DISCONNECTED_C){   // -127 = sensor disconnected
        TemperatureCelsiusDS18B20 = dsTemp;
        ExtTempValid = true;
      }else{
        ExtTempValid = false;
      }
    }
  #endif
  #if defined(HTU21D)
    if(HTU21Denable==true){
      htuTemp = htu.readTemperature();
      HumidityRelPercentHTU21D = constrain(htu.readHumidity(), 0, 100);
      DewPointCelsiusHTU21D = htuTemp - (100.0 - HumidityRelPercentHTU21D) / 5.0;
      DewPointCelsius = DewPointCelsiusHTU21D;
      HumidityRelPercent = HumidityRelPercentHTU21D;
    }
  #endif
  #if defined(BMP280)
    if(BMP280enable==true){
      float bmpPress = bmp.readPressure();
      TemperatureCelsiusBMP280 = bmp.readTemperature();
      // reference temperature for sea-level pressure conversion (in Fahrenheit)
      double refTempF;
      if(ExtTempValid==true){
        refTempF = (TemperatureCelsiusDS18B20+TempCal)*1.8+32;
      }else{
        refTempF = htuTemp*1.8+32;   // fallback to HTU21D (or 0 if not present)
      }
      PressureHPaBMP280 = Babinet(double(bmpPress), refTempF)/100;
      TemperatureCelsius = TemperatureCelsiusBMP280;
      PressureHPA = PressureHPaBMP280;
    }
  #endif
  // DS18B20 is the master temperature source when available
  if(ExtTempValid==true){
    TemperatureCelsius = TemperatureCelsiusDS18B20;
  }

  TemperatureCelsius = TemperatureCelsius + TempCal;

  if(EnableSerialDebug>0){
    Prn(3, 0, "GET WindDir ");
      Prn(3, 1, String(WindDir));
    Prn(3, 0, "GET RainCount ");
      Prn(3, 1, String(RainCount)+" (day "+String(UtcTime(2))+")");
    Prn(3, 0, "GET RainTodayMM ");
      Prn(3, 1, String(RainTodayMM));
    Prn(3, 0, "GET WindSpeedAvgMPS ");
      Prn(3, 1, String(WindSpeedAvgMPS));
    Prn(3, 0, "GET WindSpeedMaxPeriodMPS ");
      Prn(3, 1, String(WindSpeedMaxPeriodMPS));
    Prn(3, 0, "GET WindSpeedMaxPeriodUTC ");
      Prn(3, 1, WindSpeedMaxPeriodUTC);
    Prn(3, 0, "GET PressureHPaBMP280 ");
      Prn(3, 1, String(PressureHPaBMP280));
    Prn(3, 0, "GET TemperatureCelsiusBMP280 ");
      Prn(3, 1, String(TemperatureCelsiusBMP280));
    Prn(3, 0, "GET HumidityRelPercentHTU21D ");
      Prn(3, 1, String(HumidityRelPercentHTU21D));
    Prn(3, 0, "GET DewPointCelsiusHTU21D ");
      Prn(3, 1, String(DewPointCelsiusHTU21D));
    Prn(3, 0, "GET TemperatureCelsiusDS18B20 ");
      Prn(3, 1, String(TemperatureCelsiusDS18B20));

    Prn(3, 0, "GET PressureHPA ");
      Prn(3, 1, String(PressureHPA));
    Prn(3, 0, "GET TemperatureCelsius ");
      Prn(3, 1, String(TemperatureCelsius));
    Prn(3, 0, "GET HumidityRelPercent ");
      Prn(3, 1, String(HumidityRelPercent));
    Prn(3, 0, "GET DewPointCelsius ");
      Prn(3, 1, String(DewPointCelsius));
  }

  #if defined(RF69_EXTERNAL_SENSOR)
    if(temp_radio.toInt()>0 && humidity_radio.toInt()>0 && vbat_radio.toInt()>0 && RF69enable==true){
      MqttPubString("RF-Temperature-Celsius", temp_radio, false);
      MqttPubString("RF-HumidityRel-Percent", humidity_radio, false);
      MqttPubString("RF-BattVoltage", vbat_radio, false);
    }
  #endif

  // Wind accumulators (period average + period max) are reset by the publish cycle AFTER publishing,
  // not here, so they span the full publish period regardless of how often GetValue() samples.

}
//-------------------------------------------------------------------------------------------------------

void MqttPubValue(){

  String StringUtc1 = UtcTime(1);
  MqttPubString("utc", StringUtc1, false);
  MqttPubString("WindDir-azimuth", String(WindDir), false);

  MqttPubString("RainCount", String(RainCount)+" (day "+String(UtcTime(2))+")", false);
  MqttPubString("RainToday-mm", String(RainTodayMM), false);

  MqttPubString("WindSpeedAvg-mps", String(WindSpeedAvgMPS), false);
  // if(PeriodMinRpmPulse<987654321){
    MqttPubString("WindSpeedMaxPeriod-mps", String(WindSpeedMaxPeriodMPS), false);
    MqttPubString("WindSpeedMaxPeriod-utc", String(WindSpeedMaxPeriodUTC), false);
  // }else{
  //   MqttPubString("WindSpeedMaxPeriod-mps", "0", false);
  // }

  #if defined(BMP280)
    if(BMP280enable==true){
      MqttPubString("Pressure-hPa-BMP280", String(PressureHPaBMP280), false);
      MqttPubString("Temperature-Celsius-BMP280", String(TemperatureCelsiusBMP280), false);
    }else{
      MqttPubString("Pressure-hPa-BMP280", "n/a", false);
      // MqttPubString("TemperatureBak-Celsius", "n/a", false);
    }
  #endif

  #if defined(RF69_EXTERNAL_SENSOR)
    if(temp_radio.toInt()>0 && humidity_radio.toInt()>0 && vbat_radio.toInt()>0 && RF69enable==true){
      MqttPubString("RF-Temperature-Celsius", temp_radio, false);
      MqttPubString("RF-HumidityRel-Percent", humidity_radio, false);
      MqttPubString("RF-BattVoltage", vbat_radio, false);
    }
  #endif
  #if defined(HTU21D)
    if(HTU21Denable==true){
      MqttPubString("HumidityRel-Percent-HTU21D", String(HumidityRelPercentHTU21D), false);
      MqttPubString("DewPoint-Celsius-HTU21D", String(DewPointCelsiusHTU21D), false);
      if(ExtTemp==false){
      }
    }else{
      MqttPubString("HumidityRel-Percent-HTU21D", "n/a", false);
      // MqttPubString("Temperature-Celsius", "n/a", false);
    }
  #endif
  #if defined(DS18B20)
    if(ExtTemp==true){
      MqttPubString("Temperature-Celsius-DS18B20", String(TemperatureCelsiusDS18B20), false);
    }
  #endif

  MqttPubString("Pressure-hPa", String(PressureHPA), false);
  MqttPubString("HumidityRel-Percent", String(HumidityRelPercent), false);
  MqttPubString("DewPoint-Celsius", String(DewPointCelsius), false);
  // must be last (trigger for e-ink)
  MqttPubString("Temperature-Celsius", String(TemperatureCelsius), false);

}


//----------------------------RADIO--------------------------------------------------------------------
// https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF69.html

void check_radio() {
  #if defined(RF69_EXTERNAL_SENSOR)
    static long CheckRadioTimer;
    if ((millis() - CheckRadioTimer) > 2000 && RF69enable==true){
      Interrupts(false);
      if (rf69.available()) {
        if(EnableSerialDebug>0){
          Prn(3, 0,"RF69 ");
        }
        if(rf69.recv(buf, &len)) {
          if (!len) return;
          buf[len] = 0;
          received_data = ((char*)buf);
          if(EnableSerialDebug>0){
            Prn(3, 1,"RX: "+String(received_data) );
          }
          // Serial.print("Received:" );
          // Serial.println(received_data);
          temp_radio = getValue(received_data, ',', 0);
          humidity_radio = getValue(received_data, ',', 1);
          vbat_radio = getValue(received_data, ',', 2);
          MqttPubString("RF-Temperature-Celsius", temp_radio, false);
          MqttPubString("RF-HumidityRel-Percent", humidity_radio, false);
          MqttPubString("RF-BattVoltage", vbat_radio, false);
        }else{
          if(EnableSerialDebug>0){
            Prn(3, 1,"receive failed!");
          }
        }
      // }else{
      //   if(EnableSerialDebug>0){
      //     Prn(3, 1,"RF69 not available!");
      //   }
      }
      CheckRadioTimer=millis();
      Interrupts(true);
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
void GetHttpsWindy(){
  #if defined(WINDY)
  // no key configured -> nothing to upload (set it with the 'K' menu command)
  if(WindyApiKey.length()==0){
    if(EnableSerialDebug>0){
      Prn(3, 1, "[windy] no API key set, skip upload");
    }
    return;
  }

  // The TLS client is created on the stack so all its handshake buffers (~30-40 kB) are
  // freed the moment this function returns -> keeps the long-term free heap high.
  WiFiClientSecure client;
  #if defined(WINDY_VERIFY_CERT)
    client.setCACert(rootCACertificate);   // authenticate server (costs extra heap)
  #else
    client.setInsecure();                  // skip cert validation -> lower memory footprint
  #endif
  client.setTimeout(8);   // 8s timeout for TLS read/connect, avoid blocking on a stalled server

  Prn(3, 1, "\n[windy] connecting to "+String(windyServer)+" ...");
  Prn(3, 1, "[windy] free heap before: "+String(ESP.getFreeHeap()));
  if (!client.connect(windyServer, 443)){
    Prn(3, 1, "[windy] connection failed!");
    client.stop();   // release TLS context/socket on failed connect (avoid leak)
  }else{
    Prn(3, 1, "[windy] connected!");
    // windy.com PWS (Wunderground-style) protocol, imperial units:
    // https://stations.windy.com/pws/update/API_KEY?winddir=&windspeedmph=&windgustmph=&tempf=&rainin=&baromin=&dewptf=&humidity=
    //   windspeedmph/windgustmph : mph   = m/s / 0.44704
    //   rainin                   : inch  = mm / 25.4
    //   baromin                  : inHg  = hPa * 0.02953
    //   tempf/dewptf             : F     = C * 1.8 + 32
    String req = "GET /pws/update/" + WindyApiKey
      + "?winddir="      + String(WindDir)
      + "&windspeedmph=" + String(WindSpeedAvgMPS/0.44704, 1)
      + "&windgustmph="  + String(WindSpeedMaxPeriodMPS/0.44704, 1)
      + "&tempf="        + String(TemperatureCelsius*1.8+32, 1)
      + "&rainin="       + String(RainTodayMM/25.4, 3)
      + "&baromin="      + String(PressureHPA*0.02953, 2)
      + "&dewptf="       + String(DewPointCelsius*1.8+32, 1)
      + "&humidity="     + String(HumidityRelPercent, 0)
      + " HTTP/1.1";
    if(EnableSerialDebug>0){
      Prn(3, 1, "[windy] "+req);
    }
    client.println(req);
    client.println("Host: stations.windy.com");
    client.println("Connection: close");
    client.println();

    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        Prn(3, 1, "[windy] headers received");
        break;
      }
    }

    Prn(3, 0, "[windy RX] ");
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }

    client.stop();
    Prn(3, 1, "[windy] stop");
  }
  #endif
}
//-------------------------------------------------------------------------------------------------------

// Prepocet tlaku vzduchu z absolutni hodnoty na hladinu more (OK1IRG)
// double CMain::Babinet(double Pz, double T){
double Babinet(double Pz, double T){
  double C, P0;
  // prepocet provadime podle Babinetova vzorce - nejdriv spocti teplotni korekci
  C = 16e3 * (1 + T/273);
  // vypocti tlak na hladine more
  P0 = Pz * (C + double(Altitude)) / (C - double(Altitude));
  return(P0);
}


//-------------------------------------------------------------------------------------------------------
// ToDo
// https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
int MpsToMs(int MPS){
  if(MPS>0){
    int ms;
    // ms = 1000*CupRotationCircleCircumferenceInMeters*Kfactor/MPS; // 0,21195 = cup rotaion circle circumference in meters (for 135 mm diameter), 1,3 = K factor
    ms = pow(MPS/500, 1/-0.784);
    return ms;
  }else{
    return 0;
    // for (int i = 0; i < MappingRow; i++) {
    //   if(mapping[i][0] >= PULSE && PULSE >= mapping[i+1][0]){   // find range
    //     return map(PULSE, mapping[i][0], mapping[i+1][0], mapping[i][1], mapping[i+1][1]);  // map(value, fromLow, fromHigh, toLow, toHigh) //*10 one decimal
    //     break;
    //   }
    // }
    // return 0;
  }
}
//-------------------------------------------------------------------------------------------------------
float PulseToMetterBySecond(long PULSE){
  // < 0,28 m/s
  if(PULSE>2000 || PULSE==0){
    return 0;
  }else{
    float mps;
    // ms = 1000/PULSE*CupRotationCircleCircumferenceInMeters*Kfactor; // 0,21195 = cup rotaion circle circumference in meters (for 135 mm diameter), 1,3 = K factor
    mps = 500*pow(PULSE, -0.784);  // https://mycurvefit.com
    return mps;

    // for (int i = 0; i < MappingRow; i++) {
    //   if(mapping[i][0] >= PULSE && PULSE >= mapping[i+1][0]){   // find range
    //     return map(PULSE, mapping[i][0], mapping[i+1][0], mapping[i][1], mapping[i+1][1]);  // map(value, fromLow, fromHigh, toLow, toHigh) //*10 one decimal
    //     break;
    //   }
    // }
    // return 0;
  }
}
//-------------------------------------------------------------------------------------------------------
// average pulse length over the period, safe against division by zero
long AvgRpmPulse(){
  portENTER_CRITICAL(&rpmMux);
  unsigned long cnt = RpmAverage[0];
  unsigned long sum = RpmAverage[1];
  portEXIT_CRITICAL(&rpmMux);
  if(cnt>0){
    return sum/cnt;
  }else{
    return 0;
  }
}
//-------------------------------------------------------------------------------------------------------
// Instantaneous wind speed for the live display. RpmPulse holds the last pulse interval, which would
// otherwise read as a fixed speed forever once the wind drops, so we decay to 0 after >2s with no new
// pulse (matches PulseToMetterBySecond's own >2000ms "no wind" cutoff). pow() runs outside the lock.
float WindNowMps(){
  portENTER_CRITICAL(&rpmMux);
  long pulse = RpmPulse;
  long t0 = RpmTimer[0];
  portEXIT_CRITICAL(&rpmMux);
  if((long)(millis() - (unsigned long)t0) > 2000) return 0;
  return PulseToMetterBySecond(pulse);
}
//-------------------------------------------------------------------------------------------------------
// WindTask - pinned to core 0 (PRO_CPU, the lwIP/EMAC core). With Ethernet there is no WiFi driver
// task hogging core 0, so there is spare capacity here. It recomputes the live wind-speed cache
// (period average + period max in m/s) that /api/live serves, AND pushes the instantaneous wind to
// the dashboard via the /events SSE stream so the speed needle tracks gusts in real time - all
// independently of how long loop() on core 1 is blocked in a slow publish (windy.com TLS, DS18B20,
// MQTT). It only reads the ISR accumulators under rpmMux and the atomic int WindDir; it deliberately
// does NOT touch EEPROM, MQTT, I2C, the shift register, or any shared String, so there is no
// cross-core race. Crucially WindTask is non-critical to measurement: the gust accumulation lives in
// the ISR, so even if this task stalled completely no wind data would be lost. Low priority so it
// never starves the network stack; not subscribed to the task-WDT.
// Rolling-window max: one min-pulse slot per second over a 300s window = exactly the trailing 5.0 min.
#define WMAX_WINDOW_S 300
void WindTask(void *pv){
  unsigned long lastPush = 0;
  static long secMin[WMAX_WINDOW_S];
  for(int i=0;i<WMAX_WINDOW_S;i++) secMin[i] = 987654321;
  unsigned long lastSec = millis()/1000;
  for(;;){
    bool pulse;
    long avg, periodMin, rollRaw;
    portENTER_CRITICAL(&rpmMux);
    pulse = RpmInterrupt; RpmInterrupt = false;   // consume the "a pulse arrived" signal from the ISR
    avg = (RpmAverage[0]>0) ? (long)(RpmAverage[1]/RpmAverage[0]) : 0;
    periodMin = PeriodMinRpmPulse;
    rollRaw = RollBucketMin; RollBucketMin = 987654321;   // take + reset the inter-fold minimum
    portEXIT_CRITICAL(&rpmMux);

    WindSpeedAvgMPS = PulseToMetterBySecond(avg);
    WindSpeedMaxPeriodMPS = (periodMin < 987654321) ? PulseToMetterBySecond(periodMin) : 0;

    // Exact 5.0-min trailing max (dashboard only, independent of the publish-period reset): a circular
    // buffer holding the strongest gust (smallest pulse) of each of the last 300 seconds. As wall time
    // advances we clear the seconds just entered, fold the ISR's running min into the current second,
    // then take the min across the whole window. 1s resolution => window is exactly 300s.
    unsigned long nowSec = millis()/1000;
    if(nowSec != lastSec){
      unsigned long steps = nowSec - lastSec;
      if(steps > WMAX_WINDOW_S) steps = WMAX_WINDOW_S;
      for(unsigned long k=1;k<=steps;k++) secMin[(lastSec+k)%WMAX_WINDOW_S] = 987654321;
      lastSec = nowSec;
    }
    long idx = nowSec % WMAX_WINDOW_S;
    if(rollRaw < secMin[idx]) secMin[idx] = rollRaw;
    long rmin = 987654321;
    for(int i=0;i<WMAX_WINDOW_S;i++) if(secMin[i] < rmin) rmin = secMin[i];
    WindRollMaxMps = (rmin < 987654321) ? PulseToMetterBySecond(rmin) : 0;

    #if defined(OTAWEB)
    // Push to the dashboard SSE stream: immediately on a new pulse (capped to 10 Hz so a storm of
    // pulses can't flood clients) plus a 1s keepalive. Only numeric fields go here - all 32-bit, safe
    // to read across cores; the String timestamp stays in /api/live (loop context). Skip when no one
    // is connected so calm weather costs nothing.
    unsigned long nowMs = millis();
    bool keepalive = (nowMs - lastPush >= 1000);
    bool onPulse   = (pulse && (nowMs - lastPush >= 100));
    if((onPulse || keepalive) && events.count() > 0){
      lastPush = nowMs;
      String j; j.reserve(96);
      j  = "{";
      j += "\"lastMs\":" + String(WindNowMps(), 1) + ",";
      j += "\"windMaxMs\":" + String(WindRollMaxMps, 1) + ",";   // rolling 5-min max (dashboard)
      j += "\"windDir\":" + String(WindDir);
      j += "}";
      events.send(j.c_str(), "wind", nowMs);
    }
    #endif

    vTaskDelay(pdMS_TO_TICKS(50));   // 20 Hz tick: a new pulse is reflected within ~50ms
  }
}
//-------------------------------------------------------------------------------------------------------
byte Azimuth(){    // run from interrupt
  byte rxShiftInByte=0x00;
  digitalWrite(ShiftInLatchPin,0);     //Set latch pin to 0 to get data from the CD4021
  delayMicroseconds(10);
  digitalWrite(ShiftInLatchPin,1);   //Set latch pin to 1 to get recent data into the CD4021
  delayMicroseconds(10);

  for (int i=0; i<8; i++){                // 16 = two bank
    digitalWrite(ShiftInClockPin, 0);
    delayMicroseconds(10);
    // delay(1);
    // rxShiftInByte = rxShiftInByte  | (digitalRead(ShiftInDataPin)<<i);
    bitWrite(rxShiftInByte, i, digitalRead(ShiftInDataPin));
    digitalWrite(ShiftInClockPin, 1);
    delayMicroseconds(10);
    // delay(1);
  }
  switch (rxShiftInByte){
    case 0b01111111:
      AzShift(180); // 0
      break;
    case 0b01111110:
      AzShift(202); // 22
      break;
    case 0b11111110:
      AzShift(225); // 45
      break;
    case 0b11111100:
      AzShift(247); // 67
      break;
    case 0b11111101:
      AzShift(270); // 90
      break;
    case 0b11111001:
      AzShift(292); // 112
      break;
    case 0b11111011:
      AzShift(315); // 135
      break;
    case 0b11110011:
      AzShift(337); // 157
      break;
    case 0b11110111:
      AzShift(0); // 180
      break;
    case 0b11100111:
      AzShift(22);
      break;
    case 0b11101111:
      AzShift(45);
      break;
    case 0b11001111:
      AzShift(67);
      break;
    case 0b11011111:
      AzShift(90);
      break;
    case 0b10011111:
      AzShift(112);
      break;
    case 0b10111111:
      AzShift(135);
      break;
    case 0b00111111:
      AzShift(157);
      break;
    default:
      WindDir=-1;
      break;
  }
  return rxShiftInByte;
}

//-------------------------------------------------------------------------------------------------------
void AzShift(int AZ){
  WindDir=(AZ+WindDirShift)%360;   // wrap 0-359, handles also exactly 360
}
//-------------------------------------------------------------------------------------------------------

void AprsWxIgate() {
  if(AprsON==true){
    AprsClient.setTimeout(5);   // 5s socket timeout, avoid blocking on a stalled server
    if (!AprsClient.connect("czech.aprs2.net", 14580)) {  // client.connect(URL, port);  char URL[]="google.com"
      if(EnableSerialDebug>0){
        Prn(3, 1,"APRS client connect failed!");
      }
      AprsClient.stop();   // release socket even on failed connect (avoid fd leak)
      return;
    }
    // login
    if(EnableSerialDebug>0){
      Prn(3, 1,"APRS-TX | user "+YOUR_CALL+" pass "+AprsPassword+" vers esp32wx "+REV+" filter m/1");
    }
    AprsClient.println("user "+YOUR_CALL+" pass "+AprsPassword+" vers esp32wx "+REV+" filter m/1");
    delay (250);

    // send data
    if(EnableSerialDebug>0){
      Prn(3, 1,"APRS-TX | "+YOUR_CALL+">APRSWX,TCPIP*,qAS,:="
      +AprsCoordinates+"_"
      +LeadingZero(3,WindDir)
      +"/"+LeadingZero(3,WindSpeedAvgMPS/0.447)
      +"g"+LeadingZero(3,WindSpeedMaxPeriodMPS/0.447)
      +"t"+LeadingZero(3,(TemperatureCelsius)*1.8+32)
      +"h"+LeadingZero(3,constrain(HumidityRelPercent, 0, 100))
      +"b"+LeadingZero(6,PressureHPA*100)
      +"P"+LeadingZero(3,RainTodayMM/0.254) );
    }

    AprsClient.println(YOUR_CALL+">APRSWX,TCPIP*,qAS,:="
    +AprsCoordinates+"_"
    +LeadingZero(3,WindDir)
    +"/"+LeadingZero(3,WindSpeedAvgMPS/0.447)
    +"g"+LeadingZero(3,WindSpeedMaxPeriodMPS/0.447)
    +"t"+LeadingZero(3,(TemperatureCelsius)*1.8+32)
    +"h"+LeadingZero(3,constrain(HumidityRelPercent, 0, 100))
    +"b"+LeadingZero(6,PressureHPA*100)
    +"P"+LeadingZero(3,RainTodayMM/0.254) );

    if(EnableSerialDebug>0){
      // APRS-TX | user OK1HRA-8 pass ***** vers esp32wx 20230916 filter m/1
      // APRS-TX | OK1HRA-8>APRSWX,TCPIP*,qAS,:=5001.99N/01350.16E_270/009g017t066h057b100307P000
      // APRS-TX | OK1HRA-8>APRSWX,TCPIP*,qAC:> remoteqth.com 3D-printed WX station
      Prn(3, 1,"APRS-TX | "+YOUR_CALL+">APRSWX,TCPIP*,qAC:> remoteqth.com 3D-printed WX station");
    }
    AprsClient.println(YOUR_CALL+">APRSWX,TCPIP*,qAC:> remoteqth.com 3D-printed WX station");

    AprsClient.flush();  // push the TX buffer out before closing
    delay(500);          // let the server read the lines; stop() right away aborts unsent data -> nothing reaches APRS
    AprsClient.stop();   // close TCP socket, otherwise sockets leak every cycle -> lwip runs out of fd
  }
}

//-------------------------------------------------------------------------------------------------------
String LeadingZero(int NumberOfZero, int NR){
  String StrBuf = String((int)round(NR));
  if(StrBuf.length()>NumberOfZero){
    StrBuf="0";
  }
  for (int i=StrBuf.length(); i<NumberOfZero; i++){
    StrBuf="0"+StrBuf;
  }
  return StrBuf;
}

//-------------------------------------------------------------------------------------------------------
// void Print_SDCard_Info ()
// {
//     uint8_t cardType = SD_MMC.cardType();
//
//     if(cardType == CARD_NONE){
//         Serial.println("No SD_MMC card attached");
//         return;
//     }
//
//     Serial.print("SD_MMC Card Type: ");
//     if(cardType == CARD_MMC){
//         Serial.println("MMC");
//     } else if(cardType == CARD_SD){
//         Serial.println("SDSC");
//     } else if(cardType == CARD_SDHC){
//         Serial.println("SDHC");
//     } else {
//         Serial.println("UNKNOWN");
//     }
//
//     uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
//     Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
//
//     listDir(SD_MMC, "/", 0);
//     createDir(SD_MMC, "/mydir");
//     listDir(SD_MMC, "/", 0);
//     removeDir(SD_MMC, "/mydir");
//     listDir(SD_MMC, "/", 2);
//     writeFile(SD_MMC, "/hello.txt", "Hello ");
//     appendFile(SD_MMC, "/hello.txt", "World!\n");
//     readFile(SD_MMC, "/hello.txt");
//     deleteFile(SD_MMC, "/foo.txt");
//     renameFile(SD_MMC, "/hello.txt", "/foo.txt");
//     readFile(SD_MMC, "/foo.txt");
//     testFileIO(SD_MMC, "/test.txt");
//     Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
//     Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));
//
// }
//---------------------------------------------------------------------------------------------------------
byte HiByte(int ID){
  bitClear(ID, 0);  // ->
  bitClear(ID, 1);
  bitClear(ID, 2);
  bitClear(ID, 3);
  ID = ID >> 4;
  return(ID);
}

//---------------------------------------------------------------------------------------------------------
byte LowByte(int ID){
  bitClear(ID, 4);
  bitClear(ID, 5);
  bitClear(ID, 6);
  bitClear(ID, 7);  // <-
  return(ID);
}

//-------------------------------------------------------------------------------------------------------
void CLI(){
  int OUT=2;
  // incomingByte = 0;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    OUT = 0;
  }

  if(TelnetServerClients[0].connected() && OUT!=0){
    TelnetAuthorized = false;
    if(TelnetServerClients[0].remoteIP()==TelnetServerClientAuth){
        TelnetAuthorized = true;
        // timeout ... > false
    }else{
      TelnetAuth();
    }
    if(TelnetAuthorized==true){
      // incomingByte = TelnetRX();
      if(incomingByte!=0){
        OUT=1;
        ListCommands(OUT);
        if(FirstListCommands==true){
          FirstListCommands=false;
        }
      }
    }
  }else if(!TelnetServerClients[0].connected()){
    TelnetAuthStep=0;
  }

  if(OUT==1){     // telnet only
  // if(OUT>-1){  // always
    esp_task_wdt_reset();
    WdtTimer=millis();

    // ?
    if(incomingByte==63){
      ListCommands(OUT);

    // +
    }else if(incomingByte==43){
      Prn(OUT, 1,"  enter MQTT broker IP address by four number (0-255) and press [enter] after each");
      Prn(OUT, 1,"  NOTE: public remoteqth broker 54.38.157.134:1883");
      for (int i=0; i<5; i++){
        if(i==4){
          Prn(OUT, 0,"enter IP port (1-65535) and press [");
        }
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        int intBuf=0;
        int mult=1;
        for (int j=InputByte[0]; j>0; j--){
          intBuf = intBuf + ((InputByte[j]-48)*mult);
          mult = mult*10;
        }
        if( (i<4 && intBuf>=0 && intBuf<=255) || (i==4 && intBuf>=1 && intBuf<=65535) ){
          if(i==4){
            EEPROM.writeInt(165, intBuf);
          }else{
            EEPROM.writeByte(161+i, intBuf);
          }
          // Prn(OUT, 1,"EEPROMcomit");
          EEPROMcommit();
        }else{
          Prn(OUT, 1,"Out of range.");
          break;
        }
      }
      Prn(OUT, 0,"** device will be restarted **");
      delay(1000);
      TelnetServerClients[0].stop();
      ESP.restart();

    // e
    }else if(incomingByte==101){
        Prn(OUT, 1,"List EEPROM [adr>value]");
        for (int i=0; i<EEPROM_SIZE; i++){
          if( (i<41)||(i>140&&i<208)||(i>212) ){  // hiden pass
            Prn(OUT, 0, String(i));
            Prn(OUT, 0, ">" );
            Prn(OUT, 0, String(EEPROM.read(i)) );
            Prn(OUT, 0, " " );
          }
        }
        Prn(OUT, 1, "" );

    // *
    }else if(incomingByte==42){
      EnableSerialDebug++;
      if(EnableSerialDebug>2){
        EnableSerialDebug=0;
      }
      Prn(OUT, 0,"** Terminal DEBUG ");
      if(EnableSerialDebug==0){
        Prn(OUT, 1,"DISABLE **");
      }else if(EnableSerialDebug==1){
        Prn(OUT, 1,"ENABLE **");
      }else if(EnableSerialDebug==2){
        Prn(OUT, 1,"ENABLE with frenetic mode **");
      }

    // E
    }else if(incomingByte==69 && OUT==0){
      Prn(OUT, 1,"  Erase whole eeprom (also telnet key)? (y/n)");
      EnterChar(OUT);
      if(incomingByte==89 || incomingByte==121){
        Prn(OUT, 1,"  Stop erase? (y/n)");
        EnterChar(OUT);
        if(incomingByte==78 || incomingByte==110){
          for(int i=0; i<EEPROM_SIZE; i++){
            EEPROM.write(i, 0xff);
            Prn(OUT, 0,".");
          }
          EEPROMcommit();
          Prn(OUT, 1,"");
          Prn(OUT, 1,"  Eeprom erased done");
        }else{
          Prn(OUT, 1,"  Erase aborted");
        }
      }else{
        Prn(OUT, 1,"  Erase aborted");
      }

    #if defined(OTAWEB)
    // P - clear web Basic-auth password / disable auth (recovery channel: web /setup is locked
    // behind this very password, so this lives here in the serial/telnet menu instead)
    }else if(incomingByte==80){
      if(WebAuthPassword.length()==0){
        Prn(OUT, 1,"  Web auth already OFF (no password set)");
      }else{
        Prn(OUT, 1,"  Clear web password / disable auth? (y/n)");
        EnterChar(OUT);
        if(incomingByte==89 || incomingByte==121){
          WebAuthPassword=""; WebAuthEnabled=false;
          eepromWriteString(405, 64, ""); EEPROM.write(469, 0);
          EEPROMcommit();
          Prn(OUT, 1,"  Web password cleared, auth OFF | ** device will be restarted **");
          delay(1000);
          TelnetServerClients[0].stop();
          ESP.restart();
        }else{
          Prn(OUT, 1,"  Aborted");
        }
      }
    #endif

    // .
    }else if(incomingByte==46){
      Prn(OUT, 1,"Reset timer and sent measure");
      MeasureTimer[0]=millis()-MeasureTimer[1];

    #if !defined(BMP280) && !defined(HTU21D)
      // 2
      }else if(incomingByte==50){
        I2cScanner();
    #endif

    // q
    }else if(incomingByte==113 && TelnetServerClients[0].connected() ){
      TelnetServerClients[0].stop();
      TelnetAuthorized=false;
      // TelnetServerClientAuth = {0,0,0,0};
      TelnetAuthStep=0;
      FirstListCommands=true;

    // Q
    }else if(incomingByte==81 && TelnetServerClients[0].connected() ){
      TelnetServerClients[0].stop();
      TelnetAuthorized=false;
      TelnetServerClientAuth = {0,0,0,0};
      Prn(OUT, 1,String(TelnetServerClientAuth));
      TelnetAuthStep=0;
      FirstListCommands=true;
      EEPROM.write(37, 0); // address, value
      EEPROM.write(38, 0); // address, value
      EEPROM.write(39, 0); // address, value
      EEPROM.write(40, 0); // address, value
      EEPROMcommit();

    // @
    }else if(incomingByte==64){
      Prn(OUT, 1,"** Device will be restarted **");
      TelnetServerClients[0].stop();
      ESP.restart();

    // ~K
    // }else if(incomingByte==126){
    //   EnterChar(OUT);
    //   if(incomingByte==75){
    //     Prn(OUT, 1,"");
    //     Prn(OUT, 0," ");
    //     for(int i=0; i<10; i++){
    //       Prn(OUT, 0,"    ["+String(i*10+1)+"-"+String(i*10+10)+"]  ");
    //       if(i<9){
    //         Prn(OUT, 0," ");
    //       }
    //       for(int j=0; j<10; j++){
    //         Prn(OUT, 0, String(key[i*10+j]));
    //       }
    //       Prn(OUT, 1,"");
    //     }
    //     Prn(OUT, 1,"");
    //   }

    // L
    }else if(incomingByte==76){
      Prn(OUT, 0,"  Input new location (CALLSIGN-ssid) and press [");
      if(TelnetAuthorized==true){
        Prn(OUT, 0,"enter");
      }else{
        Prn(OUT, 0,";");
      }
      Prn(OUT, 1,"]. If blank, will be use MAC");
      Enter();
      YOUR_CALL="";
      for (int i=1; i<21; i++){
        YOUR_CALL=YOUR_CALL+char(InputByte[i]);
        if(i<InputByte[0]+1){
          EEPROM.write(140+i, InputByte[i]);
        }else{
          EEPROM.write(140+i, 0xff);
        }
      }
      EEPROMcommit();
      Prn(OUT, 1,"** Device will be restarted **");
      delay(1000);
      ESP.restart();

    // A
    }else if(incomingByte==65){
      if(AprsON==true){
        AprsON=false;
        Prn(OUT, 1,"** APRS DISABLE **");
        EEPROM.write(199, AprsON); // address, value
        EEPROMcommit();
      }else{
        Prn(OUT, 1,"** Do you own a valid amateur radio license? **");
        EnterChar(OUT);
        if(incomingByte==89 || incomingByte==121){
          Prn(OUT, 1,"  Do you nothing disable APRS, after the license expires?");
          EnterChar(OUT);
          if(incomingByte==78 || incomingByte==110){
            AprsON=true;
            Prn(OUT, 1,"** APRS ENABLE **");
            EEPROM.write(199, AprsON); // address, value
            EEPROMcommit();
          }else{
            Prn(OUT, 1,"  bye");
          }
        }else{
          Prn(OUT, 1,"  bye");
        }
      }

    // p
    }else if(incomingByte==112 && AprsON==true){
        Prn(OUT, 0,"  Input APRS i-gate password and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        AprsPassword="";
        for (int i=1; i<6; i++){
          AprsPassword=AprsPassword+char(InputByte[i]);
          if(i<InputByte[0]+1){
            EEPROM.write(207+i, InputByte[i]);
          }else{
            EEPROM.write(207+i, 0xff);
          }
        }
        EEPROMcommit();

      // c
      }else if(incomingByte==99 && AprsON==true){
        Prn(OUT, 1,"  ddmm.ssN/dddmm.ssE | dd-degrees | mm-minutes | ss-seconds | N-north, S-south.");
        Prn(OUT, 1,"  for example 5108.41N/01259.35E");
        Prn(OUT, 0,"  Input WX coordinates and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        AprsCoordinates="";
        for (int i=1; i<19; i++){
          AprsCoordinates=AprsCoordinates+char(InputByte[i]);
          if(i<InputByte[0]+1){
            EEPROM.write(212+i, InputByte[i]);
          }else{
            EEPROM.write(212+i, 0xff);
          }
        }
        EEPROMcommit();

      // m
      }else if(incomingByte==109){
        Prn(OUT, 0,"write your altitude in meter, and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        int intBuf=0;
        int mult=1;
        for (int j=InputByte[0]; j>0; j--){
          // detect -
          if(j==1 && InputByte[j]==45){
            intBuf = 0-intBuf;
          }else{
            intBuf = intBuf + ((InputByte[j]-48)*mult);
            mult = mult*10;
          }
        }
        if(intBuf>=0 && intBuf<=7300){
          Altitude = intBuf;
          EEPROM.writeInt(231, Altitude); // address, value
          EEPROMcommit();
          Prn(OUT, 1," altitude "+String(EEPROM.readInt(231))+"m has been saved");
        }else{
          Prn(OUT, 1," Out of range");
        }

      // K - windy.com API key
      #if defined(WINDY)
      }else if(incomingByte==75){
        Prn(OUT, 1,"  Get the station API key at https://stations.windy.com (My stations).");
        Prn(OUT, 0,"  Paste windy API key (empty = disable) and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        // The key can be up to 123 chars - too long for the shared InputByte[21] buffer,
        // so read it straight into the String here (same CR/;/timeout rules as Enter()).
        String keyBuf="";
        bool br=false;
        unsigned long EnterTimeout=millis();
        Prn(OUT, 0,"> ");
        while(br==false){
          esp_task_wdt_reset();
          if(millis()-EnterTimeout>30000){
            Prn(OUT, 1," timeout");
            keyBuf="";
            break;
          }
          int rx=-1;
          if(OUT==1 && TelnetServerClients[0] && TelnetServerClients[0].connected() && TelnetServerClients[0].available()){
            rx=TelnetServerClients[0].read();
          }else if(OUT==0 && Serial.available()){
            rx=Serial.read();
          }
          if(rx<0){
            continue;
          }
          EnterTimeout=millis();
          if(rx==13 || rx==10 || rx==59){   // CR, LF or ;
            br=true;
          }else if(rx==127){                // backspace
            if(keyBuf.length()>0){
              keyBuf.remove(keyBuf.length()-1);
            }
          }else if(keyBuf.length()<123){
            keyBuf += char(rx);
          }
        }
        keyBuf.trim();
        if(keyBuf.length()<=123){
          WindyApiKey=keyBuf;
          for (int i=0; i<123; i++){
            if(i<(int)keyBuf.length()){
              EEPROM.write(244+i, keyBuf[i]);
            }else{
              EEPROM.write(244+i, 0xff);
            }
          }
          EEPROMcommit();
          if(WindyApiKey.length()==0){
            Prn(OUT, 1," windy upload disabled (key erased)");
          }else{
            Prn(OUT, 1," windy API key saved ("+String(WindyApiKey.length())+" chars)");
          }
        }else{
          Prn(OUT, 1," key too long (max 123)");
        }

      // I - windy.com public Station ID (for the web-page link)
      }else if(incomingByte==73){
        Prn(OUT, 1,"  Find it on https://stations.windy.com (public URL windy.com/station/pws-<ID>).");
        Prn(OUT, 0,"  Enter windy Station ID without pws- prefix, e.g. f052c6e5 (empty = remove) and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        String idBuf="";
        for (int i=1; i<InputByte[0]+1; i++){
          idBuf += char(InputByte[i]);
        }
        idBuf.trim();
        WindyStationId=idBuf;
        for (int i=0; i<32; i++){
          if(i<(int)idBuf.length()){
            EEPROM.write(373+i, idBuf[i]);
          }else{
            EEPROM.write(373+i, 0xff);
          }
        }
        EEPROMcommit();
        if(WindyStationId.length()==0){
          Prn(OUT, 1," windy Station ID removed (web link hidden)");
        }else{
          Prn(OUT, 1," windy Station ID saved | https://www.windy.com/station/pws-"+WindyStationId);
        }
      #endif

      // S
      }else if(incomingByte==83){
        Prn(OUT, 0,"write wind direction shift 0-359°, and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        int intBuf=0;
        int mult=1;
        for (int j=InputByte[0]; j>0; j--){
          // detect -
          if(j==1 && InputByte[j]==45){
            intBuf = 0-intBuf;
          }else{
            intBuf = intBuf + ((InputByte[j]-48)*mult);
            mult = mult*10;
          }
        }
        if(intBuf>=0 && intBuf<=359){
          WindDirShift = intBuf;
          EEPROM.writeInt(240, WindDirShift); // address, value
          EEPROMcommit();
          Prn(OUT, 1," shift "+String(EEPROM.readInt(240))+"° has been saved");
        }else{
          Prn(OUT, 1," Out of range");
        }

      // H
    }else if(incomingByte==72){
        if(HWREVpcb==0){
          Prn(OUT, 0,"define version of PCB, and press [");
          if(TelnetAuthorized==true){
            Prn(OUT, 1,"enter]");
          }else{
            Prn(OUT, 1,";]");
          }
          Enter();
          int intBuf=0;
          int mult=1;
          for (int j=InputByte[0]; j>0; j--){
            // detect -
            if(j==1 && InputByte[j]==45){
              intBuf = 0-intBuf;
            }else{
              intBuf = intBuf + ((InputByte[j]-48)*mult);
              mult = mult*10;
            }
          }
          // >7
          if(intBuf>=6 && intBuf<=255){
            HWREVpcb = intBuf;
            EEPROM.writeUChar(4, HWREVpcb); // address, value
            EEPROMcommit();
            Prn(OUT, 1," PCB version "+String(EEPROM.readUChar(4))+" has been saved");
          }else{
            Prn(OUT, 1," Out of range");
          }
        }

      // C
      }else if(incomingByte==67){
        Prn(OUT, 0,"write temperature shift in C°, and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        int intBuf=0;
        int mult=1;
        bool negativ = false;
        bool decimals = true;
        int decimal=1;
        for (int j=InputByte[0]; j>0; j--){
          // 0-9 || ,-.
          if( (InputByte[j]>=48 && InputByte[j]<=57) || (InputByte[j]>=44 && InputByte[j]<=46) ){
            if(InputByte[j]==45){
              negativ = true;
            }else if(InputByte[j]==44 || InputByte[j]==46){
              decimals=false;
            }else{
              intBuf = intBuf + ((InputByte[j]-48)*mult);
              mult = mult*10;
              if(decimals==true){
                decimal= decimal*10;
              }
            }
          }
        }
        if(negativ==true){
          intBuf=-intBuf;
        }
        if(decimals==false){
          TempCal = (float)intBuf/(float)decimal;
        }else{
          TempCal = (float)intBuf;
        }
        EEPROM.writeShort(2, TempCal*100.0); // address, value
        EEPROMcommit();
        Prn(OUT, 1," shift "+String((float)EEPROM.readShort(2)/100.0)+"C° has been saved");


      // R
      }else if(incomingByte==82){
        Prn(OUT, 0,"write rain in mm per pulse, and press [");
        if(TelnetAuthorized==true){
          Prn(OUT, 1,"enter]");
        }else{
          Prn(OUT, 1,";]");
        }
        Enter();
        int intBuf=0;
        int mult=1;
        bool negativ = false;
        bool decimals = true;
        int decimal=1;
        for (int j=InputByte[0]; j>0; j--){
          // 0-9 || ,-.
          if( (InputByte[j]>=48 && InputByte[j]<=57) || (InputByte[j]>=44 && InputByte[j]<=46) ){
            if(InputByte[j]==45){
              negativ = true;
            }else if(InputByte[j]==44 || InputByte[j]==46){
              decimals=false;
            }else{
              intBuf = intBuf + ((InputByte[j]-48)*mult);
              mult = mult*10;
              if(decimals==true){
                decimal= decimal*10;
              }
            }
          }
        }
        // if(negativ==true){
        //   intBuf=-intBuf;
        // }
        if(decimals==false){
          mmInPulse = (float)intBuf/(float)decimal;
        }else{
          mmInPulse = (float)intBuf;
        }
        EEPROM.writeShort(5, mmInPulse*100.0); // address, value
        EEPROMcommit();
        Prn(OUT, 1," rain "+String((float)EEPROM.readShort(5)/100.0)+"mm/pulse has been saved");

    // W
    }else if(incomingByte==87){
      Prn(OUT, 1,"** Erase WindSpeedMax memory? [y/n] **");
      EnterChar(OUT);
      if(incomingByte==89 || incomingByte==121){
        EEPROM.writeLong(169, 987654321);
        EEPROM.writeByte(173, 255);
        EEPROMcommit();
        MqttPubString("WindSpeedMax-mps", "", true);
        MqttPubString("WindSpeedMax-utc", "", true);
        Prn(OUT, 1,"** IP switch will be restarted **");
        delay(1000);
        TelnetServerClients[0].stop();
        ESP.restart();
      }else{
        Prn(OUT, 1,"  bye");
      }

// // a
// }else if(incomingByte==97){
//     Prn(OUT, 0,"write limit for wind speed alert 0-60 m/s, and press [");
//     if(TelnetAuthorized==true){
//       Prn(OUT, 1,"enter]");
//     }else{
//       Prn(OUT, 1,";]");
//     }
//     Enter();
//     int intBuf=0;
//     int mult=1;
//     for (int j=InputByte[0]; j>0; j--){
//       // detect -
//       if(j==1 && InputByte[j]==45){
//         intBuf = 0-intBuf;
//       }else{
//         intBuf = intBuf + ((InputByte[j]-48)*mult);
//         mult = mult*10;
//       }
//     }
//     if(intBuf>=0 && intBuf<=60){
//       SpeedAlertLimit_ms = MpsToMs(intBuf);
//       EEPROM.writeInt(235, SpeedAlertLimit_ms); // address, value
//       EEPROM.commit();
//     }else{
//       Prn(OUT, 1," Out of range");
//     }

    // CR/LF
    }else if(incomingByte==13||incomingByte==10){

    // anykey
    }else{

      if(EnableSerialDebug>0){
        Prn(OUT, 0," [");
        Prn(OUT, 0, String(incomingByte) ); //, DEC);
        Prn(OUT, 1,"] unknown command");
      }
    }
    incomingByte=0;

  }else if(OUT==0){
    ListCommands(OUT);
  }

}
//-------------------------------------------------------------------------------------------------------
void Enter(){
  int OUT;
  if(TelnetAuthorized==true){
    OUT=1;
  }else{
    OUT=0;
  }

  InputByte[0]=0;
  incomingByte = 0;
  bool br=false;
  unsigned long EnterTimeout=millis();   // abort input after 30s (avoid WDT reset)
  Prn(OUT, 0,"> ");

  if(OUT==0){
    while(br==false) {
      esp_task_wdt_reset();
      if(millis()-EnterTimeout>30000){
        br=true;
        InputByte[0]=0;
        Prn(OUT, 1," timeout");
        break;
      }
      if(Serial.available()){
        EnterTimeout=millis();
        incomingByte=Serial.read();
        if(incomingByte==13 || incomingByte==59){ // CR or ;
          br=true;
          Prn(OUT, 1,"");
        }else{
          Serial.write(incomingByte);
          if(incomingByte!=10 && incomingByte!=13){
            if(incomingByte==127){
              InputByte[0]--;
            }else{
              InputByte[InputByte[0]+1]=incomingByte;
              InputByte[0]++;
            }
          }
        }
        if(InputByte[0]==20){
          br=true;
          Prn(OUT, 1," too long");
        }
      }
    }

  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){

        while(br==false){
          esp_task_wdt_reset();
          if(millis()-EnterTimeout>30000){
            br=true;
            InputByte[0]=0;
            Prn(OUT, 1," timeout");
            break;
          }
          if(TelnetServerClients[0].available()){
            EnterTimeout=millis();
            incomingByte=TelnetServerClients[0].read();
            if(incomingByte==10){
              br=true;
              Prn(OUT, 1,"");
            }else{
              TelnetServerClients[0].write(incomingByte);
              if(incomingByte!=10 && incomingByte!=13){
                if(incomingByte==127){
                  InputByte[0]--;
                }else{
                  InputByte[InputByte[0]+1]=incomingByte;
                  InputByte[0]++;
                }
              }
            }
            if(InputByte[0]==20){
              br=true;
              Prn(OUT, 1," too long");
            }
          }
        }
    }
  }

  // Serial.println();
  // for (int i=1; i<InputByte[0]+1; i++){
    // Serial.write(InputByte[i]);
  // }
  // Serial.println();

  // Prn(OUT, 1, "out"+String(CompareInt) );
}

//-------------------------------------------------------------------------------------------------------
void EnterChar(int OUT){
  incomingByte = 0;
  unsigned long EnterTimeout=millis();   // abort input after 30s (avoid WDT reset)
  Prn(OUT, 0," > ");
  if(OUT==0){
    while (Serial.available() == 0) {
      esp_task_wdt_reset();
      if(millis()-EnterTimeout>30000){ break; }
    }
    if(Serial.available()>0){
      incomingByte = Serial.read();
    }
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      while(incomingByte==0){
        esp_task_wdt_reset();
        if(millis()-EnterTimeout>30000){ break; }
        if(TelnetServerClients[0].available()){
          incomingByte=TelnetServerClients[0].read();
        }
      }
      if(EnableSerialDebug>0){
        Serial.println();
        Serial.print("Telnet rx-");
        Serial.print(incomingByte, DEC);
      }
    }
  }
  Prn(OUT, 1, String(char(incomingByte)) );
}

//-------------------------------------------------------------------------------------------------------

void EnterInt(int OUT){
  incomingByte = 0;
  unsigned long EnterTimeout=millis();   // abort input after 30s (avoid WDT reset)
  Prn(OUT, 0,"> ");
  if(OUT==0){
    while(!Serial.available()) {
      esp_task_wdt_reset();
      if(millis()-EnterTimeout>30000){ break; }
    }
    delay(3000);
    CompareInt = Serial.parseInt();
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      bool br=true;
      int intField[10];
      int count=0;

      while(incomingByte==0 && br==true){
        esp_task_wdt_reset();
        if(millis()-EnterTimeout>30000){ break; }
        if(TelnetServerClients[0].available()){
          incomingByte=TelnetServerClients[0].read();
          // out of 0-9
          if(incomingByte<48 || incomingByte>57){
            br=false;
            intField[count]=0;
            Prn(OUT, 1,"");
          }else{
            intField[count]=incomingByte-48;
            Prn(OUT, 0,String(intField[count]));
            count++;
            incomingByte=0;
          }
        }
      }

      count--;
      CompareInt=0;
      int i=1;
      while(count>-1){
        CompareInt=CompareInt+intField[count]*i;
        // Prn(OUT, 1, String(intField[count])+"*"+String(i)+"="+String(CompareInt) );
        i=i*10;
        count--;
      }
    }
  }
  // Prn(OUT, 1, "out"+String(CompareInt) );
}
//-------------------------------------------------------------------------------------------------------

void EnterIntOld(int OUT){
  Prn(OUT, 0,"> ");
  if(OUT==0){
    while(!Serial.available()) {
    }
    delay(3000);
    CompareInt = Serial.parseInt();
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      delay(5000);
      if(TelnetServerClients[0].available()){
        // while(TelnetServerClients[0].available()){
        //   Prn(OUT, 1, "4" );
        //   // incomingByte=TelnetServerClients[0].read();
        // }
        CompareInt = TelnetServerClients[0].parseInt();
      }
    }
  }
  Prn(OUT, 1, String(CompareInt) );
}

//-------------------------------------------------------------------------------------------------------
// --- debug ring buffer (web Debug tab) -------------------------------------------------------
// Captures Prn() output while EnableSerialDebug>0 into a small circular line buffer that
// /api/debug serves, so the serial debug stream is visible in the browser without a cable.
#define DBG_RING_LINES 50
#define DBG_RING_WIDTH 100
char dbgRing[DBG_RING_LINES][DBG_RING_WIDTH];
int  dbgRingHead = 0;     // next slot to write
int  dbgRingCount = 0;    // valid lines so far (<= DBG_RING_LINES)
char dbgCur[DBG_RING_WIDTH];
int  dbgCurPos = 0;       // build-up of the current (not yet newline-terminated) line

void dbgCapture(const String& s, int ln){
  if(EnableSerialDebug==0) return;
  for(unsigned int i=0;i<s.length();i++){
    char c=s[i];
    if(c=='\r'||c=='\n') continue;
    if(dbgCurPos < DBG_RING_WIDTH-1) dbgCur[dbgCurPos++]=c;
  }
  if(ln==1){
    dbgCur[dbgCurPos]='\0';
    strncpy(dbgRing[dbgRingHead], dbgCur, DBG_RING_WIDTH);
    dbgRing[dbgRingHead][DBG_RING_WIDTH-1]='\0';
    dbgRingHead=(dbgRingHead+1)%DBG_RING_LINES;
    if(dbgRingCount<DBG_RING_LINES) dbgRingCount++;
    dbgCurPos=0;
  }
}

void Prn(int OUT, int LN, String STR){
  dbgCapture(STR, LN);
  if(OUT==3){
    if(TelnetAuthorized==true){
      OUT=1;
    }else{
      OUT=0;
    }
  }

  if(OUT==0){
    Serial.print(STR);
    if(LN==1){
      Serial.println();
    }
  }else if(OUT==1){
    size_t len = STR.length()+1;
    // uint8_t sbuf[len];
    char sbuf[len];
    STR.toCharArray(sbuf, len);
    //push data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (TelnetServerClients[i] && TelnetServerClients[i].connected()){
        TelnetServerClients[i].write(sbuf, len);
        // delay(1);
        if(LN==1){
          TelnetServerClients[i].write(13); // CR
          TelnetServerClients[i].write(10); // LF
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------
void ListCommands(int OUT){
  // digitalWrite(EnablePin,1);
  if(OUT==0){
    Prn(OUT, 1,"");
    Prn(OUT, 1,"");
    Prn(OUT, 1," =============================================================");
    Prn(OUT, 1," Please copy and save the IP address, MAC and telnet acces KEY");
    Prn(OUT, 1,"");
      Prn(OUT, 1, "   "+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
      Serial.print("   ");
      Serial.println(MACString);
    Prn(OUT, 1,"");
    Prn(OUT, 1,"   [position]    key");
    Prn(OUT, 0," ");
    for(int i=0; i<10; i++){
      Prn(OUT, 0,"    ["+String(i*10+1)+"-"+String(i*10+10)+"]  ");
      if(i<9){
        Prn(OUT, 0," ");
      }
      for(int j=0; j<10; j++){
        Prn(OUT, 0, String(key[i*10+j]));
      }
      Prn(OUT, 1,"");
    }
    Prn(OUT, 1,"");
    Prn(OUT, 1," Then disconnect the USB, and log in using telnet");
    Prn(OUT, 1," More information https://remoteqth.com/w/doku.php?id=3d_print_wx_station#second_step_connect_remotely_via_ip");
    Prn(OUT, 1," =============================================================");
    Prn(OUT, 1,"");
  }
  else{
    #if defined(ETHERNET)
      Prn(OUT, 1,"");
      Prn(OUT, 1,"------------ DivaDroid International | WX station status  ------------");
      Prn(OUT, 0,"  http://");
      Prn(OUT, 1, String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
      Prn(OUT, 0,"  ETH: MAC ");
      Prn(OUT, 0, MACString +", " );
      // Prn(OUT, 0, String(ETH.macAddress()[0], HEX)+":"+String(ETH.macAddress()[1], HEX)+":"+String(ETH.macAddress()[2], HEX)+":"+String(ETH.macAddress()[3], HEX)+":"+String(ETH.macAddress()[4], HEX)+":"+String(ETH.macAddress()[5], HEX)+", " );
      Prn(OUT, 0, String(ETH.linkSpeed()) );
      Prn(OUT, 0,"Mbps");
      if (ETH.fullDuplex()) {
        Prn(OUT, 0,", FULL_DUPLEX ");
      }
    #else
      Prn(OUT, 0,"  ETHERNET OFF ");
    #endif
    #if defined(WIFI)
      Prn(OUT, 1,"  =================================");
      Prn(OUT, 0,"  http://");
      Prn(OUT, 1, String(WiFi.localIP()[0])+"."+String(WiFi.localIP()[1])+"."+String(WiFi.localIP()[2])+"."+String(WiFi.localIP()[3]) );
      Prn(OUT, 0,"  dBm: ");
      Prn(OUT, 1, String(WiFi.RSSI()) );
    #else
      Prn(OUT, 1,"  WIFI: OFF");
    #endif

    if(OUT==0){
      Prn(OUT, 1,"  Key for telnet access:");
      Prn(OUT, 0,"    ");
      for(int i=0; i<100; i++){
        Prn(OUT, 0, String(key[i]));
        if((i+1)%10==0){
          Prn(OUT, 0," ");
        }
      }
      Prn(OUT, 1,"");
    }
    // Prn(OUT, 1, "  ChipID: "+ChipidHex);

    Prn(OUT, 0,"  NTP UTC:");
    Prn(OUT, 1, UtcTime(1));
    Prn(OUT, 0,"  Uptime: ");
    if(millis() < 60000){
      Prn(OUT, 0, String(millis()/1000) );
      Prn(OUT, 1," second");
    }else if(millis() > 60000 && millis() < 3600000){
      Prn(OUT, 0, String(millis()/60000) );
      Prn(OUT, 1," minutes");
    }else if(millis() > 3600000 && millis() < 86400000){
      Prn(OUT, 0, String(millis()/3600000) );
      Prn(OUT, 1," hours");
    }else{
      Prn(OUT, 0, String(millis()/86400000) );
      Prn(OUT, 1," days");
    }

    Prn(OUT, 0,"  MqttSubscribe: "+String(mqtt_server_ip[0])+"."+String(mqtt_server_ip[1])+"."+String(mqtt_server_ip[2])+"."+String(mqtt_server_ip[3])+":"+String(MQTT_PORT)+"/");
      String topic = String(YOUR_CALL) + "/WX/sub";
      const char *cstr = topic.c_str();
      if(mqttClient.subscribe(cstr)==true){
        Prn(OUT, 1, String(cstr));
      }else{
        Prn(OUT, 1, "FALSE");
      }

    Prn(OUT, 0,"  Firmware: ");
    Prn(OUT, 0, String(REV));
    Prn(OUT, 0," | PCB: ");
    Prn(OUT, 0, String(HWREVsw));
    Prn(OUT, 0," sw | ");
    Prn(OUT, 0, String(HWREVpcb));
    Prn(OUT, 1," pcb");

    #if defined(OTAWEB)
      Prn(OUT, 0,"  Web setup http://");
      #if defined(ETHERNET)
        Prn(OUT, 0, String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]));
      #endif
      Prn(OUT, 1,"/setup");
      if(WebAuthEnabled){
        Prn(OUT, 0,"  Web auth ON | user "+String(WEB_AUTH_USER)+" | password ");
        Prn(OUT, 1, WebAuthPassword);
        Prn(OUT, 1,"      P  clear web password / disable auth (recovery)");
      }else{
        Prn(OUT, 1,"  Web auth OFF (set a web password in /setup to enable)");
      }
    #endif


    Prn(OUT, 1,"-----------------  Sensors  -----------------");
    #if HWREVsw==8
      int intBuf = analogRead(RainPin);
      Prn(OUT, 0,"  RainPin raw "+String(intBuf));
      if(intBuf<1000){
        Prn(OUT, 0," > false|");
      }else if(intBuf>=1000 && intBuf<=2000){
        Prn(OUT, 0," > true|");
      }else if(intBuf>2000){
        Prn(OUT, 1," > MAGNET not detected - check north orientation or position");
        Prn(OUT, 0,"  ");
      }
      Prn(OUT, 1,RainCountDayOfMonth+"th day Rain counter "+String(RainCount)+"|"+String(RainPulseToMM(RainCount))+"mm|"+String(mmInPulse)+" rain mm per pulse" );
    #endif

    #if HWREVsw==7
      Prn(OUT, 0,"  Rain1Pin ");
      Prn(OUT, 0,String(digitalRead(Rain1Pin)));
      Prn(OUT, 0,"|Rain2Pin ");
      Prn(OUT, 0,String(digitalRead(Rain2Pin)));
      // Prn(OUT, 0," | ButtonPin ");
      //   Prn(OUT, 1, String(digitalRead(ButtonPin)));
      if(digitalRead(Rain1Pin)==digitalRead(Rain2Pin)){
        Prn(OUT, 1," > MAGNET not detected - check north orientation or position");
        Prn(OUT, 0,"  ");
      }else{
        Prn(OUT, 1,RainCountDayOfMonth+"th day Rain counter "+String(RainCount)+"|"+String(RainPulseToMM(RainCount))+"mm|"+String(mmInPulse)+" rain mm per pulse" );
      }
    #endif
    // if(EnableSerialDebug>0){
    // }else{
    // }
    Azimuth();
    Prn(3, 0,"  Wind direction ");
    Prn(3, 0,String(Azimuth(),BIN));
    if(Azimuth()==0x00){
      Prn(OUT, 1,"|MAGNET not detected - check north orientation" );
    }
    Prn(OUT, 1, "|"+String(WindDir)+"° [with shift "+String(WindDirShift)+"°]");
    Prn(OUT, 0,"  RpmPin ");
      Prn(OUT, 0,String(digitalRead(RpmPin)));
    Prn(OUT, 1, "|Wind speed last   "+String(RpmPulse)+" ms|"+String(PulseToMetterBySecond(RpmPulse))+" m/s|"+String(PulseToMetterBySecond(RpmPulse)*3.6)+" km/h");
    Prn(OUT, 1, "             avg ("+String(RpmAverage[1])+"/"+String(RpmAverage[0])+") "+String(AvgRpmPulse())+" ms|"+String(PulseToMetterBySecond(AvgRpmPulse()))+" m/s|"+String(PulseToMetterBySecond(AvgRpmPulse())*3.6)+" km/h");
    Prn(OUT, 1, "             MAX in period   "+String(PeriodMinRpmPulse)+" ms|"+String(PulseToMetterBySecond(PeriodMinRpmPulse))+" m/s|"+String(PulseToMetterBySecond(PeriodMinRpmPulse)*3.6)+" km/h("+String(WindSpeedMaxPeriodUTC)+")");
    Prn(OUT, 1, "             lifetime MAX    "+String(MinRpmPulse)+" ms|"+String(PulseToMetterBySecond(MinRpmPulse))+" m/s|"+String(PulseToMetterBySecond(MinRpmPulse)*3.6)+" km/h("+String(MinRpmPulseTimestamp)+")");
    #if defined(HTU21D)
      if(HTU21Denable==true){
        Prn(OUT, 0, "  HTU21D Humidity relative "+String(constrain(htu.readHumidity(), 0, 100))+"% | "+String(htu.readTemperature()+TempCal)+"°C ["+String(htu.readTemperature())+"°C raw]");
        if(ExtTemp==false){
          Prn(OUT, 1, " <- master");
        }else{
          Prn(OUT, 1, "");
        }
        Prn(OUT, 1, "  Dew point "+String( (htu.readTemperature()+TempCal) - (100.0 - constrain(htu.readHumidity(), 0, 100)) / 5.0)+"°C");
      }else{
        Prn(OUT, 1, "  HTU21D Humidity relative n/a");
      }
    #endif
    #if defined(SHT21)
      internal.read();
      // external.read();
      Serial.print("  SHT21 Humidity relative ");
      Serial.print(internal.getTemperature(), 1);
      Serial.print("\t");
      Serial.println(internal.getHumidity(), 1);
      // Serial.print("\t");
      // Serial.print(external.getTemperature(), 1);
      // Serial.print("\t");
      // Serial.println(external.getHumidity(), 1);
    #endif
    #if defined(BMP280)
      if(BMP280enable==true){
        if(ExtTemp==true){
          sensors.requestTemperatures();
          TemperatureCelsiusDS18B20 = sensors.getTempC(insideThermometer);
          Prn(OUT, 1, "  BMP280 Pressure "+String(Babinet(double(bmp.readPressure()), double((TemperatureCelsiusDS18B20+TempCal)*1.8+32))/100)+" hPa ["+String(bmp.readPressure()/100)+" raw] "+String(bmp.readTemperature()+TempCal)+"°C ["+String(bmp.readTemperature())+"°C raw]");
        }else{
          Prn(OUT, 1, "  BMP280 Pressure "+String(Babinet(double(bmp.readPressure()), double((htu.readTemperature()+TempCal)*1.8+32))/100)+" hPa ["+String(bmp.readPressure()/100)+" raw] "+String(bmp.readTemperature()+TempCal)+"°C ["+String(bmp.readTemperature())+"°C raw]");
        }
      }else{
        Prn(OUT, 1, "  BMP280 Pressure n/a");
      }
    #endif
    #if defined(DS18B20)
      if(ExtTemp==true){
        if(OUT==0){
          Serial.flush();
          Serial.end();
        }
        sensors.requestTemperatures();
        // float temperatureC = sensors.getTempCByIndex(0);
        float temperatureC = sensors.getTempC(insideThermometer);
        if(OUT==0){
          Serial.begin(SERIAL_BAUDRATE);
          while(!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
          }
        }
        Prn(OUT, 1, "  DS18B20 Temperature "+String(temperatureC+TempCal)+"°C ["+String(temperatureC)+"°C raw] <- master");
      }
    #endif
    #if defined(RF69_EXTERNAL_SENSOR)
      if(RF69enable==true){
        Prn(OUT, 1, "  RF sensor: "+String(temp_radio)+"°C, "+String(humidity_radio)+"%, "+String(vbat_radio)+"V");
      }
    #endif

    // Prn(OUT, 1, "  Humidity "+String()+"%");
    // if(cardType!=CARD_NONE){
    //   uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    //   // Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
    //   Prn(OUT, 1," | size "+String(printf("%lluMB", cardSize)));
    // }
    Prn(OUT, 1,"------------------  SETTINGS | press key to select -------------------");
    Prn(OUT, 1,"      ?  list refresh");
    Prn(OUT, 0,"      m  Altitude [");
    Prn(OUT, 0, String(Altitude)+" m]");
    if(Altitude==0){
      Prn(OUT, 0, " PLEASE SET ALTITUDE for conversion pressure to sea level");
    }
    Prn(OUT, 1, "");
    Prn(OUT, 0,"      L  change ");
      if(AprsON==true){
        Prn(OUT, 1,"CALLSIGN with ssid 6-4 | "+YOUR_CALL);
      }else{
        Prn(OUT, 1,"location name | "+YOUR_CALL);
      }

    Prn(OUT, 1,"");

    Prn(OUT, 0,"      R  rain mm per pulse [");
    Prn(OUT, 1, String(mmInPulse)+"mm] default 0.13 mm");
    Prn(OUT, 0,"      S  Wind direction shift [");
    Prn(OUT, 1, String(WindDirShift)+"°]");
    Prn(OUT, 0,"      C  Temperature calibration shift [");
    Prn(OUT, 1, String(TempCal)+"°C]");
    Prn(OUT, 1,"      x  TX repeat time ["+String(MeasureTimer[1]/60000)+" min]");

    // #if defined(DS18B20)
    //   // Prn(OUT, 1,"      s  scan DS18B20 1wire temperature sensor");
    //   Prn(OUT, 0,"      s  external temperature Sensor DS18B20 [O");
    //     if(ExtTemp==true){
    //       Prn(OUT, 1,"N]");
    //     }else{
    //       Prn(OUT, 1,"FF] CONNECT SENSOR (used inaccurate internal)");
    //     }
    // #endif
    // Prn(OUT, 0,"      a  speed Alert [");
    // Prn(OUT, 1, String(PulseToMetterBySecond(SpeedAlertLimit_ms))+" m/s] - not implemented");
    Prn(OUT, 1, "      +  change MQTT broker IP | "+String(mqtt_server_ip[0])+"."+String(mqtt_server_ip[1])+"."+String(mqtt_server_ip[2])+"."+String(mqtt_server_ip[3])+":"+String(MQTT_PORT));
    #if defined(WINDY)
      Prn(OUT, 0,"      K  windy.com API key [");
      if(WindyApiKey.length()>0){
        Prn(OUT, 1, "SET, upload ON]");
      }else{
        Prn(OUT, 1, "empty, upload OFF]");
      }
      Prn(OUT, 0,"      I  windy.com Station ID (web link) [");
      if(WindyStationId.length()>0){
        Prn(OUT, 1, WindyStationId+"]");
      }else{
        Prn(OUT, 1, "empty]");
      }
    #endif
    // Prn(OUT, 0, String(mqtt_server_ip));
    // Prn(OUT, 0, ":");
    // Prn(OUT, 1, String(MQTT_PORT));
    Prn(OUT, 0,"      A  APRS [");
      if(AprsON==true){
        Prn(OUT, 1,"ON]");
      }else{
        Prn(OUT, 1,"OFF]");
      }
      if(AprsON==true){
        // Prn(OUT, 0,"         i  change APRS server ip:port | ");
        // Prn(OUT, 1,String(aprs_server_ip[0])+"."+String(aprs_server_ip[1])+"."+String(aprs_server_ip[2])+"."+String(aprs_server_ip[3])+":"+String(AprsPort));
        Prn(OUT, 0,"         p  change APRS password | ");
        for (int i=208; i<213; i++){
          if(EEPROM.read(i)!=0xff){
            Prn(OUT, 0,"*");
          }
        }
        Prn(OUT, 1,"");
        Prn(OUT, 0,"         c  change APRS coordinate | ");
        Prn(OUT, 1,AprsCoordinates);
      }
    if(TelnetServerClients[0].connected()){
      Prn(OUT, 0,"      q  disconnect and close telnet [verified IP ");
      Prn(OUT, 0, String(TelnetServerClientAuth[0])+"."+String(TelnetServerClientAuth[1])+"."+String(TelnetServerClientAuth[2])+"."+String(TelnetServerClientAuth[3]) );
      Prn(OUT, 1,"]");
      Prn(OUT, 1,"      Q  logout with erase your verified IP from memory and close telnet");
    }else{
      Prn(OUT, 1,"      E  erase whole eeprom (telnet key also)");
      // Prn(OUT, 1,"      C  eeprom commit");
      Prn(OUT, 1,"      /  list directory");
      Prn(OUT, 1,"      R  read log file");
    }
    Prn(OUT, 1,"      e  list EEPROM");
    #if !defined(BMP280) && !defined(HTU21D)
      Prn(OUT, 1,"      2  I2C scanner");
    #endif
    Prn(OUT, 1,"      .  reset timer and send measure");
    Prn(OUT, 1,"      W  erase wind speed max memory");
    Prn(OUT, 0,"      *  terminal debug ");
      if(EnableSerialDebug==0){
        Prn(OUT, 1,"[OFF]");
      }else if(EnableSerialDebug==1){
        Prn(OUT, 1,"[ON]");
      }else if(EnableSerialDebug==2){
        Prn(OUT, 1,"[ON-frenetic]");
      }
    Prn(OUT, 1,"      @  restart device");
    if(HWREVpcb==0){
      Prn(OUT, 1,"      H  *** DEFINE VERSION OF PCB ***");
    }
    // Prn(OUT, 1,"---------------------------------------------");
    Prn(OUT, 0, " > " );
  }
  // digitalWrite(EnablePin,0);
}

//-------------------------------------------------------------------------------------------------------
float RainPulseToMM(int PULSE){
  if(PULSE>20000 || PULSE==0){
    return 0;
  }else{
    float mm;
    mm = PULSE*mmInPulse;
    return mm;
  }
}

//-------------------------------------------------------------------------------------------------------
char RandomChar(){
    int R = random(48, 122);
    if(R>=58 && 64>=R){
      R=R-random(7, 10);
    }
    if(R>=91 && 96>=R){
      R=R+random(6, 26);
    }
    return char(R);
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//-------------------------------------------------------------------------------------------------------
// ===== Web UI (:80) =====================================================================
// Pages live gzipped in SPIFFS (data/), uploaded via OTA filesystem image. If SPIFFS is
// missing (firmware flashed but FS not yet uploaded) a minimal PROGMEM page is served so
// the device never looks bricked.

static const char ROOT_FALLBACK_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WX</title><style>body{font-family:sans-serif;background:#333;color:#ddd;text-align:center;padding:2em}
a.btn{display:inline-block;margin:1em .5em;padding:.7em 1.4em;background:#06c;color:#fff;text-decoration:none;border-radius:6px}</style>
</head><body>
<h2>WX station</h2>
<p>Web pages are not in flash yet. Upload <code>spiffs.bin</code> via the OTA <b>Filesystem</b> image.</p>
<p><a class="btn" href="http://:82/update">OTA</a></p>
</body></html>)rawliteral";

// Map a file extension to a Content-Type for static SPIFFS serving.
String webContentType(const String& path){
  String p = path;
  if(p.endsWith(".gz")) p = p.substring(0, p.length()-3);   // negotiate on the underlying type
  if(p.endsWith(".html") || p.endsWith(".htm")) return "text/html";
  if(p.endsWith(".css"))  return "text/css";
  if(p.endsWith(".js"))   return "application/javascript";
  if(p.endsWith(".json")) return "application/json";
  if(p.endsWith(".svg"))  return "image/svg+xml";
  if(p.endsWith(".png"))  return "image/png";
  if(p.endsWith(".ico"))  return "image/x-icon";
  return "text/plain";
}

// Stream a SPIFFS file if present, preferring a .gz sibling (sets Content-Encoding: gzip).
// Returns false if neither <path> nor <path>.gz exists (so caller can fall back / 404).
bool streamSpiffsFile(const String& path){
  if(!FsMounted) return false;
  String gz = path + ".gz";
  if(SPIFFS.exists(gz)){
    File f = SPIFFS.open(gz, "r");
    if(!f) return false;
    // WebServer::streamFile() auto-adds "Content-Encoding: gzip" because the file name ends in
    // ".gz" (see core _streamFileCore). Do NOT set it ourselves or the browser sees it twice
    // and rejects the body ("unsupported content encoding").
    webserver.streamFile(f, webContentType(path));
    f.close();
    return true;
  }
  if(SPIFFS.exists(path)){
    File f = SPIFFS.open(path, "r");
    if(!f) return false;
    webserver.streamFile(f, webContentType(path));
    f.close();
    return true;
  }
  return false;
}

void handleRoot(){
  if(streamSpiffsFile("/index.html")) return;
  webserver.send_P(200, "text/html", ROOT_FALLBACK_HTML);   // FS not uploaded yet
}

void handleStatic(){
  String path = webserver.uri();
  if(path.endsWith("/")) path += "index.html";
  if(streamSpiffsFile(path)) return;
  // extensionless pretty URL (e.g. /wall -> /wall.html)
  int slash = path.lastIndexOf('/');
  if(path.indexOf('.', slash) < 0 && streamSpiffsFile(path + ".html")) return;
  webserver.send(404, "text/plain", "404: not found");
}

// GET /api/live - last measured values from the measurement cycle (never reads sensors here,
// so the poll rate cannot stall wind/temperature sampling). Hand-built JSON, no library.
void handleApiLive(){
  String j;
  j.reserve(512);
  j  = "{";
  j += "\"rev\":\"" + String(REV) + "\",";
  j += "\"call\":\"" + YOUR_CALL + "\",";
  j += "\"utc\":\"" + UtcTime(1) + "\",";
  j += "\"tempC\":" + String(TemperatureCelsius, 1) + ",";
  j += "\"hum\":" + String(HumidityRelPercent, 0) + ",";
  j += "\"dewC\":" + String(DewPointCelsius, 1) + ",";
  j += "\"pressHPa\":" + String(PressureHPA, 1) + ",";
  j += "\"windDir\":" + String(WindDir) + ",";
  j += "\"windAvgMs\":" + String(WindSpeedAvgMPS, 1) + ",";
  j += "\"windMaxMs\":" + String(WindRollMaxMps, 1) + ",";        // rolling 5-min max (dashboard)
  j += "\"periodMaxUtc\":\"" + WindRollMaxUtc + "\",";
  j += "\"lastMs\":" + String(WindNowMps(), 1) + ",";
  j += "\"rainMM\":" + String(RainTodayMM, 2) + ",";
  j += "\"rainCount\":" + String(RainCount) + ",";
  // which sensor cards are meaningful on this unit
  #if defined(HTU21D)
    j += "\"hasHum\":" + String(HTU21Denable ? "true" : "false") + ",";
  #else
    j += "\"hasHum\":false,";
  #endif
  #if defined(BMP280)
    j += "\"hasPress\":" + String(BMP280enable ? "true" : "false") + ",";
  #else
    j += "\"hasPress\":false,";
  #endif
  #if defined(WINDY)
    j += "\"windyId\":\"" + WindyStationId + "\",";
  #else
    j += "\"windyId\":\"\",";
  #endif
  j += "\"aprs\":" + String(AprsON ? "true" : "false");
  j += "}";
  webserver.sendHeader("Cache-Control", "no-store");
  webserver.send(200, "application/json", j);
}

// GET /api/wind - small, frequently polled endpoint so the dashboard tracks wind dynamics. Wind
// direction is read live from the shift register here (Azimuth() runs on core 1 like every HTTP
// handler, so there is no race with the ISR or WindTask); the speed values come from the cache
// WindTask refreshes every 250 ms. lastMs is the instantaneous gust so a spike shows up at once.
void handleApiWind(){
  WindDir = Azimuth();   // refresh direction live (side effect: updates the global via AzShift)
  String j; j.reserve(160);
  j  = "{";
  j += "\"windDir\":" + String(WindDir) + ",";
  j += "\"windAvgMs\":" + String(WindSpeedAvgMPS, 1) + ",";
  j += "\"windMaxMs\":" + String(WindRollMaxMps, 1) + ",";        // rolling 5-min max (dashboard)
  j += "\"periodMaxUtc\":\"" + WindRollMaxUtc + "\",";
  j += "\"lastMs\":" + String(WindNowMps(), 1);
  j += "}";
  webserver.sendHeader("Cache-Control", "no-store");
  webserver.send(200, "application/json", j);
}

// GET /api/status - full sensor + raw debug dump for the /setup Status tab, mirroring the telnet
// status block (RainPin ADC, wind direction register, RPM pulse periods + sentinels, raw vs
// calibrated temps, raw barometric pressure). Polled only while the Status tab is open. I2C sensors
// are read live like telnet; DS18B20 uses the cached value (a live read blocks ~750 ms). Auth-gated.
void handleApiStatus(){
  if(!webAuthOK()) return;
  String j; j.reserve(768);
  j  = "{";
  // --- wind direction ---
  j += "\"windDirRaw\":" + String(Azimuth()) + ",";          // 8-bit shift-register read (binary in UI)
  j += "\"windDir\":" + String(WindDir) + ",";
  j += "\"windDirShift\":" + String(WindDirShift) + ",";
  // --- wind speed (pulse period in ms; 987654321 = no pulse / sentinel) ---
  j += "\"rpmPin\":" + String(digitalRead(RpmPin)) + ",";
  j += "\"lastPulseMs\":" + String(RpmPulse) + ",";
  j += "\"lastMps\":" + String(PulseToMetterBySecond(RpmPulse), 2) + ",";
  j += "\"avgPulseMs\":" + String(AvgRpmPulse()) + ",";
  j += "\"avgMps\":" + String(PulseToMetterBySecond(AvgRpmPulse()), 2) + ",";
  j += "\"avgSum\":" + String(RpmAverage[1]) + ",";
  j += "\"avgCount\":" + String(RpmAverage[0]) + ",";
  j += "\"periodMaxPulseMs\":" + String(PeriodMinRpmPulse) + ",";
  j += "\"periodMaxMps\":" + String(PulseToMetterBySecond(PeriodMinRpmPulse), 2) + ",";
  j += "\"periodMaxUtc\":\"" + WindSpeedMaxPeriodUTC + "\",";
  j += "\"lifeMaxPulseMs\":" + String(MinRpmPulse) + ",";
  j += "\"lifeMaxMps\":" + String(PulseToMetterBySecond(MinRpmPulse), 2) + ",";
  j += "\"lifeMaxUtc\":\"" + MinRpmPulseTimestamp + "\",";
  // --- rain ---
  #if HWREVsw==8
    j += "\"rainPinRaw\":" + String(analogRead(RainPin)) + ",";  // ADC: <1000 false, 1000-2000 true, >2000 no magnet
  #else
    j += "\"rainPinRaw\":-1,";
  #endif
  j += "\"rainDay\":\"" + RainCountDayOfMonth + "\",";
  j += "\"rainCount\":" + String(RainCount) + ",";
  j += "\"rainMM\":" + String(RainPulseToMM(RainCount), 2) + ",";
  j += "\"mmInPulse\":" + String(mmInPulse) + ",";
  j += "\"tempCal\":" + String(TempCal, 2) + ",";
  // --- HTU21D humidity/temperature (live, raw = without TempCal) ---
  #if defined(HTU21D)
    if(HTU21Denable){
      float htuHum = constrain(htu.readHumidity(), 0, 100);
      float htuTemp = htu.readTemperature();
      j += "\"hasHum\":true,";
      j += "\"htuHum\":" + String(htuHum, 2) + ",";
      j += "\"htuTempRaw\":" + String(htuTemp, 2) + ",";
      j += "\"dewC\":" + String((htuTemp+TempCal) - (100.0 - htuHum) / 5.0, 2) + ",";
    }else{ j += "\"hasHum\":false,"; }
  #else
    j += "\"hasHum\":false,";
  #endif
  // --- BMP280 pressure/temperature (live; raw barometric + Babinet sea-level) ---
  #if defined(BMP280)
    if(BMP280enable){
      double bmpPress = bmp.readPressure();
      float bmpTemp = bmp.readTemperature();
      #if defined(DS18B20)
        float refTempF = (ExtTemp ? (TemperatureCelsiusDS18B20+TempCal) : (bmpTemp+TempCal))*1.8+32;
      #else
        float refTempF = (bmpTemp+TempCal)*1.8+32;
      #endif
      j += "\"hasPress\":true,";
      j += "\"bmpPressSea\":" + String(Babinet(bmpPress, refTempF)/100, 2) + ",";
      j += "\"bmpPressRaw\":" + String(bmpPress/100, 2) + ",";
      j += "\"bmpTempRaw\":" + String(bmpTemp, 2) + ",";
    }else{ j += "\"hasPress\":false,"; }
  #else
    j += "\"hasPress\":false,";
  #endif
  // --- DS18B20 external temperature (cached, raw = without TempCal) ---
  #if defined(DS18B20)
    j += "\"hasDs18\":" + String(ExtTemp ? "true" : "false") + ",";
    j += "\"ds18TempRaw\":" + String(TemperatureCelsiusDS18B20, 2) + ",";
  #else
    j += "\"hasDs18\":false,\"ds18TempRaw\":0,";
  #endif
  j += "\"uptimeSec\":" + String(millis()/1000);
  j += "}";
  webserver.sendHeader("Cache-Control", "no-store");
  webserver.send(200, "application/json", j);
}

// GET /api/wallcfg - WebSocket broker URI + station topic for the MQTT wall page (data/wall.html).
// Mirrors what the old built-in :80 mqtt-wall page generated inline.
void handleApiWallCfg(){
  String uri = "ws://" + String(mqtt_server_ip[0]) + "." + String(mqtt_server_ip[1]) + "."
             + String(mqtt_server_ip[2]) + "." + String(mqtt_server_ip[3]) + ":1884/";
  String topic = YOUR_CALL + "/WX/#";
  String j = "{\"uri\":\"" + uri + "\",\"topic\":\"" + topic + "\"}";
  webserver.sendHeader("Cache-Control", "no-store");
  webserver.send(200, "application/json", j);
}

// Minimal JSON string escaping for the few user strings we emit (callsign, coordinates, ...).
String jsonEsc(const String& s){
  String o; o.reserve(s.length()+4);
  for(unsigned int i=0;i<s.length();i++){
    char c=s[i];
    if(c=='"'||c=='\\') { o+='\\'; o+=c; }
    else if(c=='\n') o+="\\n";
    else if(c=='\r') {}
    else if((unsigned char)c < 0x20) {}
    else o+=c;
  }
  return o;
}

// Write a fixed EEPROM string field: first s.length() bytes, rest padded 0xff (the read loops stop
// on 0xff). Matches the existing telnet write convention (e.g. YOUR_CALL 141-160).
void eepromWriteString(int start, int maxlen, const String& s){
  for(int i=0;i<maxlen;i++){
    if((unsigned int)i < s.length()) EEPROM.write(start+i, s[i]);
    else EEPROM.write(start+i, 0xff);
  }
}

// Basic-auth gate for the settings endpoints. Enforced only when a web password is set
// (WebAuthEnabled). Public read endpoints (/, /api/live, /api/wallcfg, static) do not call this.
bool webAuthOK(){
  #if defined(OTAWEB)
    if(WebAuthEnabled && !webserver.authenticate(WEB_AUTH_USER, WebAuthPassword.c_str())){
      webserver.requestAuthentication();
      return false;
    }
  #endif
  return true;
}

// GET /setup - serve the settings page, auth-gated. Gating the page (not just /api/config) makes the
// browser show the Basic-auth prompt on navigation and then reuse the credentials for the api fetches.
void handleSetupPage(){
  if(!webAuthOK()) return;
  if(streamSpiffsFile("/setup.html")) return;
  webserver.send(404, "text/plain", "Missing /setup.html in SPIFFS");
}

// GET /api/debug - current debug level + recent captured lines (oldest first). Auth-gated.
void handleApiDebug(){
  if(!webAuthOK()) return;
  String j; j.reserve(DBG_RING_LINES*40);
  j = "{\"level\":" + String(EnableSerialDebug) + ",\"lines\":[";
  int start = (dbgRingHead - dbgRingCount + DBG_RING_LINES) % DBG_RING_LINES;
  for(int n=0; n<dbgRingCount; n++){
    int idx = (start + n) % DBG_RING_LINES;
    if(n) j += ",";
    j += "\"" + jsonEsc(String(dbgRing[idx])) + "\"";
  }
  j += "]}";
  webserver.sendHeader("Cache-Control", "no-store");
  webserver.send(200, "application/json", j);
}

// GET /api/config - everything the /setup page needs: editable settings (secrets returned only as a
// "set" flag, never the value) plus read-only diagnostics. Hand-built JSON.
void handleApiConfig(){
  if(!webAuthOK()) return;
  String j; j.reserve(900);
  j  = "{";
  // --- Stanice ---
  j += "\"altitude\":" + String(Altitude) + ",";
  j += "\"call\":\"" + jsonEsc(YOUR_CALL) + "\",";
  j += "\"rainMmPulse\":" + String(mmInPulse, 2) + ",";
  j += "\"windDirShift\":" + String(WindDirShift) + ",";
  j += "\"tempCal\":" + String(TempCal, 2) + ",";
  // --- Sit ---
  j += "\"mqttIp\":\"" + String(mqtt_server_ip[0]) + "." + String(mqtt_server_ip[1]) + "." + String(mqtt_server_ip[2]) + "." + String(mqtt_server_ip[3]) + "\",";
  j += "\"mqttPort\":" + String(MQTT_PORT) + ",";
  j += "\"aprsOn\":" + String(AprsON ? "true":"false") + ",";
  j += "\"aprsPassSet\":" + String(AprsPassword.length()>0 ? "true":"false") + ",";
  j += "\"aprsCoord\":\"" + jsonEsc(AprsCoordinates) + "\",";
  #if defined(WINDY)
    j += "\"windyKeySet\":" + String(WindyApiKey.length()>0 ? "true":"false") + ",";
    j += "\"windyId\":\"" + jsonEsc(WindyStationId) + "\",";
  #else
    j += "\"windyKeySet\":false,\"windyId\":\"\",";
  #endif
  // --- System ---
  j += "\"pcb\":" + String(HWREVpcb) + ",";
  j += "\"webAuthSet\":" + String(WebAuthPassword.length()>0 ? "true":"false") + ",";
  // --- Debug ---
  j += "\"debugLevel\":" + String(EnableSerialDebug) + ",";
  // --- Diagnostics (read-only) ---
  j += "\"rev\":\"" + String(REV) + "\",";
  j += "\"swRev\":" + String(HWREVsw) + ",";
  j += "\"uptimeSec\":" + String(millis()/1000) + ",";
  j += "\"mac\":\"" + jsonEsc(MACString) + "\",";
  #if defined(ETHERNET)
    j += "\"ip\":\"" + String(ETH.localIP()[0]) + "." + String(ETH.localIP()[1]) + "." + String(ETH.localIP()[2]) + "." + String(ETH.localIP()[3]) + "\",";
    j += "\"linkMbps\":" + String(ETH.linkSpeed()) + ",";
  #else
    j += "\"ip\":\"\",\"linkMbps\":0,";
  #endif
  j += "\"ntp\":\"" + jsonEsc(UtcTime(1)) + "\",";
  j += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
  j += "\"mqttUp\":" + String(mqttClient.connected() ? "true":"false") + ",";
  j += "\"mqttTopic\":\"" + jsonEsc(String(YOUR_CALL) + "/WX/sub") + "\",";
  j += "\"fsTotal\":" + String(FsMounted ? SPIFFS.totalBytes() : 0) + ",";
  j += "\"fsUsed\":" + String(FsMounted ? SPIFFS.usedBytes() : 0) + ",";
  // Real on-flash SPIFFS partition geometry (independent of mount state). Build the OTA spiffs.bin
  // with mkspiffs -s <fsPartSize>; OTA never rewrites the partition table, so this is the truth.
  {
    const esp_partition_t* _sp = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    j += "\"fsPartOff\":" + String(_sp ? (uint32_t)_sp->address : 0) + ",";
    j += "\"fsPartSize\":" + String(_sp ? (uint32_t)_sp->size : 0);
  }
  j += "}";
  webserver.sendHeader("Cache-Control", "no-store");
  webserver.send(200, "application/json", j);
}

// POST /api/config - one global Save from /setup. Applies+persists every provided field, then if a
// network field changed (MQTT broker/port, UDP port) reboots so it takes effect (mirrors telnet).
void handleApiConfigSave(){
  if(!webAuthOK()) return;
  bool reboot=false;
  String msg="saved";

  // --- Stanice ---
  if(webserver.hasArg("altitude")){
    int v=webserver.arg("altitude").toInt();
    if(v>=0 && v<=9000){ Altitude=v; EEPROM.writeInt(231, v); }
  }
  if(webserver.hasArg("call")){
    String v=webserver.arg("call"); if(v.length()>20) v=v.substring(0,20);
    YOUR_CALL=v; eepromWriteString(141, 20, v);
  }
  if(webserver.hasArg("rainMmPulse")){
    float v=webserver.arg("rainMmPulse").toFloat();
    if(v>=0 && v<=10){ mmInPulse=v; EEPROM.writeShort(5, (int)round(v*100)); }
  }
  if(webserver.hasArg("windDirShift")){
    int v=webserver.arg("windDirShift").toInt();
    if(v>=0 && v<=359){ WindDirShift=v; EEPROM.writeInt(240, v); }
  }
  if(webserver.hasArg("tempCal")){
    float v=webserver.arg("tempCal").toFloat();
    if(v>=-50 && v<=50){ TempCal=v; EEPROM.writeShort(2, (int)round(v*100)); }
  }
  // --- Sit (network fields require reboot) ---
  if(webserver.hasArg("mqttIp")){
    String s=webserver.arg("mqttIp"); int a=getValue(s,'.',0).toInt(),b=getValue(s,'.',1).toInt(),c=getValue(s,'.',2).toInt(),d=getValue(s,'.',3).toInt();
    if(a>=0&&a<=255&&b>=0&&b<=255&&c>=0&&c<=255&&d>=0&&d<=255){
      if(mqtt_server_ip[0]!=a||mqtt_server_ip[1]!=b||mqtt_server_ip[2]!=c||mqtt_server_ip[3]!=d) reboot=true;
      EEPROM.write(161,a); EEPROM.write(162,b); EEPROM.write(163,c); EEPROM.write(164,d);
    }
  }
  if(webserver.hasArg("mqttPort")){
    int v=webserver.arg("mqttPort").toInt();
    if(v>=1 && v<=65535){ if(MQTT_PORT!=v) reboot=true; EEPROM.writeInt(165, v); }
  }
  if(webserver.hasArg("aprsOn")){
    bool v=(webserver.arg("aprsOn")=="1"||webserver.arg("aprsOn")=="true");
    AprsON=v; EEPROM.write(199, v?1:0);
  }
  if(webserver.hasArg("aprsPass") && webserver.arg("aprsPass").length()>0){
    String v=webserver.arg("aprsPass"); if(v.length()>5) v=v.substring(0,5);
    AprsPassword=v; eepromWriteString(208, 5, v);
  }
  if(webserver.hasArg("aprsCoord")){
    String v=webserver.arg("aprsCoord"); if(v.length()>18) v=v.substring(0,18);
    AprsCoordinates=v; eepromWriteString(213, 18, v);
  }
  #if defined(WINDY)
  if(webserver.hasArg("windyKey") && webserver.arg("windyKey").length()>0){
    String v=webserver.arg("windyKey"); if(v.length()>123) v=v.substring(0,123);
    WindyApiKey=v; eepromWriteString(244, 123, v);
  }
  if(webserver.hasArg("windyId")){
    String v=webserver.arg("windyId"); if(v.length()>32) v=v.substring(0,32);
    WindyStationId=v; eepromWriteString(373, 32, v);
  }
  #endif
  // --- System ---
  // web Basic-auth password: set here to enable/change auth. Clearing/disabling is recovery-only
  // and lives in the serial/telnet menu ('P' command), since /setup is locked behind this password.
  // The vendored OTA copies the password at begin(), so a change needs a reboot to apply everywhere.
  #if defined(OTAWEB)
  if(webserver.hasArg("webPass") && webserver.arg("webPass").length()>0){
    String v=webserver.arg("webPass"); if(v.length()>64) v=v.substring(0,64);
    WebAuthPassword=v; WebAuthEnabled=true;
    eepromWriteString(405, 64, v); EEPROM.write(469, 1);
    reboot=true;
  }
  #endif

  // --- Debug (runtime only, not persisted - same as telnet '*') ---
  if(webserver.hasArg("debugLevel")){
    int v=webserver.arg("debugLevel").toInt();
    if(v>=0 && v<=2) EnableSerialDebug=v;
  }

  EEPROMcommit();

  if(reboot) msg="saved, rebooting to apply changes";
  String j = "{\"ok\":true,\"reboot\":" + String(reboot?"true":"false") + ",\"msg\":\"" + jsonEsc(msg) + "\"}";
  webserver.send(200, "application/json", j);

  if(reboot){
    delay(600);
    ESP.restart();
  }
}

//-------------------------------------------------------------------------------------------------------


void EthEvent(WiFiEvent_t event)
{
  switch (event) {
    // case SYSTEM_EVENT_ETH_START:
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH  Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    // case SYSTEM_EVENT_ETH_CONNECTED:
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH  Connected");
      break;
    // case SYSTEM_EVENT_ETH_GOT_IP:
    case ARDUINO_EVENT_ETH_GOT_IP:
      MACString = ETH.macAddress();
      MACString.toCharArray( MACchar, 18 );
      Serial.print("ETH  MAC: ");
      Serial.println(MACString);
      Serial.println("===============================");
      Serial.print("   IPv4: ");
      Serial.println(ETH.localIP());
      Serial.println("===============================");
      if (ETH.fullDuplex()) {
        Serial.print("FULL_DUPLEX, ");
      }
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;

      #if defined(MQTT)
        if (MQTT_ENABLE == true && MQTT_LOGIN == true){
          // if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)){
          //   AfterMQTTconnect();
          // }
        }else if(MQTT_ENABLE == true){
          Serial.print("EthEvent-mqtt ");

          mqttClient.setServer(mqtt_server_ip, MQTT_PORT);
          mqttClient.setCallback(MqttRx);
          lastMqttReconnectAttempt = 0;

          // EEPROM YOUR_CALL
          if(EEPROM.read(141)==0xff){
            YOUR_CALL=MACString;
            YOUR_CALL.remove(0, 12);
          }else{
            for (int i=141; i<161; i++){
              if(EEPROM.read(i)!=0xff){
                YOUR_CALL=YOUR_CALL+char(EEPROM.read(i));
              }
            }
          }

          char charbuf[50];
           // // memcpy( charbuf, ETH.macAddress(), 6);
           // ETH.macAddress().toCharArray(charbuf, 18);
           // // charbuf[6] = 0;
          if (mqttClient.connect(MACchar)){
            // Serial.println(charbuf);
            Prn(3, 1, String(MACchar));
            mqttReconnect();
            AfterMQTTconnect();
          }
        }
      #endif
      ListCommands(0);
      break;

    // case SYSTEM_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH  Disconnected");
      eth_connected = false;
      break;
    // case SYSTEM_EVENT_ETH_STOP:
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH  Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}
//-------------------------------------------------------------------------------------------------------
void Mqtt(){
  if (millis()-MqttStatusTimer[0]>MqttStatusTimer[1] && MQTT_ENABLE == true && eth_connected==1){
    if(!mqttClient.connected()){
      long now = millis();
      if (now - lastMqttReconnectAttempt > 5000) {
        lastMqttReconnectAttempt = now;
        if(EnableSerialDebug>0){
          Prn(3, 1, "Attempt to MQTT reconnect | "+String(millis()/1000) );
        }
        if (mqttReconnect()) {
          lastMqttReconnectAttempt = 0;
        }
      }
    }else{
      // Client connected
      mqttClient.loop();
    }
    MqttStatusTimer[0]=millis();
  }
}

//-------------------------------------------------------------------------------------------------------

bool mqttReconnect() {
  Prn(3, 0, "MQTT");
  char charbuf[50];
  // // memcpy( charbuf, ETH.macAddress(), 6);
  // ETH.macAddress().toCharArray(charbuf, 18);
  // charbuf[6] = 0;
  if (mqttClient.connect(MACchar)) {
    if(EnableSerialDebug>0){
      Prn(3, 0, "mqttReconnect-connected");
    }
    // IPAddress IPlocalAddr = ETH.localIP();                           // get
    // String IPlocalAddrString = String(IPlocalAddr[0]) + "." + String(IPlocalAddr[1]) + "." + String(IPlocalAddr[2]) + "." + String(IPlocalAddr[3]);   // to string
    // MqttPubStringQC(1, "IP", IPlocalAddrString, true);

    // resubscribe

    String topic = String(YOUR_CALL) + "/WX/get";
    const char *cstr = topic.c_str();
    if(mqttClient.subscribe(cstr)==true){
      if(EnableSerialDebug>0){
        Prn(3, 1, " > subscribe "+String(cstr));
      }
    }
  }
  return mqttClient.connected();
}

//------------------------------------------------------------------------------------
void MqttRx(char *topic, byte *payload, unsigned int length) {
  String CheckTopicBase;
  CheckTopicBase.reserve(100);
  // static bool HeardBeatStatus;
  if(EnableSerialDebug>0){
    Prn(3, 0, "RX MQTT ");
  }

    CheckTopicBase = String(YOUR_CALL) + "/WX/get";
    if ( CheckTopicBase.equals( String(topic) )){
      if(EnableSerialDebug>0){
        Prn(3, 1, "/get ");
      }
      Interrupts(false);
        MqttPubValue();
      Interrupts(true);
    }

} // MqttRx END

//-------------------------------------------------------------------------------------------------------
void AfterMQTTconnect(){
  #if defined(MQTT)
  //    if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)) {          // public IP addres to MQTT
        IPAddress IPlocalAddr = ETH.localIP();                           // get
        String IPlocalAddrString = String(IPlocalAddr[0]) + "." + String(IPlocalAddr[1]) + "." + String(IPlocalAddr[2]) + "." + String(IPlocalAddr[3]);   // to string
        IPlocalAddrString.toCharArray( mqttTX, 50 );                          // to array
        String path2 = String(YOUR_CALL) + "/WX/ip";
        path2.toCharArray( mqttPath, 100 );
        mqttClient.publish(mqttPath, mqttTX, true);
          Serial.print("MQTT-TX ");
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(mqttTX);

        // String MAClocalAddrString = ETH.macAddress();   // to string
        // MAClocalAddrString.toCharArray( mqttTX, 50 );                          // to array
        path2 = String(YOUR_CALL) + "/WX/mac";
        path2.toCharArray( mqttPath, 100 );
        mqttClient.publish(mqttPath, MACchar, true);
          Serial.print("MQTT-TX ");
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(MACchar);

        // String pcbString = String(HWREVsw);   // to string
        // pcbString.toCharArray( mqttTX, 2 );                          // to array
        // path2 = String(YOUR_CALL) + "/WX/pcb";
        // path2.toCharArray( mqttPath, 100 );
        //   mqttClient.publish(mqttPath, mqttTX, true);
            // Serial.print("MQTT-TX ");
            // Serial.print(mqttPath);
            // Serial.print(" ");
            // Serial.println(mqttTX);

    // MeasureTimer[0]=2800000;
    MeasureTimer[0]=millis()-MeasureTimer[1];

  #endif
}
//-----------------------------------------------------------------------------------
void MqttPubString(String TOPIC, String DATA, bool RETAIN){
  Interrupts(false);
  // reconnect is handled in Mqtt() - here only publish when connected
  if(mqttClient.connected()==true){
    String topic = String(YOUR_CALL) + "/WX/"+TOPIC;
    // publish directly from String, no fixed 50 char truncation
    mqttClient.publish(topic.c_str(), DATA.c_str(), RETAIN);
  }
  Interrupts(true);
}
//-----------------------------------------------------------------------------------

void testClient(const char * host, uint16_t port)
{
  Serial.print("\nETH  connecting to ");
  Serial.println(host);
  WiFiClient webClient;
  if (!webClient.connect(host, port)) {
    Serial.println("ETH  connection failed");
    return;
  }
  webClient.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (webClient.connected() && !webClient.available());
  while (webClient.available()) {
    Serial.write(webClient.read());
  }
  Serial.println("ETH closing connection\n");
  webClient.stop();
}

//-------------------------------------------------------------------------------------------------------
void TelnetAuth(){

  switch (TelnetAuthStep) {
    case 0: {
      if(TelnetLoginFails>=3 && millis()-TelnetLoginFailsBanTimer[0]<TelnetLoginFailsBanTimer[1]){
        Prn(1, 1,"");
        Prn(1, 0,"   Ten minutes login ban, PSE QRX ");
        Prn(1, 0,String((TelnetLoginFailsBanTimer[1]-(millis()-TelnetLoginFailsBanTimer[0]))/1000));
        Prn(1, 1," seconds");
        delay(3000);
        TelnetServerClients[0].stop();
        break;
      }else if(TelnetLoginFails>2 && millis()-TelnetLoginFailsBanTimer[0]>TelnetLoginFailsBanTimer[1]){
        TelnetLoginFails=0;
      }
      if(TelnetLoginFails<=3){
        Prn(1, 1,"Login? [y/n] ");
        TelnetAuthStep++;
        incomingByte=0;
      }
      break; }
    case 1: {
      // incomingByte=TelnetRX();
      if(incomingByte==121 || incomingByte==89){
        Prn(1, 1,String(char(incomingByte)));
        TelnetAuthStep++;
      }else if(incomingByte!=121 && incomingByte!=0){
        // TelnetServerClients[0].stop();
        TelnetAuthorized=false;
        TelnetAuthStep=0;
        // TelnetServerClientAuth = {0,0,0,0};
      }
      break; }
    case 2: {
      AuthQ(1, 0);
      TelnetAuthStepFails=0;
      break; }
    case 3: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(2, 0);
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(2, 1);
      }
      break; }
    case 4: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(3, 0);
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(3, 1);
      }
      break; }
    case 5: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(4, 0);
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(4, 1);
      }
      break; }
    case 6: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        TelnetAuthStep++;
        incomingByte=0;
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        TelnetAuthStep++;
        incomingByte=0;
        TelnetAuthStepFails++;
      }
      break; }
    case 7: {
      if(TelnetAuthStepFails==0){
        TelnetAuthorized = true;
        TelnetServerClientAuth = TelnetServerClients[0].remoteIP();
        Prn(1, 1,"Login OK");
        ListCommands(1);
        TelnetAuthStep++;
        incomingByte=0;
      }else{
        TelnetAuthorized = false;
        TelnetServerClientAuth = {0,0,0,0};
        Prn(1, 1,"Access denied");
        TelnetAuthStep=0;
        incomingByte=0;
        TelnetLoginFails++;
        TelnetLoginFailsBanTimer[0]=millis();
      }
      EEPROM.write(37, TelnetServerClientAuth[0]); // address, value
      EEPROM.write(38, TelnetServerClientAuth[1]); // address, value
      EEPROM.write(39, TelnetServerClientAuth[2]); // address, value
      EEPROM.write(40, TelnetServerClientAuth[3]); // address, value
      EEPROMcommit();
      break; }
  }
}

//-------------------------------------------------------------------------------------------------------

void AuthQ(int NR, bool BAD){
  Prn(1, 0,"What character is at ");
  // RandomNumber=random(0, strlen(key));
  RandomNumber=random(0, 99);
  Prn(1, 0, String(RandomNumber+1) );
  Prn(1, 0," position, in key? (");
  Prn(1, 0,String(NR));
  Prn(1, 1,"/4)");
  // Prn(1, 1, String(key[RandomNumber]) );
  TelnetAuthStep++;
  incomingByte=0;
  if(BAD==true){
    TelnetAuthStepFails++;
  }
}

//-------------------------------------------------------------------------------------------------------
void Telnet(){
  uint8_t i;
  // if (wifiMulti.run() == WL_CONNECTED) {
  if (eth_connected==true) {

    //check if there are any new clients
    if (TelnetServer.hasClient()){
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        //find free/disconnected spot
        if (!TelnetServerClients[i] || !TelnetServerClients[i].connected()){
          if(TelnetServerClients[i]) TelnetServerClients[i].stop();
          TelnetServerClients[i] = TelnetServer.available();
          if (!TelnetServerClients[i]) Serial.println("Telnet available broken");
          if(EnableSerialDebug>0){
            Serial.println();
            Serial.print("New Telnet client: ");
            Serial.print(i); Serial.print(' ');
            Serial.println(TelnetServerClients[i].remoteIP());
          }
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        TelnetServer.available().stop();
      }
    }

    //check clients for data
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (TelnetServerClients[i] && TelnetServerClients[i].connected()){
        if(TelnetServerClients[i].available()){
          //get data from the telnet client and push it to the UART
          // while(TelnetServerClients[i].available()) Serial_one.write(TelnetServerClients[i].read());
          if(EnableSerialDebug>0){
            Serial.println();
            Serial.print("TelnetRX ");
          }

          while(TelnetServerClients[i].available()){
            incomingByte=TelnetServerClients[i].read();
            // Serial_one.write(RX);
            if(EnableSerialDebug>0){
              // Serial.write(RX);
              Serial.print(char(incomingByte));
            }
          }
        }
      }else{
        if (TelnetServerClients[i]) {
          TelnetServerClients[i].stop();
          TelnetAuthorized=false;
          FirstListCommands=true;
          // TelnetServerClientAuth = {0,0,0,0};
        }
      }
    }

    //check UART for data
    // if(Serial_one.available()){
    //   size_t len = Serial_one.available();
    //   uint8_t sbuf[len];
    //   Serial_one.readBytes(sbuf, len);
    //   //push UART data to all connected telnet clients
    //   for(i = 0; i < MAX_SRV_CLIENTS; i++){
    //     if (TelnetServerClients[i] && TelnetServerClients[i].connected()){
    //       TelnetServerClients[i].write(sbuf, len);
    //       // delay(1);
    //       if(EnableSerialDebug>0){
    //         Serial.println();
    //         Serial.print("Telnet tx-");
    //         Serial.write(sbuf, len);
    //       }
    //     }
    //   }
    // }

  }else{
    // if(EnableSerialDebug>0){
    //   Serial.println("Telnet not connected!");
    // }
    for(i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (TelnetServerClients[i]) TelnetServerClients[i].stop();
    }
    delay(1000);
  }
}

//-------------------------------------------------------------------------------------------------------

void SerialToIp(){
  #if defined(Ser2net)
  uint8_t i;
  // if (wifiMulti.run() == WL_CONNECTED) {
  if (eth_connected==true) {
    //check if there are any new clients
    if (SerialServer.hasClient()){
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        //find free/disconnected spot
        if (!SerialServerClients[i] || !SerialServerClients[i].connected()){
          if(SerialServerClients[i]) SerialServerClients[i].stop();
          SerialServerClients[i] = SerialServer.available();
          if (!SerialServerClients[i]) Serial.println("available broken");
          if(EnableSerialDebug>0){
            Serial.println();
            Serial.print("New Ser2net client: ");
            Serial.print(i); Serial.print(' ');
            Serial.println(SerialServerClients[i].remoteIP());
          }
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        SerialServer.available().stop();
      }
    }
    //check clients for data
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (SerialServerClients[i] && SerialServerClients[i].connected()){
        if(SerialServerClients[i].available()){
          //get data from the telnet client and push it to the UART
          // while(SerialServerClients[i].available()) Serial_one.write(SerialServerClients[i].read());
          if(EnableSerialDebug>0){
            Serial.println();
            Serial.print("rx-");
          }

/*
Zapinaci pakety
< 2019/04/17 22:05:29.670771  length=1 from=2009 to=2009
 0d
< 2019/04/17 22:05:29.734563  length=3 from=2010 to=2012
 30 31 0d
> 2019/04/17 22:05:29.924267  length=2 from=3338 to=3339
 0d 0d
> 2019/04/17 22:05:29.972263  length=3 from=3340 to=3342
 30 31 0d
< 2019/04/17 22:05:30.086153  length=1 from=2013 to=2013
 0d
> 2019/04/17 22:05:30.249406  length=3 from=3343 to=3345
 30 31 0d
< 2019/04/17 22:05:30.261738  length=2 from=2014 to=2015
 ff 0d

Udrzovaci pakety
> 2019/04/17 21:59:00.549347  length=2 from=3319 to=3320
 ff 0d
< 2019/04/17 21:59:00.559100  length=2 from=1998 to=1999
 ff 0d
< 2019/04/17 21:59:02.555837  length=1 from=2000 to=2000
 0d
< 2019/04/17 21:59:04.568329  length=1 from=2001 to=2001
 0d
> 2019/04/17 21:59:05.549832  length=2 from=3321 to=3322
 ff 0d
< 2019/04/17 21:59:05.558790  length=2 from=2002 to=2003
 ff 0d

Vypinaci Paket
 < 2019/04/17 21:59:10.382425  length=3 from=2006 to=2008
 30 30 0d
> 2019/04/17 21:59:10.388125  length=15 from=3323 to=3337
 0d 0d 0d 0d 0d 0d 0d 0d 0d 0d 0d 0d 30 30 0d

*/


          while(SerialServerClients[i].available()){
            byte RX;
            RX=SerialServerClients[i].read();
            Serial_one.write(RX);
            if(EnableSerialDebug>0){
              Serial.write(RX);
            }
          }
        }
      }
      else {
        if (SerialServerClients[i]) {
          SerialServerClients[i].stop();
        }
      }
    }
    //check UART for data
    if(Serial_one.available()){
      size_t len = Serial_one.available();
      uint8_t sbuf[len];
      Serial_one.readBytes(sbuf, len);
      //push UART data to all connected telnet clients
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        if (SerialServerClients[i] && SerialServerClients[i].connected()){
          SerialServerClients[i].write(sbuf, len);
          // delay(1);
          if(EnableSerialDebug>0){
            Serial.println();
            Serial.print("tx-");
            Serial.write(sbuf, len);
          }
        }
      }
    }
  }
  else {
    if(EnableSerialDebug>0){
      Serial.println("Ser2net not connected!");
    }
    for(i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (SerialServerClients[i]) SerialServerClients[i].stop();
    }
    delay(1000);
  }
  #endif
}
//-------------------------------------------------------------------------------------------------------
// void bmp280(){
//   unsigned int b1[24];
//   unsigned int data[8];
//   for (int i = 0; i < 24; i++)
//   {
//     // Start I2C Transmission
//     Wire.beginTransmission(BMP280Addr);
//     // Select data register
//     Wire.write((136 + i));
//     // Stop I2C Transmission
//     Wire.endTransmission();
//
//     // Request 1 byte of data
//     Wire.requestFrom(BMP280Addr, 1);
//
//     // Read 1 byte of data
//     if (Wire.available() == 1)
//     {
//       b1[i] = Wire.read();
//     }
//   }
//   // Convert the data
//   // temp coefficients
//   unsigned int dig_T1 = (b1[0] & 0xFF) + ((b1[1] & 0xFF) * 256);
//   int dig_T2 = b1[2] + (b1[3] * 256);
//   int dig_T3 = b1[4] + (b1[5] * 256);
//
//   // pressure coefficients
//   unsigned int dig_P1 = (b1[6] & 0xFF) + ((b1[7] & 0xFF) * 256);
//   int dig_P2 = b1[8] + (b1[9] * 256);
//   int dig_P3 = b1[10] + (b1[11] * 256);
//   int dig_P4 = b1[12] + (b1[13] * 256);
//   int dig_P5 = b1[14] + (b1[15] * 256);
//   int dig_P6 = b1[16] + (b1[17] * 256);
//   int dig_P7 = b1[18] + (b1[19] * 256);
//   int dig_P8 = b1[20] + (b1[21] * 256);
//   int dig_P9 = b1[22] + (b1[23] * 256);
//
//   // Start I2C Transmission
//   Wire.beginTransmission(BMP280Addr);
//   // Select control measurement register
//   Wire.write(0xF4);
//   // Normal mode, temp and pressure over sampling rate = 1
//   Wire.write(0x27);
//   // Stop I2C Transmission
//   Wire.endTransmission();
//
//   // Start I2C Transmission
//   Wire.beginTransmission(BMP280Addr);
//   // Select config register
//   Wire.write(0xF5);
//   // Stand_by time = 1000ms
//   Wire.write(0xA0);
//   // Stop I2C Transmission
//   Wire.endTransmission();
//
//   for (int i = 0; i < 8; i++)
//   {
//     // Start I2C Transmission
//     Wire.beginTransmission(BMP280Addr);
//     // Select data register
//     Wire.write((247 + i));
//     // Stop I2C Transmission
//     Wire.endTransmission();
//
//     // Request 1 byte of data
//     Wire.requestFrom(BMP280Addr, 1);
//
//     // Read 1 byte of data
//     if (Wire.available() == 1)
//     {
//       data[i] = Wire.read();
//     }
//   }
//
//   // Convert pressure and temperature data to 19-bits
//   long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
//   long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;
//
//   // Temperature offset calculations
//   double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
//   double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
//                  (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
//   double t_fine = (long)(var1 + var2);
//   double cTemp = (var1 + var2) / 5120.0;
//   double fTemp = cTemp * 1.8 + 32;
//
//   // Pressure offset calculations
//   var1 = ((double)t_fine / 2.0) - 64000.0;
//   var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
//   var2 = var2 + var1 * ((double)dig_P5) * 2.0;
//   var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
//   var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
//   var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
//   double p = 1048576.0 - (double)adc_p;
//   p = (p - (var2 / 4096.0)) * 6250.0 / var1;
//   var1 = ((double) dig_P9) * p * p / 2147483648.0;
//   var2 = p * ((double) dig_P8) / 32768.0;
//   double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;
//
//   float height = 44330 * (1 - pow((pressure / 1013.25), 0.1903));
//   float h = height * 0.3048;
//
//   // Output data to serial monitor
//  Serial.print("Altitude : ");
//  Serial.print(height);
//  Serial.println(" m");
//  Serial.print("Altitude in Feet : ");
//  Serial.println(h);
//
//  Serial.print("Pressure : ");
//  Serial.print(pressure);
//  Serial.println(" hPa");
//  Serial.print("Temperature in Celsius : ");
//  Serial.print(cTemp);
//  Serial.println(" C");
//  Serial.print("Temperature in Fahrenheit : ");
//  Serial.print(fTemp);
//  Serial.println(" F");
//
//   //volatile float tempc, tempf, presr, altim, altif
//
//   // tempc = cTemp;
//   // tempf = fTemp;
//   // presr = pressure;
//   // altm = height;
//   // altf = h;
//   // delay(1000);
// }
//-------------------------------------------------------------------------------------------------------
// void printBME() {
//   Serial.print("Temperature = ");
//   Serial.print(bme.readTemperature());
//   Serial.println(" *C");
//
//   // Convert temperature to Fahrenheit
//   /*Serial.print("Temperature = ");
//   Serial.print(1.8 * bme.readTemperature() + 32);
//   Serial.println(" *F");*/
//
//   Serial.print("Pressure = ");
//   Serial.print(bme.readPressure() / 100.0F);
//   Serial.println(" hPa");
//
//   // Serial.print("Approx. Altitude = ");
//   // Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//   // Serial.println(" m");
//
//   Serial.println();
// }
//-------------------------------------------------------------------------------------------------------
void I2cScanner() {
  byte error, address;
  int nDevices;
  // Serial.println("Scanning...");
  Prn(3, 1,"Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      // Serial.print("I2C device found at address 0x");
      Prn(3, 0,"I2C device found at address 0x");
      if (address<16) {
        // Serial.print("0");
        Prn(3, 0,"0");
      }
      // Serial.println(address,HEX);
      Prn(3, 1,String(address,HEX));
      nDevices++;
    }
    else if (error==4) {
      // Serial.print("Unknow error at address 0x");
      Prn(3, 1,"Unknow error at address 0x");
      if (address<16) {
        // Serial.print("0");
        Prn(3, 0,"0");
      }
      // Serial.println(address,HEX);
      Prn(3, 1,String(address,HEX));
    }
  }
  if (nDevices == 0) {
    // Serial.println("No I2C devices found\n");
    Prn(3, 1,"No I2C devices found");
  }
  else {
    // Serial.println("done\n");
    Prn(3, 1,"done");
  }
  // delay(5000);
}

//-------------------------------------------------------------------------------------------------------
String UtcTime(int format){
  tm timeinfo;
  char buf[50]; //50 chars should be enough
  if (eth_connected==false) {
    strcpy(buf, "n/a");
  }else{
    if(!getLocalTime(&timeinfo, 0)){   // 0ms timeout - return immediately, never block the loop
      strcpy(buf, "n/a");
    }else{
      if(format==1){
        strftime(buf, sizeof(buf), "%Y-%b-%d %H:%M:%S", &timeinfo);
      }else if(format==2){
        strftime(buf, sizeof(buf), "%d", &timeinfo);
      }else if(format==3){
        strftime(buf, sizeof(buf), "%Y", &timeinfo);
      }
    }
  }
  // Serial.println(buf);
  return String(buf);
}

String Timestamp(){
  //time_t now;
  //time(&now);
  struct timeval tv;
  gettimeofday(&tv, NULL);
  char timestamp[40];
  // sprintf(timestamp,"%u.%06u",tv.tv_sec, tv.tv_usec);
  sprintf(timestamp,"%ld.%06ld",tv.tv_sec, tv.tv_usec);
  return String(timestamp);
}

//--SD-----------------------------------------------------------------------------------------------------

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

#if defined(DS18B20)
  // function to print a device address
  void printAddress(DeviceAddress deviceAddress)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      // zero pad the address if necessary
      if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
    }
  }

  // function to print the temperature for a device
  void printTemperature(DeviceAddress deviceAddress)
  {
    float tempC = sensors.getTempC(deviceAddress);
    if(tempC == DEVICE_DISCONNECTED_C)
    {
      Serial.println("DS18B20 Error: Could not read temperature data");
      return;
    }
    // Serial.print("Temp C: ");
    Serial.print(tempC);
  //  Serial.print(" Temp F: ");
  //  Serial.print(DallasTemperature::toFahrenheit(tempC));
  }

  // function to print a device's resolution
  void printResolution(DeviceAddress deviceAddress)
  {
    Serial.print("Resolution: ");
    Serial.print(sensors.getResolution(deviceAddress));
    Serial.println();
  }

  // main function to print information about a device
  void printData(DeviceAddress deviceAddress)
  {
    Serial.print("DS18B20 address: ");
    printAddress(deviceAddress);
    Serial.print(" ");
    printTemperature(deviceAddress);
    Serial.println();
  }
#endif


//-------------------------------------------------------------------------------------------------------
