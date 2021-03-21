/*

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

HARDWARE ESP32-POE

Changelog:
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
- add DNS
- telnet inactivity to close
- SD card log
- clear code

*/
//-------------------------------------------------------------------------------------------------------
#define OTAWEB                      // enable upload firmware via web
#define DS18B20                     // external 1wire Temperature sensor
#define BMP280                      // pressure I2C sensor
#define HTU21D                      // humidity I2C sensor
#define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
// #define WIFI                     // Enable ESP32 WIFI (DHCP IPv4) - NOT TESTED
const char* ssid     = "";
const char* password = "";
//-------------------------------------------------------------------------------------------------------
const char* REV = "20210321";

// values
const int keyNumber = 1;
char key[100];
String YOUR_CALL = "";
long MeasureTimer[2]={2800000,1800000};   //  1/2 hour
long RainTimer[2]={0,5000};   //  <---------------- rain timing
int RainCount;
String RainCountDayOfMonth;
bool RainStatus;
float mmInPulse = 0.154;        // rain mm in one rain pulse

unsigned int WindDir = 0;

long RpmTimer[2]={0,3000};
long RpmPulse = 0;
long PeriodMinRpmPulse = 987654321;
String PeriodMinRpmPulseTimestamp;
long MinRpmPulse;
String MinRpmPulseTimestamp;
unsigned int RpmSpeed = 0;
unsigned long RpmAverage[2]={1,0};

int SpeedAlertLimit_ms = 0;
int SpeedAlert_ms = 3000;
// bool NeedSpeedAlert_ms = false;
long AlertTimer[2]={0,60000};
//  |alert...........|alert........... everry max 1 minutes and with publish max value in period

// 73 seconds WDT (WatchDogTimer)
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 73
long WdtTimer=0;

byte InputByte[21];
// #define Ser2net                  // Serial to ip proxy - DISABLE if board revision 0.3 or lower
#define EnableOTA                // Enable flashing ESP32 Over The Air
int NumberOfEncoderOutputs = 8;  // 2-16
byte NET_ID = 0x00;              // Unique ID number [0-F] hex format
int EnableSerialDebug     = 0;
long FreneticModeTimer ;
#define HTTP_SERVER_PORT  80     // Web server port
int IncomingSwitchUdpPort;
#define ShiftOut                 // Enable ShiftOut register
#define UdpAnswer                // Send UDP answer confirm packet
int BroadcastPort;               // destination broadcast packet port
bool EnableGroupPrefix = 0;      // enable multi controller control
bool EnableGroupButton = 0;      // group to one from
unsigned int GroupButton[8]={1,2,3,4,5,6,7,8};
byte DetectedRemoteSw[16][4];
unsigned int DetectedRemoteSwPort[16];

const int SERIAL_BAUDRATE = 115200; // serial debug baudrate
int SERIAL1_BAUDRATE; // serial1 to IP baudrate
int incomingByte = 0;   // for incoming serial data

int i = 0;
#include <WiFi.h>
#include <WiFiUdp.h>
#include "EEPROM.h"
#define EEPROM_SIZE 240   /*
0    -listen source
1    -net ID
2    -encoder range
3    -HW_BCD_SW
4    -EnableGroupPrefix
5    -EnableGroupButton
6-13 -GroupButton
14-17  - SERIAL1_BAUDRATE
18-21 - SerialServerIPport
22-25 - IncomingSwitchUdpPort
26-29 - RebootWatchdog
30-33 - OutputWatchdog
34    - Bank0 storage
35    - Bank1 storage
36    - Bank2 storage
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

!! Increment EEPROM_SIZE #define !!

*/
int Altitude = 0;
bool needEEPROMcommit = false;
unsigned int RebootWatchdog;
unsigned int OutputWatchdog;
unsigned long WatchdogTimer=0;

WiFiServer server(HTTP_SERVER_PORT);
bool DHCP_ENABLE = 1;
// Client variables
char linebuf[80];
int charcount=0;
//Are we currently connected?
boolean connected = false;
//The udp library class
WiFiUDP UdpCommand;
uint8_t buffer[50] = "";
unsigned char packetBuffer[10];
int UDPpacketSize;
byte TxUdpBuffer[8];
#include <ETH.h>
static bool eth_connected = false;
IPAddress RemoteSwIP(0, 0, 0, 0);         // remote UDP IP switch - set from UDP DetectRemote array
int RemoteSwPort         = 0;             // remote UDP IP switch port
String HTTP_req;
#if defined(EnableOTA)
  #include <ESPmDNS.h>
  #include <ArduinoOTA.h>
#endif
#if defined(OTAWEB)
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
  AsyncWebServer OTAserver(82);
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

byte ShiftOutByte[3];
// https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
#include <Wire.h>
#define I2C_SDA 33
#define I2C_SCL 32

#if defined(BMP280)||defined(HTU21D)
  #include <SPI.h>
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

const int RpmPin = 39;
const int Rain1Pin = 36;
const int Rain2Pin = 35;
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
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 12
#include "FS.h"
#include "SD_MMC.h"

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
  #include <OneWire.h>
  #include <DallasTemperature.h>
  // const int DsPin = 3;
  // OneWire ds(DsPin);
  // DallasTemperature sensors(&ds);
  const int oneWireBus = 3;
  OneWire oneWire(oneWireBus);
  DallasTemperature sensors(&oneWire);
  bool ExtTemp = false;
#endif

//-------------------------------------------------------------------------------------------------------

void setup() {
  #if defined(DS18B20)
    sensors.begin();
  #endif

  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  #if defined(BMP280)
    I2Cone.begin(I2C_SDA, I2C_SCL, 100000); // SDA pin 21, SCL pin 22, 100kHz frequency
    if(!bmp.begin(0x76)){
      Serial.println(F(" *** Could not find a valid BMP280 sensor!"));
      // while (1) delay(10);
      BMP280enable=false;
    }else{
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

  #if defined(HTU21D)
    Wire.begin(I2C_SDA, I2C_SCL);
    if(!htu.begin()){
      Serial.println(F(" *** Could not find a valid HTU21D sensor!"));
      // while (1);
      HTU21Denable=false;
    }else{
      HTU21Denable=true;
    }
  #endif

  #if !defined(BMP280) && !defined(HTU21D)
    Wire.begin(I2C_SDA, I2C_SCL);
  #endif

  // SD
  if(!SD_MMC.begin()){
    Serial.println("SD card Mount Failed");
    // return;
  }

  // for (int i = 0; i < 8; i++) {
  //   pinMode(TestPin[i], INPUT);
  // }
  pinMode(RpmPin, INPUT);
  pinMode(Rain1Pin, INPUT);
  pinMode(Rain2Pin, INPUT);
  // pinMode(ButtonPin, INPUT);
  // SHIFT IN
  pinMode(ShiftInLatchPin, OUTPUT);
  pinMode(ShiftInClockPin, OUTPUT);
  pinMode(ShiftInDataPin, INPUT);

  // Listen source
  if (!EEPROM.begin(EEPROM_SIZE)){
    if(EnableSerialDebug>0){
      Serial.println("failed to initialise EEPROM"); delay(1);
    }
  }
  // 0-listen source
  // TxUdpBuffer[2] = EEPROM.read(0);
  // if(TxUdpBuffer[2]=='o'||TxUdpBuffer[2]=='r'||TxUdpBuffer[2]=='m'||TxUdpBuffer[2]=='e'){
  //   // OK
  // }else{
    TxUdpBuffer[2]='n';
  // }

  // 1-net ID
      NET_ID = EEPROM.read(1);
      TxUdpBuffer[0] = NET_ID;

  // 2-encoder range
  NumberOfEncoderOutputs = EEPROM.read(2);
  if(NumberOfEncoderOutputs < 2 || NumberOfEncoderOutputs > 0x0f){
    NumberOfEncoderOutputs=8;
  }

  // 4-EnableGroupPrefix
  // Serial.print("Enable group NET-ID prefix ");
  if(EEPROM.read(4)<2){
    EnableGroupPrefix=EEPROM.read(4);
  }
  // Serial.println(EnableGroupPrefix);

  // 5-EnableGroupButton
  if(EEPROM.read(5)<2){
    EnableGroupButton=EEPROM.read(5);
  }

  // 6-13 ButtonGroup
  if(EnableGroupButton==true){
    for (int i = 0; i < 8; i++) {
      if(EEPROM.read(6+i)<9){
        GroupButton[i]=EEPROM.read(6+i);
      }
    }
  }

  SERIAL1_BAUDRATE=EEPROM.readInt(14);
  SerialServerIPport=EEPROM.readInt(18);
  IncomingSwitchUdpPort=EEPROM.readInt(22);
  BroadcastPort=IncomingSwitchUdpPort;
  RebootWatchdog=EEPROM.readUInt(26);
  if(RebootWatchdog>10080){
    RebootWatchdog=0;
  }
  OutputWatchdog=EEPROM.readUInt(30);
  if(OutputWatchdog>10080){
    OutputWatchdog=0;
  }
  if(RebootWatchdog>0){
    ShiftOutByte[0]=EEPROM.readByte(34);
    ShiftOutByte[1]=EEPROM.readByte(35);
    ShiftOutByte[2]=EEPROM.readByte(36);
  }
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

  // if(EnableSerialDebug>0){
    Serial.println();
    Serial.print("Version: ");
    Serial.println(REV);
    Serial.println("===============================");
    Serial.print("SLAVE DEVICE NET-ID: 0x");
    if(NET_ID <=0x0f){
      Serial.print(F("0"));
    }
    Serial.println(NET_ID, HEX);
    Serial.print("Listen MASTER: ");
    if(TxUdpBuffer[2] == 'o'){
      Serial.println("Open Interface III");
    }
    if(TxUdpBuffer[2] == 'r' ){
      Serial.println("Band decoder MK2");
    }
    if(TxUdpBuffer[2] == 'm' ){
      Serial.println("IP switch master");
    }
    if(TxUdpBuffer[2] == 'n' ){
      Serial.println("none");
    }
    Serial.println("===============================");
    Serial.println("  press '?' for list commands");
    Serial.println();
    Serial.println();
  // }

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
    ETH.begin();
    if(DHCP_ENABLE==false){
      ETH.config(IPAddress(192, 168, 1, 188), IPAddress(192, 168, 1, 255),IPAddress(255, 255, 255, 0),IPAddress(8, 8, 8, 8));
      //config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = (uint32_t)0x00000000, IPAddress dns2 = (uint32_t)0x00000000);
    }
  #endif
    server.begin();
    UdpCommand.begin(IncomingSwitchUdpPort);    // incoming udp port
    // chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    //   unsigned long long1 = (unsigned long)((chipid & 0xFFFF0000) >> 16 );
    //   unsigned long long2 = (unsigned long)((chipid & 0x0000FFFF));
    //   ChipidHex = String(long1, HEX) + String(long2, HEX); // six octets
    //   YOUR_CALL=ChipidHex;

    // EEPROM YOUR_CALL
    if(EEPROM.read(141)==0xff){
      YOUR_CALL=String(ETH.macAddress()[0], HEX)+String(ETH.macAddress()[1], HEX)+String(ETH.macAddress()[2], HEX)+String(ETH.macAddress()[3], HEX)+String(ETH.macAddress()[4], HEX)+String(ETH.macAddress()[5], HEX);
    }else{
      for (int i=141; i<161; i++){
        if(EEPROM.read(i)!=0xff){
          YOUR_CALL=YOUR_CALL+char(EEPROM.read(i));
        }
      }
    }

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
  #if defined(OTAWEB)
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "PSE QSY to /update");
    });
    AsyncElegantOTA.begin(&OTAserver);    // Start ElegantOTA
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

  if(digitalRead(Rain1Pin)==0 && digitalRead(Rain2Pin)==1){
    RainStatus=false;
  }else if(digitalRead(Rain1Pin)==1 && digitalRead(Rain2Pin)==0){
    RainStatus=true;
  }

  // WDT
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  WdtTimer=millis();

  //init and get the time
   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
   RainCountDayOfMonth=UtcTime(2);
   Interrupts(true);
}

//-------------------------------------------------------------------------------------------------------

void loop() {
  // blank loop 80us
  // SerialToIp();
  http();
  Mqtt();
  // RX_UDP();
  CLI();
  Telnet();
  Watchdog();
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
   AsyncElegantOTA.loop();
  #endif
}
// SUBROUTINES -------------------------------------------------------------------------------------------------------

void Interrupts(boolean ON){
  if(ON==true){
    attachInterrupt(RpmPin, RPMcount, FALLING);
    // attachInterrupt(digitalPinToInterrupt(RpmPin), RPMcount, FALLING);
    // attachInterrupt(digitalPinToInterrupt(RpmPin), RPMcount, RISING);
  }else{
    detachInterrupt(digitalPinToInterrupt(RpmPin));
  }
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
void RPMcount(){
  Interrupts(false);
  if(digitalRead(RpmPin)==0){   // because FALLING not work
    RpmPulse = millis()-RpmTimer[0];
    RpmTimer[0] = millis();

    if(RpmPulse<PeriodMinRpmPulse){
      PeriodMinRpmPulse=RpmPulse;
      PeriodMinRpmPulseTimestamp=UtcTime(1);
    }
    if(RpmPulse<MinRpmPulse && needEEPROMcommit==false){
      // MinRpmPulseTimestamp=UtcTime(1);
      MinRpmPulseTimestamp=PeriodMinRpmPulseTimestamp;
      EEPROM.writeLong(169, RpmPulse);
      EEPROM.writeString(173, MinRpmPulseTimestamp);
      MinRpmPulse=RpmPulse;
      needEEPROMcommit = true;
    }
    if(RpmPulse<RpmTimer[1]){
      RpmAverage[1]=RpmAverage[1]+RpmPulse;
      RpmAverage[0]++;
    }
  }
  Interrupts(true);
}
//-------------------------------------------------------------------------------------------------------
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
byte Azimuth(){    // run from interrupt
  byte rxShiftInByte=0x00;
  digitalWrite(ShiftInLatchPin,1);   //Set latch pin to 1 to get recent data into the CD4021
  delayMicroseconds(10);
  // delay(1);
  digitalWrite(ShiftInLatchPin,0);     //Set latch pin to 0 to get data from the CD4021

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
      WindDir=0;
      break;
    case 0b01111110:
      WindDir=22;
      break;
    case 0b11111110:
      WindDir=45;
      break;
    case 0b11111100:
      WindDir=67;
      break;
    case 0b11111101:
      WindDir=90;
      break;
    case 0b11111001:
      WindDir=112;
      break;
    case 0b11111011:
      WindDir=135;
      break;
    case 0b11110011:
      WindDir=157;
      break;
    case 0b11110111:
      WindDir=180;
      break;
    case 0b11100111:
      WindDir=202;
      break;
    case 0b11101111:
      WindDir=225;
      break;
    case 0b11001111:
      WindDir=247;
      break;
    case 0b11011111:
      WindDir=270;
      break;
    case 0b10011111:
      WindDir=292;
      break;
    case 0b10111111:
      WindDir=315;
      break;
    case 0b00111111:
      WindDir=337;
      break;
    default:
      WindDir=1;
      break;
  }
  return rxShiftInByte;
}

//-------------------------------------------------------------------------------------------------------
// todo
// if(BMP280enable==true){
// if(HTU21Denable==true){

void AprsWxIgate() {
  #if defined(BMP280) && defined(HTU21D)
  if(AprsON==true){
    if (!AprsClient.connect(aprs_server_ip, AprsPort)) {
      if(EnableSerialDebug>0){
        Prn(3, 1,"APRS client connect failed!");
      }
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
      +"/"+LeadingZero(3,PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0])/0.447)
      +"g"+LeadingZero(3,PulseToMetterBySecond(PeriodMinRpmPulse)/0.447)
      +"t"+LeadingZero(3,htu.readTemperature()*1.8+32)
      +"h"+LeadingZero(3,constrain(htu.readHumidity(), 0, 100))
      +"b"+LeadingZero(6,Babinet(double(bmp.readPressure()), double(htu.readTemperature()*1.8+32)))
      +"P"+LeadingZero(3,RainPulseToMM(RainCount)/0.254) );
    }
    String StrBuf;
    StrBuf=LeadingZero(3,htu.readTemperature()*1.8+32);

    #if defined(DS18B20)
      if(ExtTemp==true){
        sensors.requestTemperatures();
        float temperatureF = sensors.getTempFByIndex(0);
        StrBuf=LeadingZero(3,temperatureF);
      }
    #endif

    AprsClient.println(YOUR_CALL+">APRSWX,TCPIP*,qAS,:="
    +AprsCoordinates+"_"
    +LeadingZero(3,WindDir)
    // Prn(OUT, 1, "             avg ("+String(RpmAverage[1])+"/"+String(RpmAverage[0])+") "+String(RpmAverage[1]/RpmAverage[0])+"ms | "+String(PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0]))+"m/s | "+String(PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0])*3.6)+" km/h");
    // Prn(OUT, 1, "             MAX in period "+String(PeriodMinRpmPulse)+"ms | "+String(PulseToMetterBySecond(PeriodMinRpmPulse))+"m/s | "+String(PulseToMetterBySecond(PeriodMinRpmPulse)*3.6)+" km/h ("+String(PeriodMinRpmPulseTimestamp)+")");
    // mps/0.447 = mph
    +"/"+LeadingZero(3,PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0])/0.447)
    +"g"+LeadingZero(3,PulseToMetterBySecond(PeriodMinRpmPulse)/0.447)
    // #if defined(DS18B20)
    //   +"t"+LeadingZero(3,temperatureF)
    // #else
    //   +"t"+LeadingZero(3,htu.readTemperature()*1.8+32)
    // #endif
    +"t"+StrBuf
    +"h"+LeadingZero(3,constrain(htu.readHumidity(), 0, 100))
    +"b"+LeadingZero(6,Babinet(double(bmp.readPressure()), double(htu.readTemperature()*1.8+32)))
    +"P"+LeadingZero(3,RainPulseToMM(RainCount)/0.254) );

    if(EnableSerialDebug>0){
      Prn(3, 1,"APRS-TX | "+YOUR_CALL+">APRSWX,TCPIP*,qAC:> remoteqth.com 3D-printed WX station");
    }
    AprsClient.println(YOUR_CALL+">APRSWX,TCPIP*,qAC:> remoteqth.com 3D-printed WX station");
  }
  #endif
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
void Print_SDCard_Info ()
{
    uint8_t cardType = SD_MMC.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD_MMC card attached");
        return;
    }

    Serial.print("SD_MMC Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

    listDir(SD_MMC, "/", 0);
    createDir(SD_MMC, "/mydir");
    listDir(SD_MMC, "/", 0);
    removeDir(SD_MMC, "/mydir");
    listDir(SD_MMC, "/", 2);
    writeFile(SD_MMC, "/hello.txt", "Hello ");
    appendFile(SD_MMC, "/hello.txt", "World!\n");
    readFile(SD_MMC, "/hello.txt");
    deleteFile(SD_MMC, "/foo.txt");
    renameFile(SD_MMC, "/hello.txt", "/foo.txt");
    readFile(SD_MMC, "/foo.txt");
    testFileIO(SD_MMC, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

}
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
void Watchdog(){

  // WDT
  if(millis()-WdtTimer > 60000){
    esp_task_wdt_reset();
    WdtTimer=millis();
    if(EnableSerialDebug>0){
      Prn(3, 0,"WDT reset ");
      Prn(3, 1, UtcTime(1));
    }
  }

  // frenetic mode
  if(EnableSerialDebug>1 && millis()-FreneticModeTimer > 1000){
    Azimuth();
    Prn(3, 1, "  Wind speed az/last/avg/max "+String(WindDir)+"° "+String(PulseToMetterBySecond(RpmPulse)*3.6)+"/"+String(PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0])*3.6)+"/"+String(PulseToMetterBySecond(PeriodMinRpmPulse)*3.6)+" km/h");
    if(eth_connected==true && mqttClient.connected()==true){
      MqttPubString("WindSpeed_az/last/avg/max", String(WindDir)+"° "+String(PulseToMetterBySecond(RpmPulse)*3.6)+"/"+String(PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0])*3.6)+"/"+String(PulseToMetterBySecond(PeriodMinRpmPulse)*3.6)+" km/h", false);
    }
    FreneticModeTimer=millis();
  }

  // if(NeedSpeedAlert_ms==true){
  //   NeedSpeedAlert_ms=false;
  //   MqttPubString("SpeedAlert-mps", String(PulseToMetterBySecond(RpmPulse)), false);
  //   Azimuth();
  //   MqttPubString("SpeedAlert-az", String(WindDir), false);
  // }

  if(RebootWatchdog > 0 && millis()-WatchdogTimer > RebootWatchdog*60000){
    Serial.println();
    Serial.println("** Activate reboot watchdog - IP switch will be restarted **");
    EEPROM.writeByte(34, ShiftOutByte[0]);
    EEPROM.writeByte(35, ShiftOutByte[1]);
    EEPROM.writeByte(36, ShiftOutByte[2]);
    EEPROM.commit();
    delay(1000);
    TelnetServerClients[0].stop();
    ESP.restart();
  }

  if(OutputWatchdog > 0 && millis()-WatchdogTimer > OutputWatchdog*60000 && OutputWatchdog < 123456){
    Serial.println();
    Serial.println("** Activate clear output watchdog **");
    ShiftOutByte[0]=0x00;
    ShiftOutByte[1]=0x00;
    ShiftOutByte[2]=0x00;
    EEPROM.writeByte(34, ShiftOutByte[0]);
    EEPROM.writeByte(35, ShiftOutByte[1]);
    EEPROM.writeByte(36, ShiftOutByte[2]);
    EEPROM.commit();
    #if defined(ShiftOut)
      // digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
      // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, 0x01);
      // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
      // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
      // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
      // digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
    #endif
    // deactivate
    OutputWatchdog=123456;
  }

  // if(millis()-HeartBeatTimer[0]>HeartBeatTimer[1]){
  //   MqttPubString("HeartBeat", String(UtcTime(1)), false);
  //   // MqttPubString("WindDir", String(WindDir), false);
  //   HeartBeatTimer[0]=millis();
  // }

  // Rain counter 5sec
  if(millis()-RainTimer[0]>RainTimer[1]){
    if(digitalRead(Rain1Pin)==0 && digitalRead(Rain2Pin)==1){
      if(RainStatus==true){
        #if defined(HTU21D)
          if(HTU21Denable==true){
            // debouncing
            if(htu.readHumidity()>90){
              RainCount++;
              MqttPubString("RainCount", String(RainCount)+" (day "+String(UtcTime(2))+")", false);
              MqttPubString("RainToday-mm", String(RainPulseToMM(RainCount)), false);
            }
          }
        #endif
        RainStatus=false;
      }
    }else if(digitalRead(Rain1Pin)==1 && digitalRead(Rain2Pin)==0){
      if(RainStatus==false){
        #if defined(HTU21D)
          if(HTU21Denable==true){
            // debouncing
            if(htu.readHumidity()>90){
              RainCount++;
              MqttPubString("RainCount", String(RainCount)+" (day "+String(UtcTime(2))+")", false);
              MqttPubString("RainToday-mm", String(RainPulseToMM(RainCount)), false);
            }
          }
        #endif
        RainStatus=true;
      }
    }
    RainTimer[0]=millis();

    if(RainCountDayOfMonth=="n/a"){
      RainCountDayOfMonth=UtcTime(2);
    }

  }

  // Half hour
  if(millis()-MeasureTimer[0]>MeasureTimer[1] && eth_connected==true && mqttClient.connected()==true){
    Interrupts(false);
    if(UtcTime(2)!=RainCountDayOfMonth){
      RainCount=0;
      RainCountDayOfMonth=UtcTime(2);
    }
    String StringUtc1 = UtcTime(1);
    MqttPubString("utc", StringUtc1, false);
    Azimuth();
    MqttPubString("WindDir-azimuth", String(WindDir), false);

    // MqttPubString("WindSpeedLast", String(RpmPulse)+" "+StringUtc1, false);
    MqttPubString("WindSpeedAvg-mps", String(PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0])), false);
    if(PeriodMinRpmPulse<987654321){
      MqttPubString("WindSpeedMaxPeriod-mps", String(PulseToMetterBySecond(PeriodMinRpmPulse)), false);
      MqttPubString("WindSpeedMaxPeriod-utc", String(PeriodMinRpmPulseTimestamp), false);
    }
    // MqttPubString("WindSpeedMax-mps", String(PulseToMetterBySecond(MinRpmPulse)), true);
    // MqttPubString("WindSpeedMax-utc", String(MinRpmPulseTimestamp), true);

    #if defined(BMP280)
      if(BMP280enable==true){
        MqttPubString("Pressure-hPa", String(Babinet(double(bmp.readPressure()), double(htu.readTemperature()*1.8+32))/100), false);
        // MqttPubString("TemperatureBak-Celsius", String(bmp.readTemperature()), false);
      }else{
        MqttPubString("Pressure-hPa", "n/a", false);
        // MqttPubString("TemperatureBak-Celsius", "n/a", false);
      }
    #endif
    #if defined(HTU21D)
      if(HTU21Denable==true){
        MqttPubString("HumidityRel-Percent", String(constrain(htu.readHumidity(), 0, 100)), false);
        // MqttPubString("Temperature-Celsius", String(htu.readTemperature()), false);
      }else{
        MqttPubString("HumidityRel-Percent", "n/a", false);
        // MqttPubString("Temperature-Celsius", "n/a", false);
      }
    #endif
    #if defined(DS18B20)
      if(ExtTemp==true){
        sensors.requestTemperatures();
        float temperatureC = sensors.getTempCByIndex(0);
        MqttPubString("Temperature-Celsius", String(temperatureC), false);
      }else{
        // MqttPubString("Temperature-Celsius", "n/a", false);
        MqttPubString("Temperature-Celsius", String(htu.readTemperature()), false);
      }
    #endif

    AprsWxIgate();

    RpmAverage[0]=1;
    RpmAverage[1]=0;
    PeriodMinRpmPulse=987654321;
    PeriodMinRpmPulseTimestamp="";

    // writeFile(SD_MMC, "/hello.txt", "Hello ");
    // appendFile(SD_MMC, "/wx-"+String(UtcTime(3))+".txt", UtcTime(1) );
    MeasureTimer[0]=millis();
    Interrupts(true);
  }

  if(needEEPROMcommit==true){
    Interrupts(false);
    needEEPROMcommit = false;
    EEPROM.commit();
    MqttPubString("WindSpeedMax-mps", String(PulseToMetterBySecond(MinRpmPulse)), true);
    MqttPubString("WindSpeedMax-utc", String(MinRpmPulseTimestamp), true);

    Interrupts(true);
  }

  if(!TelnetServerClients[0].connected() && FirstListCommands==false){
    FirstListCommands=true;
  }

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

  if(OUT<2){
    esp_task_wdt_reset();
    WdtTimer=millis();

    // ?
    if(incomingByte==63){
      ListCommands(OUT);

    // m
    // }else if(incomingByte==109){
    //   TxUdpBuffer[2] = 'm';
    //   EEPROM.write(0, 'm'); // address, value
    //   EEPROM.commit();
    //   Prn(OUT, 1,"Now control from: IP switch master");
    //   if(EnableSerialDebug>0){
    //     Prn(OUT, 0,"EEPROM read [");
    //     Prn(OUT, 0, String(EEPROM.read(0)) );
    //     Prn(OUT, 1,"]");
    //   }
    //   TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
    //   if(TxUdpBuffer[2] == 'm'){
    //     TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
    //   }

    // r
    // }else if(incomingByte==114){
    //   EnableGroupPrefix=false;
    //     EEPROM.write(4, EnableGroupPrefix);
    //     EEPROM.commit();
    //   EnableGroupButton=false;
    //     EEPROM.write(5, EnableGroupButton);
    //     EEPROM.commit();
    //   TxUdpBuffer[2] = 'r';
    //   EEPROM.write(0, 'r'); // address, value
    //   EEPROM.commit();
    //   Prn(OUT, 1,"Now control from: Band decoder");
    //   if(EnableSerialDebug>0){
    //     Prn(OUT, 0,"EEPROM read [");
    //     Prn(OUT, 0, String(EEPROM.read(0)) );
    //     Prn(OUT, 1,"]");
    //   }
    //   TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
    //   if(TxUdpBuffer[2] == 'm'){
    //     TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
    //   }
    //
    // // o
    // }else if(incomingByte==111){
    //   EnableGroupPrefix=false;
    //     EEPROM.write(4, EnableGroupPrefix);
    //     EEPROM.commit();
    //   EnableGroupButton=false;
    //     EEPROM.write(5, EnableGroupButton);
    //     EEPROM.commit();
    //   TxUdpBuffer[2] = 'o';
    //   EEPROM.write(0, 'o'); // address, value
    //   EEPROM.commit();
    //   Prn(OUT, 1,"Now control from: Open Interface III");
    //   if(EnableSerialDebug>0){
    //     Prn(OUT, 0,"EEPROM read [");
    //     Prn(OUT, 0, String(EEPROM.read(0)) );
    //     Prn(OUT, 1,"]");
    //   }
    //   TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
    //   if(TxUdpBuffer[2] == 'm'){
    //     TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
    //   }
    //
    // // n
    // }else if(incomingByte==110){
    //   EnableGroupPrefix=false;
    //   EEPROM.write(4, EnableGroupPrefix);
    //   EnableGroupButton=false;
    //   EEPROM.write(5, EnableGroupButton);
    //   TxUdpBuffer[2] = 'n';
    //   EEPROM.write(0, 'n'); // address, value
    //   EEPROM.commit();
    //   Prn(OUT, 1,"Now control from: none");

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
        EEPROM.commit();
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
      Prn(OUT, 1,"List EEPROM");
      for (int i=0; i<EEPROM_SIZE; i++){
        Prn(OUT, 0, String(i));
        Prn(OUT, 0, ">" );
        Prn(OUT, 0, String(EEPROM.read(i)) );
        Prn(OUT, 0, " " );
      }
      Prn(OUT, 1, "" );

    // w
    }else if(incomingByte==119 && TxUdpBuffer[2]!='n'){
      Prn(OUT, 1,"Write reboot watchdog in minutes (0-10080), 0-disable");
      Prn(OUT, 1,"recomended 1440 (1 day)");
      EnterInt(OUT);
      if(CompareInt>=0 && CompareInt<=10080){
        RebootWatchdog = CompareInt;
        EEPROM.writeUInt(26, RebootWatchdog);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 0, String(EEPROM.readUInt(26)) );
        Prn(OUT, 1," minutes");
      }else{
        Prn(OUT, 0,"Out of range.");
      }

    // W
    }else if(incomingByte==87 && TxUdpBuffer[2]!='n'){
      Prn(OUT, 1,"Write clear output watchdog in minutes (0-10080), 0-disable");
      Prn(OUT, 1,"note: if you need clear output after reboot watchdog, set smaller than it");
      EnterInt(OUT);
      if(CompareInt>=0 && CompareInt<=10080){
        OutputWatchdog = CompareInt;
        EEPROM.writeUInt(30, OutputWatchdog);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 0, String(EEPROM.readUInt(30)) );
        Prn(OUT, 1," minutes");
      }else{
        Prn(OUT, 0,"Out of range.");
      }

    // <
    }else if(incomingByte==60 && TxUdpBuffer[2]!='n'){
      Prn(OUT, 1,"write UDP port (1-65535)");
      EnterInt(OUT);
      if(CompareInt>=1 && CompareInt<=65535){
        IncomingSwitchUdpPort = CompareInt;
        EEPROM.writeInt(22, IncomingSwitchUdpPort);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 1, String(IncomingSwitchUdpPort) );
        Prn(OUT, 0,"** device will be restarted **");
        delay(1000);
        TelnetServerClients[0].stop();
        ESP.restart();
      }else{
        Prn(OUT, 0,"Out of range.");
      }

    // /
    }else if(incomingByte==47 && TxUdpBuffer[2] == 'm'){
      Prn(OUT, 1,"write encoder rannge number (2-16)");
      EnterInt(OUT);
      if(CompareInt>=2 && CompareInt<=16){
        NumberOfEncoderOutputs = CompareInt;
/*      EnterChar(OUT);
      // 2-G
      if( (incomingByte>=50 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=103) ){
        if(incomingByte>=50 && incomingByte<=57){
          NumberOfEncoderOutputs = incomingByte-48;
        }else if(incomingByte>=97 && incomingByte<=103){
          NumberOfEncoderOutputs = incomingByte-87;
        }*/
        NumberOfEncoderOutputs--;
        EEPROM.write(2, NumberOfEncoderOutputs); // address, value
        EEPROM.commit();
        Prn(OUT, 0,"** Now Encoder range change to ");
        Prn(OUT, 0, String(NumberOfEncoderOutputs+1) );
        Prn(OUT, 1," **");
        if(EnableSerialDebug>0){
          Prn(OUT, 0,"EEPROM read [");
          Prn(OUT, 0, String(EEPROM.read(2)) );
          Prn(OUT, 1,"]");
        }
        TxUDP('s', packetBuffer[2], 'c', 'f', 'm', 1);    // 0=broadcast, 1= direct to RX IP
        if(TxUdpBuffer[2] == 'm'){
          TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
        }
      }else{
        Prn(OUT, 1,"Out of range");
      }

    // %
    }else if(incomingByte==37){
      EnableGroupButton=!EnableGroupButton;
      Prn(OUT, 0,"** Group buttons (one from) [");
      EEPROM.write(5, EnableGroupButton);
      EEPROM.commit();
      if(EnableGroupButton==true){
        Prn(OUT, 1,"ON] **");
        for (int i = 0; i < 8; i++) {
          if(EEPROM.read(6+i)<9){
            GroupButton[i]=EEPROM.read(6+i);
          }
        }
      }else{
        Prn(OUT, 1,"OFF] **");
      }

    // :
    }else if(incomingByte==58 && EnableGroupButton==true){
      Prn(OUT, 1," List groups");
      for (int i = 0; i < 8; i++) {
        Prn(OUT, 0,"  Button ");
        Prn(OUT, 0, String(i+1) );
        Prn(OUT, 0," in group ");
        Prn(OUT, 1, String(GroupButton[i]) );
      }

    // !
    }else if(incomingByte==33 && EnableGroupButton==true){
      Prn(OUT, 1,"Press button number 1-8...");
      EnterChar(OUT);
      if( (incomingByte>=49 && incomingByte<=56)){
        unsigned int ButtonNumber=incomingByte-48;
        Prn(OUT, 0,"Press Group number 1-8 for button ");
        Prn(OUT, 1, String(ButtonNumber) );
        EnterChar(OUT);
        if( (incomingByte>=49 && incomingByte<=56)){
          unsigned int ButtonGroup=incomingByte-48;
          Prn(OUT, 0," store Button ");
          Prn(OUT, 0, String(ButtonNumber) );
          Prn(OUT, 0," to group ");
          Prn(OUT, 1, String(ButtonGroup) );
          GroupButton[ButtonNumber-1]=ButtonGroup;
          for (int i = 0; i < 8; i++) {
            EEPROM.write(6+i, GroupButton[i]);
          }
          EEPROM.commit();
        }else{
          Prn(OUT, 1," accepts 0-8, exit");
        }
      }else{
        Prn(OUT, 1," accepts 0-8, exit");
      }

    // *
    }else if(incomingByte==42){
      EnableSerialDebug++;
      if(EnableSerialDebug>2){
        EnableSerialDebug=0;
      }
      Prn(OUT, 0,"** Serial DEBUG ");
      if(EnableSerialDebug==0){
        Prn(OUT, 1,"DISABLE **");
      }else if(EnableSerialDebug==1){
        Prn(OUT, 1,"ENABLE **");
      }else if(EnableSerialDebug==2){
        Prn(OUT, 1,"ENABLE with frenetic mode **");
      }

    // #
    }else if(incomingByte==35 && TxUdpBuffer[2]!='n'){
      if(EnableGroupPrefix==false){
        Prn(OUT, 1,"Press NET-ID X_ prefix 0-f...");
      }else{
        Prn(OUT, 1,"Press NET-ID _X sufix 0-f...");
      }
      EnterChar(OUT);
      if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
        // prefix
        if(EnableGroupPrefix==false){
          bitClear(NET_ID, 4);
          bitClear(NET_ID, 5);
          bitClear(NET_ID, 6);
          bitClear(NET_ID, 7);
          Serial.write(incomingByte);
          Prn(OUT, 1,"");
          if(incomingByte>=48 && incomingByte<=57){
            incomingByte = incomingByte-48;
            incomingByte = (byte)incomingByte << 4;
            NET_ID = NET_ID | incomingByte;
            TxUdpBuffer[0] = NET_ID;
          }else if(incomingByte>=97 && incomingByte<=102){
            incomingByte = incomingByte-87;
            incomingByte = (byte)incomingByte << 4;
            NET_ID = NET_ID | incomingByte;
            TxUdpBuffer[0] = NET_ID;
          }
        }
        // sufix
        // if(HW_BCD_SW==false){
          if(EnableGroupPrefix==false){
            Prn(OUT, 1,"Press NET-ID _X sufix 0-f...");
            EnterChar(OUT);
          }
          if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
            bitClear(NET_ID, 0);
            bitClear(NET_ID, 1);
            bitClear(NET_ID, 2);
            bitClear(NET_ID, 3);
            Serial.write(incomingByte);
            Prn(OUT, 1, "");
            if(incomingByte>=48 && incomingByte<=57){
              incomingByte = incomingByte-48;
              NET_ID = NET_ID | incomingByte;
              TxUdpBuffer[0] = NET_ID;
            }else if(incomingByte>=97 && incomingByte<=102){
              incomingByte = incomingByte-87;
              NET_ID = NET_ID | incomingByte;
              TxUdpBuffer[0] = NET_ID;
            }
        // #endif
            EEPROM.write(1, NET_ID); // address, value
            EEPROM.commit();
            Prn(OUT, 0,"** Now NET-ID change to 0x");
            if(NET_ID <=0x0f){
              Prn(OUT, 0, String("0"));
            }
            Prn(OUT, 0, String(NET_ID, HEX) );
            Prn(OUT, 1," **");
            if(EnableSerialDebug>0){
              Prn(OUT, 0,"EEPROM read [");
              Prn(OUT, 0, String(EEPROM.read(1), HEX) );
              Prn(OUT, 1,"]");
            }
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
        // #if !defined(HW_BCD_SW)
          }else{
            Prn(OUT, 1," accepts 0-f, exit");
          }
        // #endif
        // }
      }else{
        Prn(OUT, 1," accepts 0-f, exit");
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
        EEPROM.commit();
        Prn(OUT, 1,"");
        Prn(OUT, 1,"  Eeprom erased done");
      }else{
        Prn(OUT, 1,"  Erase aborted");
      }
    }else{
      Prn(OUT, 1,"  Erase aborted");
    }

    // C
    // }else if(incomingByte==67){
    //   Prn(OUT, 1,"  EEPROM.commit");
    //   EEPROM.commit();

    // $
    // }else if(incomingByte==36){
    //   EnableGroupPrefix=!EnableGroupPrefix;
    //   Prn(OUT, 0,"** Group sufix (multi control) [");
    //   EEPROM.write(4, EnableGroupPrefix);
    //   EEPROM.commit();
    //   if(EnableGroupPrefix==true){
    //     Prn(OUT, 1,"ON] **");
    //   }else{
    //     Prn(OUT, 1,"OFF] **");
    //   }
    //   if(EnableGroupPrefix==true){
    //     // clear prefix
    //     bitClear(NET_ID, 4);
    //     bitClear(NET_ID, 5);
    //     bitClear(NET_ID, 6);
    //     bitClear(NET_ID, 7); // <-
    //   }
    //   Prn(OUT, 1,"** IP switch will be restarted **");
    //   delay(1000);
    //   TelnetServerClients[0].stop();
    //   ESP.restart();

    // .
    }else if(incomingByte==46){
      Prn(OUT, 1,"Reset timer and sent measure");
      MeasureTimer[0]=millis()-MeasureTimer[1];

    // (
    // }else if(incomingByte==40){
    //   Prn(OUT, 1,"Write baudrate");
    //   EnterInt(OUT);
    //   if(CompareInt>=80 && CompareInt<=5000000){
    //     SERIAL1_BAUDRATE = CompareInt;
    //     EEPROM.writeInt(14, SERIAL1_BAUDRATE);
    //     EEPROM.commit();
    //     Prn(OUT, 0," Set ");
    //     Prn(OUT, 1, String(SERIAL1_BAUDRATE) );
    //     Prn(OUT, 0,"** device will be restarted **");
    //     delay(1000);
    //     TelnetServerClients[0].stop();
    //     ESP.restart();
    //   }else{
    //     Prn(OUT, 0,"Out of range.");
    //   }

    // )
    // }else if(incomingByte==41){
    //   Prn(OUT, 1,"Write IP port (1-65535)");
    //   EnterInt(OUT);
    //   if(CompareInt>=1 && CompareInt<=65535){
    //     SerialServerIPport = CompareInt;
    //     EEPROM.writeInt(18, SerialServerIPport);
    //     EEPROM.commit();
    //     Prn(OUT, 0," Set ");
    //     Prn(OUT, 1, String(SerialServerIPport) );
    //     Prn(OUT, 0,"** device will be restarted **");
    //     delay(1000);
    //     TelnetServerClients[0].stop();
    //     ESP.restart();
    //   }else{
    //     Prn(OUT, 0,"Out of range.");
    //   }

    // R
    // }else if(incomingByte==82){
    //   readFile(SD_MMC, "/wx-"+String(UtcTime(3))+".txt");

    // /
    }else if(incomingByte==47){
      listDir(SD_MMC, "/", 2);

    #if !defined(BMP280) && !defined(HTU21D)
      // 2
      }else if(incomingByte==50){
        I2cScanner();
    #endif

    // &
    // }else if(incomingByte==38 && TxUdpBuffer[2]!='n'){
    //   TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
    //   if(TxUdpBuffer[2] == 'm'){
    //     TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
    //   }

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
      EEPROM.commit();

    // @
    }else if(incomingByte==64){
      Prn(OUT, 1,"** IP switch will be restarted **");
      if(RebootWatchdog > 0){
        Prn(OUT, 1,"   Activate reboot watchdog - store outputs to EEPROM...");
        EEPROM.writeByte(34, ShiftOutByte[0]);
        EEPROM.writeByte(35, ShiftOutByte[1]);
        EEPROM.writeByte(36, ShiftOutByte[2]);
        EEPROM.commit();
        delay(1000);
      }
      TelnetServerClients[0].stop();
      ESP.restart();

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
      EEPROM.commit();
      Prn(OUT, 1,"** Device will be restarted **");
      delay(1000);
      ESP.restart();

  // A
  }else if(incomingByte==65){
      if(AprsON==true){
        AprsON=false;
        Prn(OUT, 1,"** APRS DISABLE **");
        EEPROM.write(199, AprsON); // address, value
        EEPROM.commit();
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
            EEPROM.commit();
          }else{
            Prn(OUT, 1,"  bye");
          }
        }else{
          Prn(OUT, 1,"  bye");
        }
      }

  // i
}else if(incomingByte==105 && AprsON==true){
    Prn(OUT, 1,"  enter APRS i-gate IP address, by four number (0-255) and press [enter] after each");
    Prn(OUT, 1,"  (for example 89.235.48.27:14580)");
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
        // Prn(OUT, 1,"InputByte["+String(j)+"] "+String(InputByte[j])+"/"+String(InputByte[j]-48));
        mult = mult*10;
      }
      // Prn(OUT, 1,"intBuf "+String(intBuf));
      if( (i<4 && intBuf>=0 && intBuf<=255) || (i==4 && intBuf>=1 && intBuf<=65535) ){
        if(i==4){
          EEPROM.writeInt(204, intBuf);
        }else{
          EEPROM.writeByte(200+i, intBuf);
        }
        // Prn(OUT, 1,"EEPROMcomit");
        EEPROM.commit();
      }else{
        Prn(OUT, 1,"Out of range.");
        break;
      }
    }
    for(int i=0; i<4; i++){
      aprs_server_ip[i]=EEPROM.readByte(i+200);
    }
    AprsPort = EEPROM.readInt(204);
    // Prn(OUT, 0,"** device will be restarted **");
    // delay(1000);
    // TelnetServerClients[0].stop();
    // ESP.restart();

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
      EEPROM.commit();

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
      EEPROM.commit();

  #if defined(DS18B20)
  // s
  }else if(incomingByte==115){
    ExtTemp=!ExtTemp;
      if(ExtTemp==true){
        Prn(OUT, 1,"** External temp sensor ENABLE **");
      }else{
        Prn(OUT, 1,"** External temp sensor DISABLE **");
      }
        EEPROM.write(239, ExtTemp); // address, value
        EEPROM.commit();

    // s
    // }else if(incomingByte==115){
    //   Prn(OUT, 1,"  Input WX coordinates and press [enter] in format.");
    // byte i;
    // byte addr[8];
    // if (!ds.search(addr)) {
    //   Prn(OUT, 1,"  No more addresses.");
    //   ds.reset_search();
    //   delay(250);
    //   return;
    // }
    // Prn(OUT, 0,"  ROM= ");
    // for (i = 0; i < 8; i++) {
    //   Prn(OUT, 0," ");
    //   Prn(OUT, 0, String(addr[i], HEX));
    // }
  #endif

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
      EEPROM.commit();
    }else{
      Prn(OUT, 1," Out of range");
    }

// W
}else if(incomingByte==87){
    Prn(OUT, 1,"** Erase WindSpeedMax memory? [y/n] **");
    EnterChar(OUT);
    if(incomingByte==89 || incomingByte==121){
      EEPROM.writeLong(169, 0); // 987654321
      EEPROM.commit();
      MqttPubString("WindSpeedMax-mps", "0", true);
      MqttPubString("WindSpeedMax-utc", String(MinRpmPulseTimestamp), true);
    }else{
      Prn(OUT, 1,"  bye");
    }

  // a
}else if(incomingByte==97){
    Prn(OUT, 0,"write limit for wind speed alert 0-60 m/s, and press [");
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
    if(intBuf>=0 && intBuf<=60){
      SpeedAlertLimit_ms = MpsToMs(intBuf);
      EEPROM.writeInt(235, SpeedAlertLimit_ms); // address, value
      EEPROM.commit();
    }else{
      Prn(OUT, 1," Out of range");
    }

    // LF
    // }else if(incomingByte==10){
    //   Prn(OUT, 1,"");

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
  Prn(OUT, 0,"> ");

  if(OUT==0){
    while(br==false) {
      if(Serial.available()){
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
          if(TelnetServerClients[0].available()){
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
  Prn(OUT, 0,">");
  if(OUT==0){
    while (Serial.available() == 0) {
      // Wait
    }
    incomingByte = Serial.read();
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      while(incomingByte==0){
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
  Prn(OUT, 0,"> ");
  if(OUT==0){
    while(!Serial.available()) {
    }
    delay(3000);
    CompareInt = Serial.parseInt();
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      bool br=true;
      int intField[10];
      int count=0;

      while(incomingByte==0 && br==true){
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
void Prn(int OUT, int LN, String STR){
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

  #if defined(ETHERNET)
    Prn(OUT, 1,"");
    Prn(OUT, 1,"------  WX station [ESP32-POE] status  ------");
    Prn(OUT, 0,"  http://");
    Prn(OUT, 1, String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
    Prn(OUT, 0,"  ETH: MAC ");
    Prn(OUT, 0, String(ETH.macAddress()[0], HEX)+":"+String(ETH.macAddress()[1], HEX)+":"+String(ETH.macAddress()[2], HEX)+":"+String(ETH.macAddress()[3], HEX)+":"+String(ETH.macAddress()[4], HEX)+":"+String(ETH.macAddress()[5], HEX)+", " );
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

  if(RebootWatchdog > 0){
    Prn(OUT, 0,"> Reboot countdown in ");
    Prn(OUT, 0, String(RebootWatchdog-((millis()-WatchdogTimer)/60000)) );
    Prn(OUT, 1, " minutes");
  }
  if(OutputWatchdog > 0 && OutputWatchdog<123456){
    Prn(OUT, 0,"> Clear output countdown in ");
    Prn(OUT, 0, String(OutputWatchdog-((millis()-WatchdogTimer)/60000)) );
    Prn(OUT, 1," minutes");
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
  Prn(OUT, 1, String(REV));


  Prn(OUT, 0,"  micro SD card present: ");
  uint8_t cardType = SD_MMC.cardType();

  if(cardType == CARD_NONE){
      Prn(OUT, 1,"none ");
      // return;
  }else if(cardType == CARD_MMC){
      Prn(OUT, 1,"MMC ");
  } else if(cardType == CARD_SD){
      Prn(OUT, 1,"SDSC ");
  } else if(cardType == CARD_SDHC){
      Prn(OUT, 1,"SDHC ");
  } else {
      Prn(OUT, 1,"unknown ");
  }

  Prn(OUT, 0,"  RpmPin ");
    Prn(OUT, 0,String(digitalRead(RpmPin)));
  Prn(OUT, 0," | Rain1Pin ");
    Prn(OUT, 0,String(digitalRead(Rain1Pin)));
  Prn(OUT, 0," | Rain2Pin ");
    Prn(OUT, 1,String(digitalRead(Rain2Pin)));
  // Prn(OUT, 0," | ButtonPin ");
  //   Prn(OUT, 1, String(digitalRead(ButtonPin)));
  Prn(OUT, 1,"---------------------------------------------");
  if(digitalRead(Rain1Pin)==digitalRead(Rain2Pin)){
    Prn(OUT, 1,"! Rain sensor malformed" );
  }else{
    Prn(OUT, 1,"  "+RainCountDayOfMonth+"th day Rain counter "+String(RainCount)+" | "+String(RainPulseToMM(RainCount))+"mm" );
  }
  // if(EnableSerialDebug>0){
  // }else{
  // }
  Azimuth();
  Prn(3, 0,"  Azimuth ");
  Prn(3, 1,String(Azimuth(),BIN));
  if(WindDir==1){
    Prn(OUT, 1,"! Wind direction sensor malformed" );
  }else{
    Prn(OUT, 1, "  Wind direction "+String(WindDir)+"°");
  }
  Prn(OUT, 1, "  Wind speed last "+String(RpmPulse)+"ms | "+String(PulseToMetterBySecond(RpmPulse))+"m/s | "+String(PulseToMetterBySecond(RpmPulse)*3.6)+" km/h");
  Prn(OUT, 1, "             avg ("+String(RpmAverage[1])+"/"+String(RpmAverage[0])+") "+String(RpmAverage[1]/RpmAverage[0])+"ms | "+String(PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0]))+"m/s | "+String(PulseToMetterBySecond(RpmAverage[1]/RpmAverage[0])*3.6)+" km/h");
  Prn(OUT, 1, "             MAX in period "+String(PeriodMinRpmPulse)+"ms | "+String(PulseToMetterBySecond(PeriodMinRpmPulse))+"m/s | "+String(PulseToMetterBySecond(PeriodMinRpmPulse)*3.6)+" km/h ("+String(PeriodMinRpmPulseTimestamp)+")");
  Prn(OUT, 1, "             lifetime MAX "+String(MinRpmPulse)+"ms | "+String(PulseToMetterBySecond(MinRpmPulse))+"m/s | "+String(PulseToMetterBySecond(MinRpmPulse)*3.6)+" km/h ("+String(MinRpmPulseTimestamp)+")");
  #if defined(HTU21D)
    if(HTU21Denable==true){
      Prn(OUT, 1, "  Humidity relative "+String(constrain(htu.readHumidity(), 0, 100))+"% ["+String(htu.readTemperature())+"°C]");
    }else{
      Prn(OUT, 1, "  Humidity relative n/a");
    }
  #endif
  #if defined(BMP280)
    if(BMP280enable==true){
      Prn(OUT, 1, "  Pressure "+String(Babinet(double(bmp.readPressure()), double(htu.readTemperature()*1.8+32))/100)+" hPa ["+String(bmp.readPressure()/100)+" raw] "+String(bmp.readTemperature())+"°C");
    }else{
      Prn(OUT, 1, "  Pressure n/a");
    }
  #endif
  #if defined(DS18B20)
    if(ExtTemp==true){
      if(OUT==0){
        Serial.flush();
        Serial.end();
      }
      sensors.requestTemperatures();
      float temperatureC = sensors.getTempCByIndex(0);
      if(OUT==0){
        Serial.begin(SERIAL_BAUDRATE);
        while(!Serial) {
          ; // wait for serial port to connect. Needed for native USB port only
        }
      }
      Prn(OUT, 1, "  Temperature "+String(temperatureC)+"°C");
    }
  #endif
  // Prn(OUT, 1, "  Humidity "+String()+"%");
  // if(cardType!=CARD_NONE){
  //   uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  //   // Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
  //   Prn(OUT, 1," | size "+String(printf("%lluMB", cardSize)));
  // }
  Prn(OUT, 1,"---------------------------------------------");
  // Prn(OUT, 1,"  You can change source, with send character:");
  // if(TxUdpBuffer[2]=='m'){
  //   Prn(OUT, 0,"     [m]");
  // }else{
  //   Prn(OUT, 0,"      m ");
  // }
  // Prn(OUT, 1,"- IP switch master");
  // if(TxUdpBuffer[2]=='r'){
  //   Prn(OUT, 0,"     [r]");
  // }else{
  //   Prn(OUT, 0,"      r ");
  // }
  // Prn(OUT, 1,"- Band decoder");
  // if(TxUdpBuffer[2]=='o'){
  //   Prn(OUT, 0,"     [o]");
  // }else{
  //   Prn(OUT, 0,"      o ");
  // }
  // Prn(OUT, 1,"- Open Interface III");
  // if(TxUdpBuffer[2]=='n'){
  //   Prn(OUT, 0,"     [n]");
  // }else{
  //   Prn(OUT, 0,"      n ");
  // }
  // Prn(OUT, 1,"- none");
  // Prn(OUT, 1,"");
  Prn(OUT, 1,"      ?  list status and commands");
  Prn(OUT, 0,"      m  Altitude [");
  Prn(OUT, 0, String(Altitude)+" m]");
  if(Altitude==0){
    Prn(OUT, 0, " PLEASE SET ALTITUDE");
  }
  Prn(OUT, 1, "");

  #if defined(DS18B20)
    // Prn(OUT, 1,"      s  scan DS18B20 1wire temperature sensor");
    Prn(OUT, 0,"      s  external temperature Sensor DS18B20 [O");
      if(ExtTemp==true){
        Prn(OUT, 1,"N]");
      }else{
        Prn(OUT, 1,"FF] CONNECT SENSOR (used inaccurate internal)");
      }
  #endif
  Prn(OUT, 0,"      a  speed Alert [");
  Prn(OUT, 1, String(PulseToMetterBySecond(SpeedAlertLimit_ms))+" m/s] - not implemented");
  if(TxUdpBuffer[2]!='n'){
    Prn(OUT, 0,"      w  inactivity reboot watchdog ");
    if(RebootWatchdog>0){
      Prn(OUT, 0,"after [");
      Prn(OUT, 0, String(RebootWatchdog) );
      Prn(OUT, 1,"] minutes");
    }else{
      Prn(OUT, 1,"[disable]");
    }
    Prn(OUT, 0,"      W  inactivity clear output watchdog ");
    if(OutputWatchdog>0){
      Prn(OUT, 0,"after [");
      Prn(OUT, 0, String(OutputWatchdog) );
      Prn(OUT, 1,"] minutes");
    }else{
      Prn(OUT, 1,"[disable]");
    }
    Prn(OUT, 0,"      <  change Switch incoming UDP port [");
    Prn(OUT, 0, String(IncomingSwitchUdpPort) );
    Prn(OUT, 0,"]");
    if(IncomingSwitchUdpPort!=88){
      Prn(OUT, 0,"<-- WARNING! default is 88");
    }
    Prn(OUT, 1,"");
  }
  if(TxUdpBuffer[2] == 'm'){
    Prn(OUT, 0,"      /  set encoder range - now [");
    Prn(OUT, 0, String(NumberOfEncoderOutputs+1) );
    if(NumberOfEncoderOutputs>7){
      Prn(OUT, 1,"] (two bank)");
    }else{
      Prn(OUT, 1,"]");
    }
    Prn(OUT, 0,"      %  group buttons (select one from group) [");
    if(EnableGroupButton==true){
      Prn(OUT, 1,"ON]");
      Prn(OUT, 1,"         !  SET group buttons");
      Prn(OUT, 1,"         :  list group buttons");
    }else{
      Prn(OUT, 1,"OFF]");
    }
  }
  Prn(OUT, 0,"      *  serial debug ");
    if(EnableSerialDebug==0){
      Prn(OUT, 1,"[OFF]");
    }else if(EnableSerialDebug==1){
      Prn(OUT, 1,"[ON]");
    }else if(EnableSerialDebug==2){
      Prn(OUT, 1,"[ON-frenetic]");
    }
  if(TxUdpBuffer[2]!='n'){
    Prn(OUT, 0,"      #  network ID prefix [");
    byte ID = NET_ID;
    bitClear(ID, 0); // ->
    bitClear(ID, 1);
    bitClear(ID, 2);
    bitClear(ID, 3);
    ID = ID >> 4;
    if(EnableGroupPrefix==true && TxUdpBuffer[2]=='m'){
      Prn(OUT, 0,"x");
    }else{
      Prn(OUT, 0, String(ID, HEX) );
    }
    Prn(OUT, 0,"] hex");
    if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
      Prn(OUT, 0," (set only on controllers)");
    }
    Prn(OUT, 1,"");

    // if(HW_BCD_SW==false){
      ID = NET_ID;
      bitClear(ID, 4);
      bitClear(ID, 5);
      bitClear(ID, 6);
      bitClear(ID, 7); // <-
      Prn(OUT, 0,"         +network ID sufix [");
      Prn(OUT, 0, String(ID, HEX) );
      Prn(OUT, 0,"] hex");
      if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
        Prn(OUT, 0," (multi control group - same at all)");
      }
      Prn(OUT, 1,"");
    // }

    if(TxUdpBuffer[2]=='m'){
      Prn(OUT, 0,"      $  group network ID prefix (multi control) [");
      if(EnableGroupPrefix==true){
        Prn(OUT, 1,"ON]");
      }else{
        Prn(OUT, 1,"OFF]");
      }
    }
    if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
      Prn(OUT, 1,"      .  list detected IP switch (multi control)");
    }
  }
  #if defined(Ser2net)
    Prn(OUT, 0,"      (  change serial1 baudrate [");
    Prn(OUT, 0, String(SERIAL1_BAUDRATE) );
    Prn(OUT, 1,"]");
    Prn(OUT, 0,"      )  change ser2net IP port [");
    Prn(OUT, 0, String(SerialServerIPport) );
    Prn(OUT, 1,"]");
  #endif

  Prn(OUT, 1, "      +  change MQTT broker IP | "+String(mqtt_server_ip[0])+"."+String(mqtt_server_ip[1])+"."+String(mqtt_server_ip[2])+"."+String(mqtt_server_ip[3])+":"+String(MQTT_PORT));
  // Prn(OUT, 0, String(mqtt_server_ip));
  // Prn(OUT, 0, ":");
  // Prn(OUT, 1, String(MQTT_PORT));
  Prn(OUT, 0,"      L  change ");
  if(AprsON==true){
    Prn(OUT, 1,"CALLSIGN with ssid 6-4 | "+YOUR_CALL);
  }else{
    Prn(OUT, 1,"location | "+YOUR_CALL);
  }
  Prn(OUT, 0,"      A  APRS [");
    if(AprsON==true){
      Prn(OUT, 1,"ON]");
    }else{
      Prn(OUT, 1,"OFF]");
    }
    if(AprsON==true){
      Prn(OUT, 0,"         i  change APRS server ip:port | ");
      Prn(OUT, 1,String(aprs_server_ip[0])+"."+String(aprs_server_ip[1])+"."+String(aprs_server_ip[2])+"."+String(aprs_server_ip[3])+":"+String(AprsPort));
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
  if(TxUdpBuffer[2]!='n'){
    Prn(OUT, 1,"      &  send broadcast packet");
  }
  if(TelnetServerClients[0].connected()){
    Prn(OUT, 0,"      q  disconnect and close telnet [verified IP ");
    Prn(OUT, 0, String(TelnetServerClientAuth[0])+"."+String(TelnetServerClientAuth[1])+"."+String(TelnetServerClientAuth[2])+"."+String(TelnetServerClientAuth[3]) );
    Prn(OUT, 1,"]");
    Prn(OUT, 1,"      Q  logout with erase your IP from memory and close telnet");
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
  Prn(OUT, 1,"      @  restart device");
  Prn(OUT, 1,"---------------------------------------------");
  Prn(OUT, 1, "" );
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

// http://www.catonmat.net/blog/low-level-bit-hacks-you-absolutely-must-know/
//-------------------------------------------------------------------------------------------------------

byte IdPrefix(byte ID){
  bitClear(ID, 0);  // ->
  bitClear(ID, 1);
  bitClear(ID, 2);
  bitClear(ID, 3);
  ID = ID >> 4;
  return(ID);
}

//---------------------------------------------------------------------------------------------------------

byte IdSufix(byte ID){
  bitClear(ID, 4);
  bitClear(ID, 5);
  bitClear(ID, 6);
  bitClear(ID, 7);  // <-
  return(ID);
}

//-------------------------------------------------------------------------------------------------------
byte AsciiToHex(int ASCII){
  if(ASCII>=48 && ASCII<=57){
    return(ASCII-48);
  }else if(ASCII>=97 && ASCII<=102){
    return(ASCII-87);
  }else{
    return(255);
  }
}

//-------------------------------------------------------------------------------------------------------
/*
ID FROM TO : BROADCAST ;
ID FROM TO : CONFIRM ;
ID FROM TO : A B C ;
ID FROM TO : A B C ;

TX  0ms:b;
RX  0sm:c;
TX  0ms:123;
RX  0sm:123;
*/

void RX_UDP(){
  UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
  if (UDPpacketSize){
    UdpCommand.read(packetBuffer, 10);      // read the packet into packetBufffer
    // Print RAW
    if(EnableSerialDebug>0){
      Serial.println();
      Serial.print("RXraw [");
      Serial.print(packetBuffer[0], HEX);
      for(int i=1; i<8; i++){
        Serial.print(char(packetBuffer[i]));
      }
      Serial.print(F("] "));
      Serial.print(UdpCommand.remoteIP());
      Serial.print(":");
      Serial.print(UdpCommand.remotePort());
      Serial.println();
    }

    // ID-FROM-TO filter
    if(
      (EnableGroupPrefix==false
      && String(packetBuffer[0], DEC).toInt()==NET_ID
      && packetBuffer[1]==TxUdpBuffer[2]  // FROM Switch
      && packetBuffer[2]== 's'  // TO
      && packetBuffer[3]== B00000000
      // && packetBuffer[3]== ':'
      && packetBuffer[7]== ';')
      ||
      (EnableGroupPrefix==true
      && IdSufix(packetBuffer[0])==IdSufix(NET_ID)
      && packetBuffer[1]==TxUdpBuffer[2]  // FROM Switch
      && packetBuffer[2]== 's'  // TO
      && packetBuffer[3]== B00000000
      // && packetBuffer[3]== ':'
      && packetBuffer[7]== ';')
    ){

      if( EnableGroupPrefix==true && (
        DetectedRemoteSwPort [IdPrefix(packetBuffer[0])] == 0
        || (packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o')
        || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f' && packetBuffer[6]== 'm')
        )
      ){
        IPAddress TmpAddr = UdpCommand.remoteIP();
        DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [0]=TmpAddr[0];     // Switch IP addres storage to array
        DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [1]=TmpAddr[1];
        DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [2]=TmpAddr[2];
        DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [3]=TmpAddr[3];
        DetectedRemoteSwPort [hexToDecBy4bit(IdPrefix(packetBuffer[0]))]=UdpCommand.remotePort();
        if(EnableSerialDebug>0){
          Serial.print("Detect controller ID ");
          Serial.print(IdPrefix(packetBuffer[0]), HEX);
          Serial.print(" on IP ");
          Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [0]);
          Serial.print(".");
          Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [1]);
          Serial.print(".");
          Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [2]);
          Serial.print(".");
          Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [3]);
          Serial.print(":");
          Serial.println(DetectedRemoteSwPort [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] );
        }
        if(TxUdpBuffer[2] == 'm'){
          TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 1);
        }
      }

      // RX Broadcast / CFM
      if((packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o')
        || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f' && packetBuffer[6]== 'm')
        ){
        if(EnableSerialDebug>0){
          Serial.print("RX [");
          Serial.print(packetBuffer[0], HEX);
          for(int i=1; i<8; i++){
            Serial.print(char(packetBuffer[i]));
          }
          Serial.print(F("] "));
          Serial.print(UdpCommand.remoteIP());
          Serial.print(":");
          Serial.println(UdpCommand.remotePort());
        }
        if(packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o'){
          TxUDP('s', packetBuffer[2], 'c', 'f', 'm', 1);    // 0=broadcast, 1= direct to RX IP
          if(TxUdpBuffer[2] == 'm'){
            TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 1);
          }
        }

      // RX DATA
      }else{
        if(EnableGroupButton==true){
          CheckGroup();
        }else{
          ShiftOutByte[0] = String(packetBuffer[4], DEC).toInt();    // Bank0
        }
        ShiftOutByte[1] = String(packetBuffer[5], DEC).toInt();    // Bank1
        ShiftOutByte[2] = String(packetBuffer[6], DEC).toInt();    // Bank2

        // SHIFT OUT
        #if defined(ShiftOut)
          // digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
          // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
          // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
          // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
          // digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
          if(EnableSerialDebug>0){
            Serial.println("ShiftOut");
          }
        #endif

        if(EnableSerialDebug>0){
          // Serial.println();
          Serial.print(F("RX ["));
          Serial.print(packetBuffer[0], HEX);
          for(int i=1; i<4; i++){
            Serial.print(char(packetBuffer[i]));
          }
          Serial.print((byte)packetBuffer[4], BIN);
          Serial.print(F("|"));
          Serial.print((byte)packetBuffer[5], BIN);
          Serial.print(F("|"));
          Serial.print((byte)packetBuffer[6], BIN);
          Serial.print(F(";] "));
          Serial.print(UdpCommand.remoteIP());
          Serial.print(F(":"));
          Serial.println(UdpCommand.remotePort());
        }
        if(UdpCommand.remotePort() != DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))] && EnableGroupPrefix==true){
          // if(EnableSerialDebug>0){
            Serial.print(F("** Change ip-port ID "));
            Serial.print(IdPrefix(packetBuffer[0]), HEX);
            Serial.print(F(" (OLD-"));
            Serial.print(DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))]);
            Serial.print(F(" NEW-"));
            Serial.print(UdpCommand.remotePort());
            Serial.println(F(") **"));
          // }
          DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))]=UdpCommand.remotePort();
        }
        TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
      }
      WatchdogTimer=millis();
      // activate
      if(OutputWatchdog==123456){
        OutputWatchdog=EEPROM.readUInt(27);
      }
    } // filtered end
    else{
      if(EnableSerialDebug>0){
        Serial.println(F("   Different NET-ID, or bad packet format"));
      }
    }
    memset(packetBuffer, 0, sizeof(packetBuffer));   // Clear contents of Buffer
  } //end IfUdpPacketSice
}
//-------------------------------------------------------------------------------------------------------

void CheckGroup(){
  int ChangeBit=9;
  int NumberOfChange=0;
  for (int i=0; i<8; i++){
    if(bitRead(packetBuffer[4], i)!=bitRead(ShiftOutByte[0], i)){
      ChangeBit=i;
      NumberOfChange++;
    }
  }
  // Serial.print("ChangeBit|NumberOfChange ");
  // Serial.print(ChangeBit+1);
  // Serial.print(" ");
  // Serial.println(NumberOfChange);

  ShiftOutByte[0] = String(packetBuffer[4], DEC).toInt();    // Bank0
  if(NumberOfChange==1){
    // Serial.println("clearGroup");
    NumberOfChange=0;
    for (int i=0; i<8; i++){
      if(GroupButton[ChangeBit]==GroupButton[i] && ChangeBit!=i){
        bitClear(ShiftOutByte[0], i);
        NumberOfChange++;
        // Serial.print("Bitclear ");
        // Serial.println(i);
      }
    }
    if(NumberOfChange>0){
      bitSet(ShiftOutByte[0], ChangeBit);
    }
  }
}
//-------------------------------------------------------------------------------------------------------

unsigned char hexToDecBy4bit(unsigned char hex)
// convert a character representation of a hexidecimal digit into the actual hexidecimal value
{
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}

//-------------------------------------------------------------------------------------------------------

void TxUDP(byte FROM, byte TO, byte A, byte B, byte C, int DIRECT){

  // TxUdpBuffer[0] = NET_ID;
  TxUdpBuffer[1] = FROM;
  // TxUdpBuffer[2] = TO;

  // if(TxUdpBuffer[2]=='m' && ( EnableGroupPrefix==true || EnableGroupButton==true ) ){
  //   TxUdpBuffer[3] = B00101101;           // -  multi control || GroupButton
  // }else{
  //   TxUdpBuffer[3] = B00111010;           // :
  // }

  TxUdpBuffer[3] = B00000000;
    // multi control
    if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
      bitSet(TxUdpBuffer[3], 0);
    }
    // group button
    if(TxUdpBuffer[2]=='m' && EnableGroupButton==true){
      bitSet(TxUdpBuffer[3], 1);
    }

  TxUdpBuffer[4] = A;
  TxUdpBuffer[5] = B;
  TxUdpBuffer[6] = C;
  TxUdpBuffer[7] = B00111011;           // ;

  // BROADCAST
  if(A=='b' && B=='r' && C=='o'){  // b r o
    if(TxUdpBuffer[2] == 'm'){
      TxUdpBuffer[6] = NumberOfEncoderOutputs;
    }
    // direct
    if(DIRECT==0){
      RemoteSwIP = ~ETH.subnetMask() | ETH.gatewayIP();
      if(EnableSerialDebug>0){
        Serial.print(F("TX broadcast ["));
      }
    }else{
      RemoteSwIP = UdpCommand.remoteIP();
      if(EnableSerialDebug>0){
        Serial.print(F("TX direct ["));
      }
    }
    UdpCommand.beginPacket(RemoteSwIP, BroadcastPort);

  // CFM
  }else if(A=='c' && B=='f' && C=='m'){  // cfm
      if(TxUdpBuffer[2] == 'm'){
        TxUdpBuffer[6] = NumberOfEncoderOutputs;
      }
      if(EnableSerialDebug>0){
        Serial.print(F("TX direct ["));
      }
      UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());

  // DATA
  }else{
    RemoteSwIP = UdpCommand.remoteIP();
    if(EnableSerialDebug>0){
      Serial.print(F("TX ["));
    }
    UdpCommand.beginPacket(RemoteSwIP, UdpCommand.remotePort());
  }

  // send
  if(EnableGroupPrefix==false){
    UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
    UdpCommand.endPacket();
    if(EnableSerialDebug>0){
      Serial.print(TxUdpBuffer[0], HEX);
      Serial.print(char(TxUdpBuffer[1]));
      Serial.print(char(TxUdpBuffer[2]));
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[3], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[4], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[5], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[6], BIN);
      Serial.print(char(TxUdpBuffer[7]));
      Serial.print(F("] "));
      Serial.print(RemoteSwIP);
      Serial.print(F(":"));
      Serial.print(UdpCommand.remotePort());
      #if defined(WIFI)
        Serial.print(" | dBm: ");
        Serial.print(WiFi.RSSI());
      #endif
      Serial.println();
    }

  // send EnableGroupPrefix
  }else{
    // answer to RX ip
    TxUdpBuffer[0]=packetBuffer[0];   // NET_ID by RX NET_ID
    UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
    UdpCommand.endPacket();
    if(EnableSerialDebug>0){
      Serial.print(TxUdpBuffer[0], HEX);
      for (int i=1; i<4; i++){
        Serial.print(char(TxUdpBuffer[i]));
      }
      Serial.print(TxUdpBuffer[4], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[5], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[6], BIN);
      Serial.print(char(TxUdpBuffer[7]));
      Serial.print(F("] "));
      Serial.print(RemoteSwIP);
      Serial.print(F(":"));
      Serial.print(UdpCommand.remotePort());
      #if defined(WIFI)
        Serial.print(" | dBm: ");
        Serial.print(WiFi.RSSI());
      #endif
      Serial.println();
    }
    // send to all ip from storage
    IPAddress ControllerIP = UdpCommand.remoteIP();
    for (int i=0; i<16; i++){
      if(DetectedRemoteSwPort[i]!=0){
        TxUdpBuffer[0]=IdSufix(NET_ID) | i<<4;       // NET_ID by destination device
        RemoteSwIP = DetectedRemoteSw[i];
        RemoteSwPort = DetectedRemoteSwPort[i];
        if(ControllerIP!=RemoteSwIP){
          UdpCommand.beginPacket(RemoteSwIP, RemoteSwPort);
            UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
          UdpCommand.endPacket();

          if(EnableSerialDebug>0){
            Serial.print(F("TX direct ID-"));
            Serial.print(i, HEX);
            Serial.print(IdSufix(NET_ID), HEX);
            Serial.print(F(" "));
            Serial.print(RemoteSwIP);
            Serial.print(F(":"));
            Serial.print(RemoteSwPort);
            Serial.print(F(" ["));
            Serial.print(TxUdpBuffer[0], HEX);
            for (int i=1; i<8; i++){
              Serial.print(char(TxUdpBuffer[i]));
              // Serial.print(F(" "));
            }
            Serial.print("]");
            #if defined(WIFI)
              Serial.print(" WiFi dBm: ");
              Serial.print(WiFi.RSSI());
            #endif
            Serial.println();
          }
        }else{
          if(EnableSerialDebug>0){
            Serial.print(F("noTX - RX prefix "));
            Serial.print(i, HEX);
            Serial.print(F(" "));
            Serial.print(RemoteSwIP);
            Serial.print(F(":"));
            Serial.println(RemoteSwPort);
          }
        }
      }
    }
    // broadcast all prefix
    if(A=='b' && B=='r' && C=='o' && DIRECT==0 && TxUdpBuffer[2] == 'm'){
      if(EnableSerialDebug>0){
        Serial.print("TX all prefix ");
        Serial.print(RemoteSwIP);
        Serial.print(":");
        Serial.print(BroadcastPort);
        Serial.print(F(" ["));
        Serial.print("*");
        for (int i=1; i<8; i++){
          Serial.print(char(TxUdpBuffer[i]));
          // Serial.print(F(" "));
        }
        Serial.println("]");
      }
      Serial.print("*) ");
      for (int i=0; i<16; i++){
        TxUdpBuffer[0]=IdSufix(NET_ID) | (i<<4);
        if(EnableSerialDebug>0){
          Serial.print(TxUdpBuffer[0], HEX);
          Serial.print(" ");
        }
        UdpCommand.beginPacket(RemoteSwIP, BroadcastPort);
        UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();
      }
      if(EnableSerialDebug>0){
        Serial.println();
      }
    }   // end b r o

  } // end EnableGroupPrefix
}
//-------------------------------------------------------------------------------------------------------

void http(){
  // listen for incoming clients
  WiFiClient webClient = server.available();
  if (webClient) {
    Interrupts(false);
    if(EnableSerialDebug>0){
      Serial.println("WIFI New webClient");
    }
    memset(linebuf,0,sizeof(linebuf));
    charcount=0;
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (webClient.connected()) {
      if (webClient.available()) {
        char c = webClient.read();
        HTTP_req += c;
        // if(EnableSerialDebug>0){
        //   Serial.write(c);
        // }
        //read char by char HTTP request
        linebuf[charcount]=c;
        if (charcount<sizeof(linebuf)-1) charcount++;
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header

          // send a standard http response header
          webClient.println(F("HTTP/1.1 200 OK"));
          webClient.println(F("Content-Type: text/html"));
          webClient.println(F("Connection: close"));  // the connection will be closed after completion of the response
          webClient.println();
          webClient.println(F("  <!DOCTYPE html>"));
          webClient.println(F("  <html>"));
          webClient.println(F("      <head>"));
          webClient.println(F("          <meta http-equiv=\"Content-Type\" content=\"text/html;charset=utf-8\"/>"));
          webClient.println(F("          <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"));
          // webClient.println(F("          <meta http-equiv=\"refresh\" content=\"10\">"));
          webClient.println(F("          <link rel=\"stylesheet\" type=\"text/css\" href=\"https://remoteqth.com/mqtt-wall/style.css\">"));
          // TITLE
          webClient.print(F("           <title>WX station "));
          webClient.print(YOUR_CALL);
          webClient.println(F("</title>"));
          // END TITLE
          webClient.println(F("          <link rel=\"apple-touch-icon\" sizes=\"180x180\" href=\"style/apple-touch-icon.png\">"));
          webClient.println(F("          <link rel=\"mask-icon\" href=\"style/safari-pinned-tab.svg\" color=\"#5bbad5\">"));
          webClient.println(F("          <link rel=\"icon\" type=\"image/png\" href=\"style/favicon-32x32.png\" sizes=\"32x32\">"));
          webClient.println(F("          <link rel=\"icon\" type=\"image/png\" href=\"style/favicon-16x16.png\" sizes=\"16x16\">"));
          webClient.println(F("          <link rel=\"manifest\" href=\"style/manifest.json\">"));
          webClient.println(F("          <link rel=\"shortcut icon\" href=\"style/favicon.ico\">"));
          webClient.println(F("          <meta name='apple-mobile-web-app-capable' content='yes'>"));
          webClient.println(F("          <meta name='mobile-web-app-capable' content='yes'>"));
          webClient.println(F("          <meta name=\"msapplication-config\" content=\"style/browserconfig.xml\">"));
          webClient.println(F("          <meta name=\"theme-color\" content=\"#ffffff\">"));
          webClient.println(F("          <script type=\"text/javascript\">"));
          webClient.println(F("          var config = {"));
          webClient.println(F("              server: {"));
          webClient.print(F("                  uri: \"ws://"));
          webClient.print(mqtt_server_ip[0]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[1]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[2]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[3]);
          webClient.println(":1884/\",");
          webClient.println(F("              },"));
          // TOPIC
          webClient.print(F("              defaultTopic: \""));
          webClient.print(YOUR_CALL);
          webClient.println(F("/WX/#\","));
          // END TOPIC
          webClient.println(F("              showCounter: true,"));
          webClient.println(F("              alphabeticalSort: true,"));
          webClient.println(F("              qos: 0"));
          webClient.println(F("          };"));
          webClient.println(F("          </script>"));
          // END TOPIC
          webClient.println(F("      </head>"));
          webClient.println(F("      <body>"));
          webClient.print(F("          <div id=\"frame\" "));
          webClient.println(F(">"));
          webClient.println(F("              <div id=\"footer\">"));
          webClient.println(F("                  <p class=\"status\" style=\"font-size: 150%;\">"));
          // STATUS
          webClient.print(F("Uptime: "));
          if(millis() < 60000){
            webClient.print(millis()/1000);
            webClient.print(F(" seconds"));
          }else if(millis() > 60000 && millis() < 3600000){
            webClient.print(millis()/60000);
            webClient.print(F(" minutes"));
          }else if(millis() > 3600000 && millis() < 86400000){
            webClient.print(millis()/3600000);
            webClient.print(F(" hours"));
          }else{
            webClient.print(millis()/86400000);
            webClient.print(F(" days"));
          }
          webClient.print(F(" | version: "));
          webClient.println(REV);
          webClient.print(F(" | eth mac: "));
          for (int i = 0; i < 6; i++) {
            webClient.print(ETH.macAddress()[i], HEX);
            webClient.print(F(":"));
          }
          webClient.println();

          webClient.print(F(" | dhcp: "));
          if(DHCP_ENABLE==1){
            webClient.print(F("ON"));
          }else{
            webClient.print(F("OFF"));
          }
          webClient.print(F(" | ip: "));
          webClient.println(ETH.localIP());
          // webClient.print(F(" | utc from ntp: "));
          // webClient.println(F("timeClient.getFormattedTime()"));
          // webClient.println(F("<br>MQTT subscribe command: $ mosquitto_sub -v -h mqttstage.prusa -t prusa-debug/prusafil/extrusionline/+/#"));
          webClient.print(F(" | Broker ip: "));
          webClient.print(mqtt_server_ip[0]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[1]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[2]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[3]);
          webClient.print(F(":"));
          webClient.print(MQTT_PORT);
          webClient.print(F(" | Refresh time "));
          webClient.print(MeasureTimer[1]/60000);
          webClient.println(F(" min"));
          if(AprsON==true){
            webClient.print(F(" | <a href=\"https://aprs.fi/#!call="));
            webClient.print(YOUR_CALL);
            webClient.println(F("\" target=_blank>APRS</a>"));
          }
          #if defined(OTAWEB)
            webClient.print(F(" | <a href=\"http://"));
            webClient.println(ETH.localIP());
            webClient.print(F(":82/update\" target=_blank>Upload FW</a>"));
          #endif
          // END STATUS
          webClient.println(F("              </p>"));
          webClient.println(F("              </div>"));
          webClient.println(F("              <div id=\"header\">"));
          webClient.println(F("                  <div id=\"topic-box\">"));
          webClient.println(F("                      <input type=\"text\" id=\"topic\" value=\"\" title=\"Topic to subscribe\">"));
          webClient.println(F("                  </div>"));
          webClient.println(F("              </div>"));
          webClient.println(F("              <div id=\"toast\"></div>"));
          webClient.println(F("              <section class=\"messages\"></section>"));
          webClient.println(F("              <div id=\"footer\">"));
          webClient.println(F("                  <p class=\"status\">"));
          webClient.println(F("                      Client <code id=\"status-client\" title=\"Client ID\">?</code> is "));
          webClient.println(F("                      <code id=\"status-state\" class=\"connecting\"><em>&bull;</em> <span>connecting...</span></code> to "));
          webClient.println(F("                      <code id=\"status-host\">?</code>"));
          webClient.println(F("                      <em>via</em> MQTT Wall 0.3.0 (<a href=\"https://github.com/bastlirna/mqtt-wall\">github</a>)"));
          webClient.println(F("                      | <a href=\"https://remoteqth.com/wiki/\" target=\"_blank\">WX Wiki</a>."));
          webClient.println(F("                  </p>"));
          webClient.println(F("              </div>"));
          webClient.println(F("          </div>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://code.jquery.com/jquery-2.1.4.min.js\"></script>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://code.jquery.com/color/jquery.color-2.1.2.min.js\"></script>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://cdnjs.cloudflare.com/ajax/libs/paho-mqtt/1.0.1/mqttws31.min.js\"></script>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://remoteqth.com/mqtt-wall/wall.js\"></script>"));
          webClient.println(F("      </body>"));
          webClient.println(F("  </html>"));

          if(EnableSerialDebug>0){
            Serial.print(HTTP_req);
          }
          HTTP_req = "";

          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
          // if (strstr(linebuf,"GET /h0 ") > 0){digitalWrite(GPIOS[0], HIGH);}else if (strstr(linebuf,"GET /l0 ") > 0){digitalWrite(GPIOS[0], LOW);}
          // else if (strstr(linebuf,"GET /h1 ") > 0){digitalWrite(GPIOS[1], HIGH);}else if (strstr(linebuf,"GET /l1 ") > 0){digitalWrite(GPIOS[1], LOW);}

          // you're starting a new line
          currentLineIsBlank = true;
          memset(linebuf,0,sizeof(linebuf));
          charcount=0;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    webClient.stop();
   if(EnableSerialDebug>0){
     Serial.println("WIFI webClient disconnected");
     MeasureTimer[0]=millis()+5000-MeasureTimer[1];
   }
   Interrupts(true);
  }
}
//-------------------------------------------------------------------------------------------------------

void EthEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH  Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH  Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH  MAC: ");
      Serial.println(ETH.macAddress());
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

          char charbuf[50];
           // memcpy( charbuf, ETH.macAddress(), 6);
           ETH.macAddress().toCharArray(charbuf, 18);
           // charbuf[6] = 0;
          if (mqttClient.connect(charbuf)){
            // Serial.println(charbuf);
            Prn(3, 1, String(charbuf));
            mqttReconnect();
            AfterMQTTconnect();
          }
        }
      #endif
      ListCommands(0);

      // EnableSerialDebug=1;
      TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
      if(TxUdpBuffer[2] == 'm'){
        TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
      }
      // EnableSerialDebug=0;
      break;

    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH  Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
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
  // charbuf[6] = 0;
  Prn(3, 0, "MQTT");
  char charbuf[50];
  // memcpy( charbuf, ETH.macAddress(), 6);
  ETH.macAddress().toCharArray(charbuf, 18);
  if (mqttClient.connect(charbuf)) {
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
  byte* p = (byte*)malloc(length);
  memcpy(p,payload,length);
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

      Azimuth();
      MqttPubString("WindDir-azimuth", String(WindDir), false);

      #if defined(DS18B20)
        if(ExtTemp==true){
          sensors.requestTemperatures();
          float temperatureC = sensors.getTempCByIndex(0);
          MqttPubString("Temperature-Celsius", String(temperatureC), false);
        }else{
          MqttPubString("Temperature-Celsius", String(htu.readTemperature()), false);
        }
      #endif

      MqttPubString("WindSpeedMaxPeriod-mps", String(PulseToMetterBySecond(PeriodMinRpmPulse)), false);

      #if defined(BMP280)
        if(BMP280enable==true){
          MqttPubString("Pressure-hPa", String(Babinet(double(bmp.readPressure()), double(htu.readTemperature()*1.8+32))/100), false);
        }
      #endif

      #if defined(HTU21D)
        if(HTU21Denable==true){
          MqttPubString("HumidityRel-Percent", String(constrain(htu.readHumidity(), 0, 100)), false);
        }
      #endif

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

        String MAClocalAddrString = ETH.macAddress();   // to string
        MAClocalAddrString.toCharArray( mqttTX, 50 );                          // to array
        path2 = String(YOUR_CALL) + "/WX/mac";
        path2.toCharArray( mqttPath, 100 );
          mqttClient.publish(mqttPath, mqttTX, true);
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
  char charbuf[50];
   // memcpy( charbuf, mac, 6);
   ETH.macAddress().toCharArray(charbuf, 10);
   charbuf[6] = 0;
  Interrupts(false);
  // if(EnableEthernet==1 && MQTT_ENABLE==1 && EthLinkStatus==1 && mqttClient.connected()==true){
  if(mqttClient.connected()==true){
    if (mqttClient.connect(charbuf)) {
      String topic = String(YOUR_CALL) + "/WX/"+TOPIC;
      topic.toCharArray( mqttPath, 50 );
      DATA.toCharArray( mqttTX, 50 );
      mqttClient.publish(mqttPath, mqttTX, RETAIN);
    }
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
        Prn(1, 0,String((TelnetLoginFailsBanTimer[1]-millis()-TelnetLoginFailsBanTimer[0])/1000));
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
      EEPROM.commit();
      break; }
  }
}

//-------------------------------------------------------------------------------------------------------

void AuthQ(int NR, bool BAD){
  Prn(1, 0,"What character is at ");
  RandomNumber=random(0, strlen(key));
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
    if(!getLocalTime(&timeinfo)){
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
