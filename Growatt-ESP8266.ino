/* 0. User_Setting_ változókat egyénileg be kell állítani  */
/*
  ESPDuino ESP13-Module
  Upload Speed 256000
  Realterm beállítás: 66 Col
  Program letöltése:
  0. Arduino fejlesztőrendszer bezárása (persza csak ha ezt elolvastad...)
  1. Ellenőrizni kell az IP és egyéb beállításokat a * LETÖLTÉSI BEÁLLÍTÁSOK résznél, projekt mentése
  2. Modbus Poll: 0. holding regiszter User_Setting_OTA_Reset beírásával az ESP újraindul OTA módban
  3. Arduino fejlesztőrendszer elindítása ezzel a programmal
  4. Megjelenik a portok között az ESP IP címe, ezt kell engedélyezni a letöltéshez  
  5. Letöltésre 2 perc áll rendelkezése
  6. A letöltés után az ESP újraindul, de ha nem volt letöltés 1 percen belül, akkor az
     NTP hibás adatokkal fog dolgozni, ezért újra kell indítani ez eszközt a 
     0. holding regiszter User_Setting_Reset beírásával

	 http paraméterek
	 
  /Restart:Controller
  /Restart:WiFi
	 
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>  //Ticker Library
#include <ArduinoOTA.h>
#include "EEPROM.h"

// ************************************************* LETÖLTÉSI BEÁLLÍTÁSOK

#define Station_Address_1
// 1. IP: 192.168.1.31
#ifdef Station_Address_1
  IPAddress local_IP(User_Setting_IP);
  const char *title = "User_Setting_Name";
  unsigned int PowerTrendMaximum_01W = User_Setting_Power_Max;
  unsigned int ETodayTrendMaximum_01kWh = User_Setting_kWh;
#endif
#ifdef Station_Address_2
  // 2. IP: 192.168.1.32
  IPAddress local_IP(192, 168, 1, 32);
  const char *title = "Growatt 2: 1500W";
  unsigned int PowerTrendMaximum_01W = 15000;
  unsigned int ETodayTrendMaximum_01kWh = 100;
#endif  
#ifdef Station_Address_3
  // Teszt. IP: 192.168.1.33
  IPAddress local_IP(192, 168, 1, 33);
  const char *title = "Teszt";
  unsigned int PowerTrendMaximum_01W = 15000;
  unsigned int ETodayTrendMaximum_01kWh = 100;
#endif


// 1. int PowerTrendMaximum_01W = 44000; ETodayMaximum_01kWh = 300;
// 2. int PowerTrendMaximum_01W = 15000; ETodayMaximum_01kWh = 100;

// ************************************************* LETÖLTÉSI BEÁLLÍTÁSOK VÉGE

IPAddress gateway(User_Setting_Gateway);
IPAddress subnet(User_Setting_Subnet);
IPAddress primaryDNS(8, 8, 8, 8); 
IPAddress secondaryDNS(8, 8, 4, 4); 

const char *ssid = "User_Setting_SSID";
const char *password = "User_Setting_Password";
Ticker blinker;

//Your UTC Time Zone Differance  India +5:30
char TIMEZONE_HH = 1; //User_Setting_TimeZone
char TIMEZONE_MM = 0;

#define Timeout 30 //Ha nincs adat az inverter felöl, akkor 30 másodperc után az adatok nullázódnak
unsigned int TimeoutCounter;
 
unsigned int localPort = 2390;      // local port to listen for UDP packets

unsigned int PowerTrendSize; 
unsigned int PowerTrendHeight = 500;
#define PowerTrendSampleTime 5
unsigned int PowerTrend[60*24/PowerTrendSampleTime+1];
unsigned int ETodayTrend[60*24/PowerTrendSampleTime+1];

unsigned long P_Avg_Summ = 0;
unsigned long P_Avg_Count = 0;

unsigned long timeNow = 0;
unsigned long timeLast = 0;
int seconds = 0;
int minutes = 0;
int hours = 0;
int days = 0;

int Inverter_seconds = 0;
int Inverter_minutes = 0;
int Inverter_hours = 0;

byte NTP_hours, NTP_minutes, NTP_seconds;

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;

#define ModbusTCP_port 502 //User_Setting_ModbusTCP port
#define WebPage_port 80 // User_Setting_WebPort
#define LED 2 //Define blinking LED pin

#define LED_ON 0
#define LED_OFF 1

//////// Required for Modbus TCP / IP /// Requerido para Modbus TCP/IP /////////
#define maxInputRegister 254
#define maxHoldingRegister 254

#define MB_FC_NONE 0
#define MB_FC_READ_REGISTERS 3 //implemented
#define MB_FC_READ_INPUT_REGISTERS 4        //implemented
#define MB_FC_WRITE_REGISTER 6 //implemented
#define MB_FC_WRITE_MULTIPLE_REGISTERS 16 //implemented
//
// MODBUS Error Codes
//
#define MB_EC_NONE 0
#define MB_EC_ILLEGAL_FUNCTION 1
#define MB_EC_ILLEGAL_DATA_ADDRESS 2
#define MB_EC_ILLEGAL_DATA_VALUE 3
#define MB_EC_SLAVE_DEVICE_FAILURE 4
//
// MODBUS MBAP offsets
//
#define MB_TCP_TID 0
#define MB_TCP_PID 2
#define MB_TCP_LEN 4
#define MB_TCP_UID 6
#define MB_TCP_FUNC 7
#define MB_TCP_REGISTER_START 8
#define MB_TCP_REGISTER_NUMBER 10

byte ByteArray[260];
unsigned int MBHoldingRegister[maxHoldingRegister];
unsigned int MBInputRegister[maxInputRegister];

#define CommStatusFault 0
#define CommStatusRxOK 1
#define CommStatusDataOK 2

#define Addr_PV_Voltage_01V 0
#define Addr_Grid_Voltage_01V 1
#define Addr_Grid_Frv_001Hz 2
#define Addr_Power_01W 3
#define Addr_Temperature_01C 4
#define Addr_Inverter_Status 5
#define Addr_Fault_Code 6
#define Addr_E_Today_01kWh 7
#define Addr_E_Total_Hi 9
#define Addr_E_Total_Lo 8
#define Addr_T_Total_Hi 11
#define Addr_T_Total_Lo 10
#define Addr_CommStatus 12
#define Addr_E_Today_Summ_Corr 13 /* Arra való, hogy a PVOutput a Addr_E_Today_01kWh 100Wh lépései alapján rosszul számolja a teljesítmény átlagot,
ami hobás megjelenítéshez vezet. Ez az integrátor számolja a Wh-kat a PVOutput jobb működéséért. Ezt az értéket hozzá kell adni az E_Today_01kWh értékéhez */

float E_Today_Summ_Integr = 0;

#define Addr_OTA_Start 0 // User_Setting_OTA_Reset a letöltéshez a jelszó, a 0 holding regiszterbe történő beírással
#define Addr_NTP_hours 1 
#define Addr_NTP_minutes 2 
#define Addr_NTP_seconds 3 
#define Addr_NTP_Status 4 

#define NTP_OK 1 
#define NTP_Not_OK 0

#define USB 0 
#define GPIO 1

bool Serial_Mode = 0;

class PersistentVariables
{
  public:
  // variables
  int Reset_By_OTA;
  int Vars_OK;
  
  // Methods
  void save();
  void get();
  void init();
} EEPROM_Vars;

void PersistentVariables::save()
{
  EEPROM.put(0,EEPROM_Vars);
  EEPROM.commit();
}

void PersistentVariables::get()
{
  EEPROM.begin(sizeof(EEPROM_Vars));
  EEPROM.get(0,EEPROM_Vars);
}

void PersistentVariables::init()
{
  Reset_By_OTA = 0;
  Vars_OK = 111;

  EEPROM.put(0,EEPROM_Vars);
  EEPROM.commit();  
}
//////////////////////////////////////////////////////////////////////////

ESP8266WebServer server ( WebPage_port );
WiFiServer MBServer(ModbusTCP_port);

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
//const char* ntpServerName = "time.nist.gov";
const char* ntpServerName = "time.kfki.hu";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

void RuntimeCalc()
{
  timeNow = millis(); // the number of milliseconds that have passed since boot
  float TrendValue;
  if ( (unsigned long)abs(timeNow - timeLast) >= 1000)
  {
    if ((MBInputRegister[Addr_CommStatus] == CommStatusDataOK) && (MBInputRegister[Addr_Power_01W] != 0))
    {
      Inverter_seconds = Inverter_seconds + 1;
      E_Today_Summ_Integr += (float) MBInputRegister[Addr_Power_01W] / 36000.0; 
      if (Inverter_seconds >= 60) 
      {
        Inverter_seconds = 0;      
        Inverter_minutes = Inverter_minutes + 1;
        if (Inverter_minutes >= 60)
        {
          Inverter_minutes = 0;
          Inverter_hours = Inverter_hours + 1;       
        }        
      }
    }   
    timeLast = timeNow;
    seconds = seconds + 1;
    if (TimeoutCounter > 0)
    {
      TimeoutCounter--;
    }
    else
    {
      MBInputRegister[Addr_PV_Voltage_01V] = 0;
      MBInputRegister[Addr_Grid_Voltage_01V] = 0;
      MBInputRegister[Addr_Grid_Frv_001Hz] = 0;
      MBInputRegister[Addr_Power_01W] = 0;
    }
    if (MBHoldingRegister[Addr_NTP_Status] != NTP_OK)
    {
      {if ((seconds == 1) ||
           (seconds == 11) || 
           (seconds == 21) ||
           (seconds == 31) ||
           (seconds == 41) ||
           (seconds == 51))
        {
          NTP_Refresh();
        }
      }
    }
    if (seconds >= 60) 
    {
      seconds = 0;      
      minutes = minutes + 1; 
      if (((NTP_minutes + NTP_hours * 60) % PowerTrendSampleTime) == 0)
      {
        if (P_Avg_Count != 0)
        {
          TrendValue = (float) P_Avg_Summ / P_Avg_Count;
          TrendValue = TrendValue / (float) PowerTrendMaximum_01W * (float) PowerTrendHeight;     
          PowerTrend[(NTP_minutes + NTP_hours * 60) / PowerTrendSampleTime] = (unsigned int) TrendValue;
          TrendValue = (float) MBInputRegister[Addr_E_Today_01kWh] / (float) ETodayTrendMaximum_01kWh * (float) PowerTrendHeight;     
          ETodayTrend[(NTP_minutes + NTP_hours * 60) / PowerTrendSampleTime] = (unsigned int) TrendValue;                    
          P_Avg_Count = 0;
          P_Avg_Summ = 0;
        }
        else
        {
          TrendValue = (float) MBInputRegister[Addr_E_Today_01kWh] / (float) ETodayTrendMaximum_01kWh * (float) PowerTrendHeight;     
          ETodayTrend[(NTP_minutes + NTP_hours * 60) / PowerTrendSampleTime] = (unsigned int) TrendValue;     
        }
      }    
    }
    if (minutes >= 60)
    {
      minutes = 0;
      hours = hours + 1;       
    }
    if (hours >= 24)
    {
      hours = 0;
      MBHoldingRegister[Addr_NTP_Status] = NTP_OK;
      days = days + 1;
      Inverter_seconds = 0;
      Inverter_minutes = 0;
      Inverter_hours = 0;             
    }  
    NTP_seconds = NTP_seconds + 1;
    if (NTP_seconds >= 60) 
    {
      NTP_seconds = 0;
      NTP_minutes = NTP_minutes + 1; 
    }
    if (NTP_minutes >= 60)
    {
      NTP_minutes = 0;
      NTP_hours = NTP_hours + 1; 
    }
    if (NTP_hours >= 24)
    {
      NTP_hours = 0;
      MBInputRegister[Addr_E_Today_01kWh] = 0;
      MBInputRegister[Addr_E_Today_Summ_Corr] = 0;
      E_Today_Summ_Integr = 0;
    }
  }
  MBHoldingRegister[Addr_NTP_hours] = NTP_hours;
  MBHoldingRegister[Addr_NTP_minutes] = NTP_minutes;
  MBHoldingRegister[Addr_NTP_seconds] = NTP_seconds;
}

void Serial_Switch(bool Mode)
{  
  if (Serial_Mode != Mode)
  {
    Serial.swap();
    Serial_Mode = Mode;
  }
}

void handleRoot() {
  char temp[1023];

  snprintf ( temp, 1023,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='25'/>\
    <title>%s</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Hello from Growatt</h1>\
    <p>Converter Uptime: %04d:%02d:%02d:%02d</p>\
    <p>Inverter Uptime: %02d:%02d:%02d</p>\
    <p>Local time: %02d:%02d:%02d ",
    title,
    days, hours, minutes , seconds,
    Inverter_hours, Inverter_minutes , Inverter_seconds,
    NTP_hours, NTP_minutes , NTP_seconds);
    if (MBHoldingRegister[Addr_NTP_Status] == NTP_OK)
    {
      snprintf ( temp + strlen(temp), 1023,", NTP OK </p>\
      <p>");
    }
    else
    {
      snprintf ( temp + strlen(temp), 1023,", NTP Error </p>\
      <p>");
    }
    if (MBInputRegister[Addr_CommStatus] == CommStatusDataOK)
    {
      snprintf ( temp + strlen(temp), 1023,"Inverter Comm: OK </p>\
      ");
    }
    else
    {
      if (MBInputRegister[Addr_CommStatus] == CommStatusRxOK)
      {
        snprintf ( temp + strlen(temp), 1023,"Inverter Comm: Error, Read Data But Cannot Recognise </p>\
        ");        
      }
      else
      {
        snprintf ( temp + strlen(temp), 1023,"Inverter Comm: Error, Not respond </p>\
        ");        
      }
    }
    snprintf ( temp + strlen(temp), 1023,
    "<p>Power: %.1f W</p>\
    <p>PV: %.1f V</p>\
    <p>Grid: %.1f V</p>\
    <p>Inverter Temperature: %.1f Celsius</p>\
    <p>E Today: %.3f kWh</p>\
    <p>E Total: %d kWh</p>\
    <p>Inverter Runtime: %d h</p>\
    <img src=\"/P_Trend.svg\" />\
  </body>\
</html>",
    (float) MBInputRegister[Addr_Power_01W] / 10,
    (float) MBInputRegister[Addr_PV_Voltage_01V] / 10,
    (float) MBInputRegister[Addr_Grid_Voltage_01V] / 10,
    (float) MBInputRegister[Addr_Temperature_01C] / 10,
    (float) MBInputRegister[Addr_E_Today_01kWh] / 10 + ((float) MBInputRegister[Addr_E_Today_Summ_Corr]) / 1000.0,
    (unsigned long) (MBInputRegister[Addr_E_Total_Hi] * 65536 + MBInputRegister[Addr_E_Total_Lo]) / 10,
    (unsigned long) (MBInputRegister[Addr_T_Total_Hi] * 65536 + MBInputRegister[Addr_T_Total_Lo]) / 3600
  );
/*
  <p></p>\
  <img src=\"/E_Trend.svg\" />\
 */
  server.send ( 200, "text/html", temp );
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, "text/plain", message );
}

void setup ( void ) {
  PowerTrendSize = 60 * 24 / PowerTrendSampleTime;
  MBHoldingRegister[Addr_NTP_Status] = NTP_Not_OK;
// TEST START
/*float TrendValue;
  MBInputRegister[Addr_Power_01W] = 10000;
  TrendValue = (float) MBInputRegister[Addr_Power_01W] / (float) PowerTrendMaximum_01W *  (float) PowerTrendHeight; 
  for (int i = 0; i < PowerTrendSize; i++)
  {        
    PowerTrend[i] = (int) TrendValue;
  }
*/  
// TEST END
  pinMode ( LED, OUTPUT );
  pinMode (15, OUTPUT);
  ESP.wdtDisable();
  digitalWrite ( LED, LED_ON );
  Serial.begin ( 9600, SERIAL_8N1 );

  EEPROM_Vars.get();
  if (EEPROM_Vars.Vars_OK != 111)
  {
    Serial.println("Failed to load persistent variables... initializing to defaults!");
    Serial.println(EEPROM_Vars.Vars_OK);
    EEPROM_Vars.init();
  }
  else
  {
    Serial.println("Load persistent variables OK.");
  }

  Serial.printf("\n\nSdk version: %s\n", ESP.getSdkVersion());
  Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
  Serial.printf("Boot Version: %u\n", ESP.getBootVersion());
  Serial.printf("Boot Mode: %u\n", ESP.getBootMode());
  Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());
  
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event)
  {
    Serial.print("Station connected, IP: ");
    Serial.println(WiFi.localIP());
    MBServer.begin();
    server.begin();
  });

  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
  {
    Serial.println("Station disconnected");
  });
  
  WiFi_Start();
  blinker.attach_ms(10, InverterHandle);
  // Serial.swap(); // Ez kapcsolja le az USB-ről a GPIO13,15-re a serial0-t
  Serial_Switch(GPIO);
  digitalWrite ( LED, LED_OFF );
  ESP.wdtEnable(5000);
}

//=======================================================================
//  send an NTP request to the time server at the given address
//=======================================================================
unsigned long sendNTPpacket(IPAddress& address)
{
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

void OTAHandle()
{
  if (MBHoldingRegister[Addr_OTA_Start] == User_Setting_OTA_Reset)
  {
    MBHoldingRegister[Addr_OTA_Start] = 0;
    EEPROM_Vars.Reset_By_OTA = 1;
    EEPROM_Vars.save();
    delay(100);
    ESP_Restart();
  }  
  if (MBHoldingRegister[Addr_OTA_Start] == User_Setting_OTA_Reset)
  {
    MBHoldingRegister[Addr_OTA_Start] = 0;
    ESP_Restart();  
  }  
  MBHoldingRegister[Addr_OTA_Start] = 1;  
}

void NTP_Refresh(void)
{
  WiFi.hostByName(ntpServerName, timeServerIP); 
 
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  bool not_finish = 1;
  int cb = 0;
  unsigned int i = 0;  
  while (not_finish)
  {
    delay(1);
    cb = udp.parsePacket();
    i ++;
    if ((cb == 48) || (i >= 2000)) not_finish = 0;    
    ESP.wdtFeed();
    yield();  
  }
  if (cb)
  {
    MBHoldingRegister[Addr_NTP_Status] = NTP_OK;
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (NTP_seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In NTP_seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
 
    // print the hour, minute and second:
    NTP_minutes = ((epoch % 3600) / 60);
    NTP_minutes = NTP_minutes + TIMEZONE_MM; //Add UTC Time Zone
    
    NTP_hours = (epoch  % 86400L) / 3600;    
    if(NTP_minutes > 59)
    {      
      NTP_hours = NTP_hours + TIMEZONE_HH + 1; //Add UTC Time Zone  
      NTP_minutes = NTP_minutes - 60;
    }
    else
    {
      NTP_hours = NTP_hours + TIMEZONE_HH;
    }
    
    NTP_seconds = (epoch % 60);
  }
/*  else
  {
    MBHoldingRegister[Addr_NTP_Status] = cb;
  }*/
}

void loop ( void ) { 
  Modbus_Handle();
  work();
}

void work()
{
  server.handleClient();
  delay(1);
  ESP.wdtFeed();
  RuntimeCalc();  
  OTAHandle(); 
  yield();  
}

void InverterHandle()
{
#define ReadCommandSend 0
#define ReadingActive 200
#define StartSend 400
#define Timeout 1000

  byte ReadCommand[] = {0x3F, 0x23, 0x7E, 0x34, 0x41, 0x7E, 0x32, 0x59, 0x32, 0x35, 0x30, 0x30, 0x23, 0x3F };
  byte StartCommand[] = {0x3F, 0x23, 0x7E, 0x34, 0x42, 0x7E, 0x23, 0x3F };

  static byte ReadBuffer[255];
  static unsigned int Counter = 0;

  if (Counter >= Timeout)
  {
    Counter = ReadCommandSend;
    MBInputRegister[Addr_CommStatus] = CommStatusFault;
    MBInputRegister[Addr_PV_Voltage_01V] = 0;
    MBInputRegister[Addr_Grid_Voltage_01V] = 0;
    MBInputRegister[Addr_Grid_Frv_001Hz] = 0;
    MBInputRegister[Addr_Power_01W] = 0;    
    digitalWrite(LED, LED_OFF);
  }

  if (Counter == ReadCommandSend)
  {
    Serial.write(ReadCommand, sizeof(ReadCommand));
  }

  if (Counter == StartSend)
  {
    Serial.write(StartCommand, sizeof(StartCommand));
  }

  while (Serial.available()) 
  {
    for ( int i = 254; i > 0; i-- ) 
    {
      ReadBuffer[i] = ReadBuffer[i-1];
    }      
    ReadBuffer[0] = Serial.read();        
    Counter = ReadingActive;
    if ((ReadBuffer[0] == 0x57) 
     && (ReadBuffer[31] == 0x57)
     && (ReadBuffer[31-16] == 0)
     && (ReadBuffer[31-17] == 0)
     && (ReadBuffer[31-18] == 0)
     && (ReadBuffer[31-19] == 0)
     && (ReadBuffer[31-20] == 0)
     && (ReadBuffer[31-21] == 0)
     && (ReadBuffer[31-1] != 0))
    {
      unsigned int PV_Voltage_01V = ReadBuffer[31-1]*256 + ReadBuffer[31-2];
      unsigned int Grid_Voltage_01V = ReadBuffer[31-7]*256 + ReadBuffer[31-8];
      unsigned int Grid_Frv_001Hz = ReadBuffer[31-9]*256 + ReadBuffer[31-10];
      unsigned int Power_01W = ReadBuffer[31-11]*256 + ReadBuffer[31-12];

      if ((PV_Voltage_01V > 500) && (PV_Voltage_01V < 10000) && 
      (Grid_Voltage_01V > 1500) && (Grid_Voltage_01V < 3000) && 
      (Grid_Frv_001Hz > 4500) && (PV_Voltage_01V < 5500) && 
      (Power_01W < (PowerTrendMaximum_01W * 2)))
      {
        MBInputRegister[Addr_CommStatus] = CommStatusDataOK;
        TimeoutCounter = Timeout;
        digitalWrite(LED, !(digitalRead(LED)));  //Invert Current State of LED
        MBInputRegister[Addr_PV_Voltage_01V] = PV_Voltage_01V;
        MBInputRegister[Addr_Grid_Voltage_01V] = Grid_Voltage_01V;
        MBInputRegister[Addr_Grid_Frv_001Hz] = Grid_Frv_001Hz;
        MBInputRegister[Addr_Power_01W] = Power_01W;
        P_Avg_Summ += (unsigned long) Power_01W;
        P_Avg_Count++;
        MBInputRegister[Addr_Temperature_01C] = ReadBuffer[31-13]*256 + ReadBuffer[31-14];
        MBInputRegister[Addr_Inverter_Status] = ReadBuffer[31-15];
        MBInputRegister[Addr_Fault_Code] = ReadBuffer[31-16];
        unsigned int E_Today_01kWh = ReadBuffer[31-21]*256 + ReadBuffer[31-22];
        if (MBInputRegister[Addr_E_Today_01kWh] != E_Today_01kWh)
        {
          E_Today_Summ_Integr = 0.0;
          MBInputRegister[Addr_E_Today_01kWh] = E_Today_01kWh; 
        }
        MBInputRegister[Addr_E_Today_Summ_Corr] = (unsigned int) E_Today_Summ_Integr;
        MBInputRegister[Addr_E_Total_Hi] = ReadBuffer[31-23]*256 + ReadBuffer[31-24];
        MBInputRegister[Addr_E_Total_Lo] = ReadBuffer[31-25]*256 + ReadBuffer[31-26];
        MBInputRegister[Addr_T_Total_Hi] = ReadBuffer[31-27]*256 + ReadBuffer[31-28];
        MBInputRegister[Addr_T_Total_Lo] = ReadBuffer[31-29]*256 + ReadBuffer[31-30];      
      }
    }
    else
    {
      if (MBInputRegister[Addr_CommStatus] == CommStatusFault)
      {
        MBInputRegister[Addr_CommStatus] = CommStatusRxOK; 
      }      
    }
  }  
  Counter++;
}

void Modbus_Handle()
{
  // Check if a client has connected // Modbus TCP/IP
  WiFiClient client = MBServer.available();
  if (!client) { return; }
  boolean flagClientConnected = 0;
  byte byteFN = MB_FC_NONE;
  int Start;
  int WordDataLength;
  int ByteDataLength;
  int MessageLength;
  // Modbus TCP/IP
  while (client.connected()) 
  {
    work();
    if(client.available())
    {
      flagClientConnected = 1;
      int i = 0;
      while(client.available())
      {
        ByteArray[i] = client.read();
        i++;
      }
      client.flush();      
      byteFN = ByteArray[MB_TCP_FUNC];
      Start = word(ByteArray[MB_TCP_REGISTER_START],ByteArray[MB_TCP_REGISTER_START+1]);
      WordDataLength = word(ByteArray[MB_TCP_REGISTER_NUMBER],ByteArray[MB_TCP_REGISTER_NUMBER+1]);
    }
    // Handle request
    switch(byteFN)
    {
      case MB_FC_NONE:  break;
      case MB_FC_READ_INPUT_REGISTERS:  // 04 Read Input Registers
        ByteDataLength = WordDataLength * 2;
        ByteArray[5] = ByteDataLength + 3; //Number of bytes after this one.
        ByteArray[8] = ByteDataLength;     //Number of bytes after this one (or number of bytes of data).
        for(int i = 0; i < WordDataLength; i++)
        {
            ByteArray[ 9 + i * 2] = highByte(MBInputRegister[Start + i]);
            ByteArray[10 + i * 2] =  lowByte(MBInputRegister[Start + i]);
        }
        MessageLength = ByteDataLength + 9;
        client.write((const uint8_t *)ByteArray,MessageLength);
        byteFN = MB_FC_NONE;
        break;      
      case MB_FC_READ_REGISTERS: // 03 Read Holding Registers
        ByteDataLength = WordDataLength * 2;
        ByteArray[5] = ByteDataLength + 3; //Number of bytes after this one.
        ByteArray[8] = ByteDataLength; //Number of bytes after this one (or number of bytes of data).
        for(int i = 0; i < WordDataLength; i++)
        {
          ByteArray[ 9 + i * 2] = highByte(MBHoldingRegister[Start + i]);
          ByteArray[10 + i * 2] = lowByte(MBHoldingRegister[Start + i]);
        }
        MessageLength = ByteDataLength + 9;
        client.write((const uint8_t *)ByteArray,MessageLength);
        byteFN = MB_FC_NONE;
        break;
      case MB_FC_WRITE_REGISTER: // 06 Write Holding Register
        MBHoldingRegister[Start] = word(ByteArray[MB_TCP_REGISTER_NUMBER],ByteArray[MB_TCP_REGISTER_NUMBER+1]);
        ByteArray[5] = 6; //Number of bytes after this one.
        MessageLength = 12;
        client.write((const uint8_t *)ByteArray,MessageLength);
        byteFN = MB_FC_NONE;
        break;
      case MB_FC_WRITE_MULTIPLE_REGISTERS: //16 Write Holding Registers
        ByteDataLength = WordDataLength * 2;
        ByteArray[5] = ByteDataLength + 3; //Number of bytes after this one.
        for(int i = 0; i < WordDataLength; i++)
        {
          MBHoldingRegister[Start + i] = word(ByteArray[ 13 + i * 2],ByteArray[14 + i * 2]);
        }
        MessageLength = 12;
        client.write((const uint8_t *)ByteArray,MessageLength); 
        byteFN = MB_FC_NONE;
        break;
    }
  }
  client.stop();
}

void ESP_Restart()
{
    // Serial.swap(); // Ez visszakapcsolja az USB-re a GPIO13,15-ről a serial0-t
    Serial_Switch(USB);
    ESP.restart();
}

void WiFi_Start()
{
  Serial_Switch(USB);
  WiFi.mode ( WIFI_STA );
  WiFi.begin ( ssid, password );
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  Serial.println ( "" );
  // Wait for connection
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
  Serial.println ( "" );
  Serial.print ( "Connected to " );
  Serial.println ( ssid );
  Serial.print ( "IP address: " );
  Serial.println ( WiFi.localIP() );
  if (EEPROM_Vars.Reset_By_OTA == 1)
  {
    EEPROM_Vars.Reset_By_OTA = 0;
    EEPROM_Vars.save();
    ArduinoOTA.onStart([]() 
    {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
  
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() 
    {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
    {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) 
    {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    }); 
  
    ArduinoOTA.begin();
    Serial.println ( "OTA begin" );
    for ( unsigned int i = 0; i < 60000; i++ ) //2 perc van a letöltésre
    {
      delay(1);
      ESP.wdtFeed();
      ArduinoOTA.handle();
    }
    Serial.println ( "OTA End" );              
  }
  if ( MDNS.begin ( "esp8266" ) ) {
    Serial.println ( "MDNS responder started" );
  }
  server.on ( "/", handleRoot );
  server.on ( "/P_Trend.svg", draw_P_Trend );
  server.on ( "/Restart:Controller", ESP_Restart );
  server.on ( "/Restart:WiFi", WiFi_Restart );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "Hello From Growatt" );
  } );
  server.onNotFound ( handleNotFound );
  server.begin();
  Serial.println ( "HTTP server started" );
  Serial.println("Modbus Server Begin ");
  MBServer.begin();
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  Serial_Switch(GPIO);
}

void WiFi_Stop()
{
  WiFi.disconnect(); 
  WiFi.mode(WIFI_OFF);
}

void WiFi_Restart()
{
  WiFi_Stop();
  delay(100);
  WiFi_Start();
}

void draw_P_Trend() {
  String out = "";
  char temp[256];
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"",
  out += String(PowerTrendSize * 2);
  out += "\" height=\"";
  out += String(PowerTrendHeight);
  out += "\">\n";
  out += "<style>\n";
  out += ".scaletext { font-size: 12px; text-anchor:middle; stroke: #888888; }\n";
  out += "</style>\n";
  out += "<rect width=\"",
  out += String(PowerTrendSize * 2);
  out += "\" height=\"";
  out += String(PowerTrendHeight);
  out += "\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"blue\" stroke-width=\"1\">\n";
  int Py = PowerTrend[0];
  int Ey = ETodayTrend[0];
  for (int x = 0; x < (PowerTrendSize - 1); x++) {
    int Py2 = PowerTrend[x + 1];
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\"/>\n", x * 2 , PowerTrendHeight - Py, x * 2 + 1, PowerTrendHeight - Py2);
    out += temp;
    Py = Py2;
    int Ey2 = ETodayTrend[x + 1];
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"green\"/>\n", x * 2 , PowerTrendHeight - Ey, x * 2 + 1, PowerTrendHeight - Ey2);
    out += temp;
    Ey = Ey2;        
  }
  sprintf(temp, "<line x1=\"0\" x2=\"%d\" y1=\"%d\" y2=\"%d\" stroke=\"gray\" stroke-dasharray=\"1\" />\n", PowerTrendSize * 2, PowerTrendHeight / 4, PowerTrendHeight / 4);
  out += temp;
  sprintf(temp, "<line x1=\"0\" x2=\"%d\" y1=\"%d\" y2=\"%d\" stroke=\"gray\" stroke-dasharray=\"1\" />\n", PowerTrendSize * 2, PowerTrendHeight / 2, PowerTrendHeight / 2);
  out += temp;
  sprintf(temp, "<line x1=\"0\" x2=\"%d\" y1=\"%d\" y2=\"%d\" stroke=\"gray\" stroke-dasharray=\"1\" />\n", PowerTrendSize * 2, PowerTrendHeight / 4 * 3, PowerTrendHeight / 4 * 3);
  out += temp;

  sprintf(temp, "<line x1=\"%d\" x2=\"%d\" y1=\"0\" y2=\"%d\" stroke=\"gray\" stroke-dasharray=\"1\" />\n", PowerTrendSize / 2, PowerTrendSize / 2, PowerTrendHeight);
  out += temp;

  sprintf(temp, "<line x1=\"%d\" x2=\"%d\" y1=\"0\" y2=\"%d\" stroke=\"gray\" stroke-dasharray=\"1\" />\n", PowerTrendSize, PowerTrendSize , PowerTrendHeight);
  out += temp;

  sprintf(temp, "<line x1=\"%d\" x2=\"%d\" y1=\"0\" y2=\"%d\" stroke=\"gray\" stroke-dasharray=\"1\" />\n", PowerTrendSize * 3 / 2, PowerTrendSize * 3 / 2 , PowerTrendHeight);
  out += temp;

  int Timeline;
  Timeline = (int) (((float) NTP_hours * 60 + (float) NTP_minutes) / (60 * 24) * (float) PowerTrendSize * 2);
  sprintf(temp, "<line x1=\"%d\" x2=\"%d\" y1=\"0\" y2=\"%d\" stroke=\"red\" stroke-dasharray=\"1\" />\n", Timeline, Timeline, PowerTrendHeight - 20);
  out += temp;

  int scale_pos_x = 23;  
  int scale_pos_y = 15;
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d W</text>\n",scale_pos_x, scale_pos_y, PowerTrendMaximum_01W / 10);
  out += temp;
  scale_pos_y += PowerTrendHeight / 4; 
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d W</text>\n",scale_pos_x, scale_pos_y, PowerTrendMaximum_01W / 10 / 4 * 3);
  out += temp;
  scale_pos_y += PowerTrendHeight / 4; 
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d W</text>\n",scale_pos_x, scale_pos_y, PowerTrendMaximum_01W / 10 / 2);
  out += temp;
  scale_pos_y += PowerTrendHeight / 4; 
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d W</text>\n",scale_pos_x, scale_pos_y, PowerTrendMaximum_01W / 10 / 4);
  out += temp;
  scale_pos_y = PowerTrendHeight - 5;
  scale_pos_x = 10;
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">0</text>\n",scale_pos_x, scale_pos_y);
  out += temp;

  scale_pos_x = 23;
  scale_pos_x += PowerTrendSize / 2 - 10;
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">6h</text>\n",scale_pos_x, scale_pos_y);
  out += temp;

  scale_pos_x += PowerTrendSize / 2;
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">12h</text>\n",scale_pos_x, scale_pos_y);
  out += temp;

  scale_pos_x += PowerTrendSize / 2;
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">18h</text>\n",scale_pos_x, scale_pos_y);
  out += temp;

  scale_pos_x = PowerTrendSize * 2 - 20;
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">24h</text>\n",scale_pos_x, scale_pos_y);
  out += temp;

  scale_pos_x = PowerTrendSize * 2 - 23;  
  scale_pos_y = 15;
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d kWh</text>\n",scale_pos_x, scale_pos_y, ETodayTrendMaximum_01kWh / 10);
  out += temp;
  scale_pos_y += PowerTrendHeight / 4; 
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d kWh</text>\n",scale_pos_x, scale_pos_y, ETodayTrendMaximum_01kWh / 10 / 4 * 3);
  out += temp;
  scale_pos_y += PowerTrendHeight / 4; 
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d kWh</text>\n",scale_pos_x, scale_pos_y, ETodayTrendMaximum_01kWh / 10 / 2);
  out += temp;
  scale_pos_y += PowerTrendHeight / 4; 
  sprintf(temp, "<text x=\"%d\" y=\"%d\" class=\"scaletext\">%d kWh</text>\n",scale_pos_x, scale_pos_y, ETodayTrendMaximum_01kWh / 10 / 4);
  out += temp;

  out += "</g>\n</svg>\n";

  server.send ( 200, "image/svg+xml", out);
}
