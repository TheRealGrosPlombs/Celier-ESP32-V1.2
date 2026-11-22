//**************************************
// USE ESP CORE 1.06 TO COMPILE
// Board NodeMCU-32S
//
//**************************************
#include "driver/ledc.h"
#include <SPI.h>
#include <TFT_eSPI.h>     // TFT LCD Display library
#include "DHT.h"          // temperature and humidity sensor library (adafruit)
#include <WiFi.h>
#include "Time.h"         // Time library
#include <NTPClient.h>    // web time update
#include <WiFiUdp.h>      // used by web time update
#include <JPEGDecoder.h>  // JPEG decoder library
//#include "LittleFS.h"   // Use this library for ESP8266
#include "FS.h"
#include "LITTLEFS.h"     // Use this library for ESP32S
#include <TimerEvent.h>
#include <SerialCommand.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "ESPAsyncWebServer.h"

// DEFINES //
#define LittleFS LITTLEFS
#define WIFI_NEW_HOSTNAME "celier"

// **********************************
// Screen defaults
// **********************************
#define TFT_DEFAULT_ROTATION  1
#define WIFI_TIMEOUT_MS       20000   // WIFI connection timeout in ms
#define TFT_TRANSPARENT_COLOR TFT_WHITE

#define TEMP_DEFAULT_SETPOINT 15
#define TEMP_TFT_TEXT_POS_X   150
#define TEMP_TFT_TEXT_POS_Y   120
#define TEMP_TFT_TEXT_WIDTH   195
#define TEMP_TFT_TEXT_HEIGHT  93
#define TEMP_NAN_TIMEOUT      5

#define TIME_TFT_TEXT_POS_X   210
#define TIME_TFT_TEXT_POS_Y   210
#define TIME_TFT_TEXT_WIDTH   100
#define TIME_TFT_TEXT_HEIGHT  40

#define HUMID_TFT_TEXT_POS_X  10
#define HUMID_TFT_TEXT_POS_Y  210
#define HUMID_TFT_TEXT_WIDTH  75
#define HUMID_TFT_TEXT_HEIGHT 40

#define TFT_TOUCH_THRESHOLD   1000    // TFT touch Z value where a touch is detected, less than this value is no touch

#define WIFI_ICON_POS_X       280
#define WIFI_ICON_POS_Y       10
#define WIFI_ICON_WIDTH       32
#define WIFI_ICON_HEIGHT      32

#define SNOW_ICON_POS_X       10
#define SNOW_ICON_POS_Y       10
#define SNOW_ICON_WIDTH       32
#define SNOW_ICON_HEIGHT      32

#define WATER_ICON_POS_X      90
#define WATER_ICON_POS_Y      50
#define WATER_ICON_WIDTH      140
#define WATER_ICON_HEIGHT     140

// **********************************
// timeouts for different events
// **********************************
#define TEMP_UPDATE_INTERVAL  5000
#define TIME_UPDATE_INTERVAL  20000
#define TFT_SCREENSAVER_TIME  10000

#define WATER_DETECTED_VALUE  2000  // ADC value where the probes are in water (this value or lower means water)
#define WATER_DETECTED_HYST   500   // Hysteris of ADC value between detected and not detected values to avoid oscillations near threshold

// **********************************
// hardware pins used
// **********************************
#define WATER_LEVEL_PIN       35      // 3.3V<--->220K<--->ADC<--->1M<--->GND
                                      // Put GND and ADC in water to change the 
                                      // voltage reading.

#define DHTTYPE               DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN                16      // Digital pin connected to the DHT sensor

#define BL_PWM_NUM            0
#define BL_PWM_FREQ           1000
#define BL_PWM_PIN            33      // Taken from schematic of custom board
#define BL_PWM_DEFAULT_VAL    255     
#define BL_PWM_LOW_VAL        50      // Lowest value for fan to turn on
#define BL_PWM_HI_VAL         255     // Highest value in our 8 bit case

#define TEC1_PIN              25      // Where the TEC element is connected to 
#define TEC_OFF               0       
#define TEC_ON                1

#define FANS_PWM_NUM          1
#define FANS_PWM_RESOLUTION   8
#define FANS_PWM_FREQ         20000//=Artic Fan //By trial and error, this value is best for my fan
#define FANS_PWM_DEFAULT_VAL  20  //14
#define FANS_PWM_LO_VAL       10
#define FANS_PWM_HI_VAL       255
#define FANS_PIN              27
#define FANS_OFF              0
#define FANS_ON               1

// Cooling fan parameters
#define FAN_C_PWM_NUM         2
#define FAN_C_PWM_RESOLUTION  8
#define FAN_C_PWM_FREQ        20000   // By trial and error, this value is best for my fan
#define FAN_C_PWM_DEFAULT_VAL 255 //220
#define FAN_C_PWM_LO_VAL      100
#define FAN_C_PWM_HI_VAL      220
#define FAN_C_PIN             26
#define FAN_C_OFF             0
#define FAN_C_ON              1

#define LEDS_PWM_NUM          LEDC_CHANNEL_3
#define LEDS_PWM_FREQ         1000
#define LEDS_PWM_DEFAULT_VAL  0       // Off on startup
#define LEDS_PWM_LOW_VAL      0
#define LEDS_PWM_HI_VAL       255
#define LEDS_PIN              17 
#define LEDS_OFF              0
#define LEDS_ON               1
#define LEDS_FADE_TIME_MS     2000

#define DOOR_PIN              39
#define DOOR_OPEN             1// 0
#define DOOR_CLOSED           0// 1

// Outside push fan for extra convection
#define FAN_BLOWER_PWM_NUM          LEDC_CHANNEL_4
#define FAN_BLOWER_PWM_FREQ         20000
#define FAN_BLOWER_PIN              14
#define FAN_BLOWER_PWM_DEFAULT_VAL  255
#define FAN_BLOWER_DEFAULT_VAL      255
#define FAN_BLOWER_LO_VAL           0
#define FAN_BLOWEER_HI_VAL          255

//#define LEDS_ON_MACRO  {ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE,LEDS_PWM_NUM,LEDS_PWM_HI_VAL,LEDS_FADE_TIME_MS);ledc_fade_start(LEDC_HIGH_SPEED_MODE,LEDS_PWM_NUM,LEDC_FADE_NO_WAIT);}
//#define LEDS_OFF_MACRO  {ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE,LEDS_PWM_NUM,LEDS_PWM_LOW_VAL,LEDS_FADE_TIME_MS);ledc_fade_start(LEDC_HIGH_SPEED_MODE,LEDS_PWM_NUM,LEDC_FADE_NO_WAIT);}

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Invoke TFT library
TFT_eSPI tft = TFT_eSPI();
//TFT_eSprite spr = TFT_eSprite(&tft); // Invoke Sprite class

// define FONTs to use
#define TIME_FONT "Arial36"
#define TEMP_FONT "Arial36"
const char *timeFontFile = "/Arial36";
const char *tempFontFile = "/Arial36";

// define images to use
const char *backgroundFile = "/background.bmp";
const char *wifiIconFile = "/wifi.bmp";
const char *snowIconFile = "/snowflake-icon.bmp";
uint16_t snowTFTBuffer[SNOW_ICON_WIDTH * SNOW_ICON_HEIGHT];
const char *waterDropFile = "/water-droplets-icon.bmp";

// Initialize WiFi and web time update
const char *ssidFile = "/SSID.txt";
const char *passwordFile = "/password.txt";
const long utcOffsetInSeconds = -5 * 60 * 60; // UTC -5 hours
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

const char *setPointFile = "/setpoint.txt";
const char *logFileName = "/log.txt";
const char *debugFileName = "/debug.txt";

// Global variables
uint16_t* ptrTimeTFTBuf;  // To save background under time area
uint16_t timeTFTBuffer[TIME_TFT_TEXT_WIDTH * TIME_TFT_TEXT_HEIGHT]; // TFT buffer to restore background before printing new time
char currentTime[10];     // string holder for current time
int32_t timePosX = TIME_TFT_TEXT_POS_X;       // TFT position for current time
int32_t timePosY = TIME_TFT_TEXT_POS_Y;       // TFT position for current time
int32_t timeWidth;            // determine width and height of time text depending on font used
int32_t timeHeight;
int oldHours,oldMinutes;  // Keep old time value and only update when it has changed
TimerEvent timerTime;

float fCelierTemperature; // temperature inside the Celier read by DHT sensor
uint16_t* tempBuffer;     // to save background under temperature text area
uint16_t tempTFTBuffer[TEMP_TFT_TEXT_WIDTH * TEMP_TFT_TEXT_HEIGHT];
char currentTemp[6];      // string holder for current temperature
int32_t tempWidth = TEMP_TFT_TEXT_POS_X;      // width in pixels of temp text area
int32_t tempHeight = TEMP_TFT_TEXT_POS_Y;     // height in pixels of temp text area
int32_t tempPosX = 70;        // position of temperature on TFT
int32_t tempPosY = 40;
int temperature_value;
int oldval;               // previous temperature value to update only when changed
float TempSetpoint = TEMP_DEFAULT_SETPOINT;
uint8_t consecutiveNANs = 0;    // count number of unavaible temp readings in a row

TimerEvent timerTemp;

float fCelierHumidity;    // humidity inside the Celier read by DHT sensor
uint16_t CelierHumidity;
uint16_t OldCelierHumidity;
uint16_t humidTFTBuffer[HUMID_TFT_TEXT_WIDTH * HUMID_TFT_TEXT_HEIGHT];
int32_t humidPosX = HUMID_TFT_TEXT_POS_X;
int32_t humidPosY = HUMID_TFT_TEXT_POS_Y;

TimerEvent timerScreensaver;

TimerEvent timerBacklight;
const uint16_t backlightDimmingSpeed = 3; // change backlight value every x milliseconds
uint16_t backlightSetpoint = BL_PWM_LOW_VAL;
uint16_t backlightCurrentVal = BL_PWM_HI_VAL;

uint16_t LEDlightSetpoint = LEDS_PWM_LOW_VAL;
uint16_t LEDlightCurrentVal = LEDS_PWM_LOW_VAL;

bool touchDetected = false;
uint8_t HotSideFanSpeed = 0;
uint8_t CoolSideFanSpeed = 0;
uint32_t HotSideFanFrequency = FANS_PWM_FREQ;
uint32_t CoolSideFanFrequency = FAN_C_PWM_FREQ;
uint8_t BlowerFanSpeed = 0;
uint32_t BlowerFanFrequency = FAN_BLOWER_PWM_FREQ;

bool previousOnOffValue = false;

bool doorOpen;

struct tm timeinfo;

int waterLevelADC = 4095;
int waterLevelHyst = WATER_DETECTED_HYST;
bool waterLevelHigh = false;
uint32_t secondsCounter = 0;

DHT dht(DHTPIN, DHTTYPE);
SerialCommand sCmd;     // The SerialCommand object

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  drawBmp(backgroundFile, 0, 0);  // redraw background to erase time and wifi logo
  //Serial.println("Trying to Reconnect");
  //WiFi.begin(ssid, password);
}

void setup() {

  Serial.begin(115200);
  Serial.flush();
  Serial.println("Celier V1.0");
  sCmd.addCommand("S",    LED_on);          // Turns LED on
  sCmd.addCommand("F",   LED_off);         // Turns LED off
  sCmd.addCommand("T",updateTempSetPoint);
  sCmd.addCommand("t",updateTempSetPoint);
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")

  pinMode(DOOR_PIN,INPUT);
  pinMode(TEC1_PIN,OUTPUT);
  digitalWrite(TEC1_PIN,TEC_OFF);  // Start with element off
  pinMode(WATER_LEVEL_PIN, INPUT);
  
  //pinMode(LEDS_PIN,OUTPUT);
  //digitalWrite(LEDS_PIN,LEDS_OFF);  // Start with light off
  //pinMode(FANS_PIN,OUTPUT);
  //digitalWrite(FANS_PIN,FANS_OFF);  // Start with light off

  HotSideFanFrequency = FANS_PWM_FREQ;
  CoolSideFanFrequency = FAN_C_PWM_FREQ;
  // Set up FANS with PWM output to control fan noise
  ledcSetup(FANS_PWM_NUM,HotSideFanFrequency,FANS_PWM_RESOLUTION);
  ledcAttachPin(FANS_PIN,FANS_PWM_NUM);
  ledcWrite(FANS_PWM_NUM, HotSideFanSpeed);

  
  // Set up FAN_C (Cool side fan) with PWM output to control fan noise
  ledcSetup(FAN_C_PWM_NUM,CoolSideFanFrequency,FAN_C_PWM_RESOLUTION);
  ledcAttachPin(FAN_C_PIN,FAN_C_PWM_NUM);
  ledcWrite(FAN_C_PWM_NUM, CoolSideFanSpeed);
  
  
  // Setup PWM on ESP32 for TFT backlight control
  ledcSetup(BL_PWM_NUM,BL_PWM_FREQ,8);
  ledcAttachPin(BL_PWM_PIN,BL_PWM_NUM);
  ledcWrite(BL_PWM_NUM, BL_PWM_DEFAULT_VAL);

  ledcSetup(LEDS_PWM_NUM,LEDS_PWM_FREQ,8);
  ledcAttachPin(LEDS_PIN,LEDS_PWM_NUM);
  ledcWrite(LEDS_PWM_NUM, LEDS_PWM_DEFAULT_VAL);

  ledcSetup(FAN_BLOWER_PWM_NUM,FAN_BLOWER_PWM_FREQ,8);
  ledcAttachPin(FAN_BLOWER_PIN,FAN_BLOWER_PWM_NUM);
  ledcWrite(FAN_BLOWER_PWM_NUM, FAN_BLOWER_PWM_DEFAULT_VAL);


  //***************************************************************
  // Init file system
  //***************************************************************
  Serial.println("Mounting file system...");
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }
  Serial.println("file system mounted!");
  
  char setPointString[5];
  if (readTextFile(setPointFile, setPointString))
  {
    String tempString = setPointString;
    Serial.print("Set point file conatains: ");
    Serial.print(setPointString);
    TempSetpoint = tempString.toFloat();
  }
  else
  {
    TempSetpoint = TEMP_DEFAULT_SETPOINT;
  }
  Serial.print("Temperature setpoint is now ");
  Serial.print(TempSetpoint);
  Serial.println(" ° Celsius.");

  // Initialise temperature sensor
  dht.begin();
  
  //***************************************************************
  // Init TFT and touch screen
  //***************************************************************
  tft.begin();
  tft.setRotation(TFT_DEFAULT_ROTATION);  // landscape
  drawBmp(backgroundFile, 0, 0);

  // Get snowflake icon background to erase it when needed...
  tft.readRect(SNOW_ICON_POS_X,SNOW_ICON_POS_Y,SNOW_ICON_WIDTH,SNOW_ICON_HEIGHT,snowTFTBuffer);

/*  
  //***************************************************************
  // Init Wifi by reading SSID.txt and password.txt files
  //***************************************************************
  char wifiSSID[33];  // SSID name holder 32 bytes MAX for SSID names
  char pw[64];        // password name holder
  long TimeoutCounter = millis();
  if ((readTextFile(ssidFile,wifiSSID)) & (readTextFile(passwordFile,pw)))
  {
    WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
    WiFi.begin(wifiSSID, pw); // Connect to WiFi with SSID and pw from filesystem
    while (WiFi.status() != WL_CONNECTED) // Check if connection is established
    {
      delay(500); // Set delay to 0.5s
      Serial.print("."); // Print dots on serial monitor until connection is established
      if ((millis() - TimeoutCounter) > WIFI_TIMEOUT_MS)
      {
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      ArduinoOTA
        .onStart([]() {
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
        });
    
      ArduinoOTA.begin();
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
    else
    {
      Serial.println("WIFI Timeout reached. No WIFI Support");
    }
  }
  else
  {
    Serial.println("No WIFI support!");
  }
*/
  connectToWifi();
  // Get time update from web if wifi is connected
  if (WiFi.status() == WL_CONNECTED)
  {
    configTime(0, 0, "pool.ntp.org");    // First connect to NTP server, with 0 TZ offset
    setenv("TZ","EST5EDT,M3.2.0,M11.1.0",1);
    tzset();
    getLocalTime(&timeinfo);
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
    
    Serial.print("Connected to WiFi.");
    drawBmp(wifiIconFile, WIFI_ICON_POS_X, SNOW_ICON_POS_Y);
    timeClient.begin();
    timeClient.update();
    sprintf(currentTime,"%02u:%02u",timeinfo.tm_hour,timeinfo.tm_min);
    //sprintf(currentTime,"%02u:%02u",timeClient.getHours(),timeClient.getMinutes());
    oldHours = timeinfo.tm_hour; //timeClient.getHours();
    oldMinutes = timeinfo.tm_min; //timeClient.getMinutes();
    // Setup font, color, position then save background and print time on TFT
    tft.setTextColor(TFT_DARKGREY);
    tft.loadFont(TIME_FONT, LittleFS);  
    timeWidth = tft.textWidth(currentTime);
    timeHeight = tft.fontHeight();
    Serial.println(timeWidth);
    Serial.println(timeHeight);
    tft.readRect(timePosX,timePosY,TIME_TFT_TEXT_WIDTH,TIME_TFT_TEXT_HEIGHT,timeTFTBuffer);
    tft.setCursor(timePosX,timePosY);
    tft.print(currentTime);
    tft.unloadFont();

    // Setup web server 
    server.serveStatic("/", LittleFS, "/");//.setDefaultFile("index.html");
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(LittleFS, "/index.html", String(), false, processor);
    });
    server.begin();
    // Print ESP32 Local IP Address
    Serial.print("Web server at IP ");
    Serial.println(WiFi.localIP());
  }  

/*
  tft.loadFont(TEMP_FONT, LittleFS);
  sprintf(currentTemp,"%2d°C",0);
  tempWidth = tft.textWidth(currentTemp);
  tempHeight = tft.fontHeight();
  Serial.print("Temp text width = ");
  Serial.print(tempWidth);
  Serial.print(" , height = ");
  Serial.print(tempHeight);
  //tempBuffer = (uint16_t*)malloc((tempWidth+5) * (tempHeight+5) * 2);
*/
  // Get background image behind time area to use when overwriting with a new time
  ptrTimeTFTBuf = tempTFTBuffer;
  tft.readRect(tempPosX, tempPosY, TEMP_TFT_TEXT_WIDTH, TEMP_TFT_TEXT_HEIGHT, tempTFTBuffer);
  tft.readRect(humidPosX,humidPosY,HUMID_TFT_TEXT_WIDTH,HUMID_TFT_TEXT_HEIGHT, humidTFTBuffer);
  
  timerTemp.set(TEMP_UPDATE_INTERVAL,updateTempCallback);
  timerTime.set(TIME_UPDATE_INTERVAL,updateTimeCallback);
  timerScreensaver.set(TFT_SCREENSAVER_TIME,updateScreenSaver);
  timerBacklight.set(backlightDimmingSpeed,updateBacklightLevel);
}

// Update the backlight level until the setpoint is reached
void updateBacklightLevel(void)
{
  // update TFT backlight here
  if (backlightSetpoint < backlightCurrentVal)
  {
    if (backlightCurrentVal > 0)
      backlightCurrentVal--;
    ledcWrite(BL_PWM_NUM, backlightCurrentVal);
  }
  if (backlightSetpoint > backlightCurrentVal)
  {
    if (backlightCurrentVal < 255)
      backlightCurrentVal++;
    ledcWrite(BL_PWM_NUM, backlightCurrentVal);
  }
  // Update LEDs value here
  if (LEDlightSetpoint < LEDlightCurrentVal)
  {
    if (LEDlightCurrentVal > 0)
      LEDlightCurrentVal--;
    ledcWrite(LEDS_PWM_NUM, LEDlightCurrentVal);
  }
  if (LEDlightSetpoint > LEDlightCurrentVal)
  {
    if (LEDlightCurrentVal < 255)
      LEDlightCurrentVal++;
    ledcWrite(LEDS_PWM_NUM, LEDlightCurrentVal);
  }
}

// Update the screensaver 
void updateScreenSaver(void)
{
  //ledcWrite(BL_PWM_NUM, BL_PWM_LOW_VAL);
  backlightSetpoint = BL_PWM_LOW_VAL;
  timerScreensaver.disable();
}

// Update the temperature in this callback function
void updateTempCallback(void)
{

  if (waterLevelHigh) return; // Don't update temp if water icon is showing

  // Initialise temperature sensor
  dht.begin();

  fCelierTemperature = dht.readTemperature(); // Current temp in Celsius
  if (isnan(fCelierTemperature)) 
  {
    consecutiveNANs++;
    if (consecutiveNANs > TEMP_NAN_TIMEOUT)
    {
      // if temp value is no good, don't change display
      Serial.println("temperature returned Not A Number!");
      //tft.pushRect(tempPosX, tempPosY, TEMP_TFT_TEXT_WIDTH, TEMP_TFT_TEXT_HEIGHT, tempTFTBuffer);
      TEC_Control(TEC_OFF);     // Turn TEC off if temp sensor is malfunctionning
    }
  }
  else
  {
    consecutiveNANs = 0;    // reset defective readings counter
    Serial.print("Current temperature is ");
    Serial.print(fCelierTemperature);
    Serial.println(" °C");
    // Basic temp control
    if (fCelierTemperature > TempSetpoint)
    {
      TEC_Control(TEC_ON);
    }
    else
    {
      TEC_Control(TEC_OFF);
    }

    temperature_value = (uint16_t) (fCelierTemperature + 0.5); // Add 0.5 to round up the integer
    if (oldval != temperature_value) 
    {
      oldval = temperature_value;
      tft.pushRect(tempPosX, tempPosY, TEMP_TFT_TEXT_WIDTH, TEMP_TFT_TEXT_HEIGHT, tempTFTBuffer);
      displayTemp(temperature_value,tempPosX,tempPosY);
    }
  }
  
  fCelierHumidity = dht.readHumidity(); // Current humidity
  if (isnan(fCelierHumidity)) 
  {
    // if temp value is no good, display nothing...
    Serial.println("Humidity returned Not A Number!");
    //tft.pushRect(humidPosX,humidPosY,HUMID_TFT_TEXT_WIDTH,HUMID_TFT_TEXT_HEIGHT, humidTFTBuffer);
  }
  else
  {
    CelierHumidity = ((uint16_t)fCelierHumidity);
    if (CelierHumidity != OldCelierHumidity)
    {
      OldCelierHumidity = CelierHumidity;
      Serial.print("Current Humidity is ");
      Serial.print(CelierHumidity);
      tft.pushRect(humidPosX,humidPosY,HUMID_TFT_TEXT_WIDTH,HUMID_TFT_TEXT_HEIGHT, humidTFTBuffer);
      tft.setTextColor(TFT_DARKGREY);
      tft.loadFont(TIME_FONT, LittleFS);  
      tft.setCursor(humidPosX,humidPosY);
      tft.print(CelierHumidity);
      tft.print("%");
      tft.unloadFont();
    }
  }
}

// Update the time displayed in this callback function
void updateTimeCallback(void)
{
  
  // if no wifi, exit without updating time
  if (WiFi.status() != WL_CONNECTED) return;
  
  //timeClient.update();
  getLocalTime(&timeinfo);
  //if ((oldMinutes != timeClient.getMinutes()) | (oldHours != timeClient.getHours()))
  if ((oldMinutes != timeinfo.tm_min) | (oldHours != timeinfo.tm_hour))
  {
    oldHours = timeinfo.tm_hour;
    oldMinutes = timeinfo.tm_min;
    tft.pushRect(timePosX,timePosY,TIME_TFT_TEXT_WIDTH,TIME_TFT_TEXT_HEIGHT,timeTFTBuffer);
    sprintf(currentTime,"%02u:%02u",timeinfo.tm_hour,timeinfo.tm_min);
    tft.setTextColor(TFT_DARKGREY);
    tft.loadFont(TIME_FONT, LittleFS);  
    timeWidth = tft.textWidth(currentTime);
    timeHeight = tft.fontHeight();
    tft.setCursor(timePosX,timePosY);
    tft.print(currentTime);
    tft.unloadFont();
    Serial.println(currentTime);
    // redraw wifi connected logo in case it got erased...
    drawBmp(wifiIconFile, WIFI_ICON_POS_X, SNOW_ICON_POS_Y);
  }
}

void loop() {

  ArduinoOTA.handle();    // Used to update the firmware via WIFI

  sCmd.readSerial();     // We don't do much, just process serial commands

  // Check for open door.
  doorOpen = digitalRead(DOOR_PIN);
  if (doorOpen == DOOR_OPEN)
  {
    backlightSetpoint = BL_PWM_HI_VAL;
    timerScreensaver.reset();
    timerScreensaver.enable();
    LEDlightSetpoint = LEDS_PWM_HI_VAL;
  }
  else
  {
    LEDlightSetpoint = LEDS_PWM_LOW_VAL;
  }
  // if any touch is detected, handle it
  if (tft.getTouchRawZ() > TFT_TOUCH_THRESHOLD)
  {
    touchDetected = true;
    backlightSetpoint = BL_PWM_HI_VAL;
    timerScreensaver.reset();
    timerScreensaver.enable();
  }
  else
  {
  }

  if ((millis() - secondsCounter) > 1000)
  {
    secondsCounter = millis();
    // Check water level in dip tray. This check has TOP PRIORITY and overrides the other checks (door, temp, etc)
    waterLevelADC = analogRead(WATER_LEVEL_PIN);
    Serial.print("ADC = ");
    Serial.println(waterLevelADC);
    
    if (waterLevelADC < (WATER_DETECTED_VALUE + WATER_DETECTED_HYST))
    {
      if (waterLevelHigh == false)
      {
        waterLevelHigh = true;
        waterLevelHyst = -WATER_DETECTED_HYST;
        // Water detected! Show water icon and stop cooling TEC but not fans
        Serial.println("*** WATER LEVEL EXCEEDED!!! ***");
        backlightSetpoint = BL_PWM_HI_VAL;
        timerScreensaver.reset();
        LEDlightSetpoint = LEDS_PWM_HI_VAL;
        drawBmp(backgroundFile, 0, 0);
        drawBmp(waterDropFile, WATER_ICON_POS_X, WATER_ICON_POS_Y);
      }
    }
    else
    {
      if (waterLevelHigh == true)
      {
        waterLevelHigh = false;
        waterLevelHyst = WATER_DETECTED_HYST;
        // No water detected!
        Serial.println("Water level OK...");
        timerScreensaver.enable();
        drawBmp(backgroundFile, 0, 0);
        oldval = 0; // This forces the temperature to be updated now
      }
    }
  }
    
  // update timers here
  timerTemp.update();
  timerTime.update();
  timerScreensaver.update();
  timerBacklight.update();

}

// *****************************************
// Read file contents and return it in *ptr
// *****************************************
bool readTextFile(const char * path, char *ptr)
{
  char *p;
  p = ptr;
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) 
  { 
    //Serial.println("Failed to open SSID file!"); 
    return false;
  }
  int i = 0;
  while (f.available()) 
  {
    *p = f.read();
    i++;
    p++;
  }
  *p = 0;  // add string terminator
  f.close();
  return true;
}


// *****************************************
// Control Heating and cooling here
// *****************************************
void TEC_Control(bool OnOff)
{
  char message[128];

  // If water level is high, turn off cooling
  if (waterLevelHigh) OnOff = false;
  
  // Keep TEC off while door is open.
  if (doorOpen == DOOR_OPEN) OnOff = false;
  
  digitalWrite(TEC1_PIN,OnOff);

  // patch to make sure PWM frequency does not change
  if (ledcReadFreq(FANS_PWM_NUM) != HotSideFanFrequency)
    ledcSetup(FANS_PWM_NUM,HotSideFanFrequency,FANS_PWM_RESOLUTION);
  if (ledcReadFreq(FAN_C_PWM_NUM) != CoolSideFanFrequency)
    ledcSetup(FAN_C_PWM_NUM,CoolSideFanFrequency,FANS_PWM_RESOLUTION);
  if (OnOff) 
  {
    HotSideFanSpeed = FANS_PWM_DEFAULT_VAL;
    CoolSideFanSpeed = FAN_C_PWM_DEFAULT_VAL;
    BlowerFanSpeed = FAN_BLOWER_DEFAULT_VAL;
    // Display snowflake icon to show that we are actively cooling
    drawBmp(snowIconFile, SNOW_ICON_POS_X, SNOW_ICON_POS_Y);
  }
  else
  {
    HotSideFanSpeed = FANS_PWM_LO_VAL;
    CoolSideFanSpeed = FAN_C_PWM_LO_VAL;
    BlowerFanSpeed = FAN_BLOWER_LO_VAL;
    // remove snowflake icon
    tft.pushRect(SNOW_ICON_POS_X, SNOW_ICON_POS_Y, SNOW_ICON_WIDTH, SNOW_ICON_HEIGHT, snowTFTBuffer);
  }
  ledcWrite(FANS_PWM_NUM, HotSideFanSpeed);
  ledcWrite(FAN_C_PWM_NUM, CoolSideFanSpeed);
  ledcWrite(FAN_BLOWER_PWM_NUM, BlowerFanSpeed);
}

// *****************************************
// Display temp as 2 bitmaps
// *****************************************
void displayTemp(int8_t temp,uint16_t x,uint16_t y)
{
  if (temp < 0) return;
  if (temp > 30) return;
  uint8_t first_digit = temp / 10;
  uint8_t second_digit = 0;
  uint16_t second_digit_offset = 0;
  if (first_digit > 0)
  {
    second_digit_offset = drawBMPDigit(first_digit, x,y);
    second_digit = temp - (first_digit * 10);
  }
  else
  {
    second_digit = temp;
  }
  uint16_t degree_offset = drawBMPDigit(second_digit, x + second_digit_offset + 2,y);
  tft.setTextColor(TFT_DARKGREY);
  tft.loadFont(TEMP_FONT, LittleFS);  
  tft.setCursor(x + second_digit_offset + degree_offset,y);
  tft.print("°C");
  tft.unloadFont();
}

void LED_on() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    HotSideFanSpeed = atoi(arg);
    ledcWrite(FANS_PWM_NUM, HotSideFanSpeed);
    Serial.print("New fan period is ");
    Serial.println(HotSideFanSpeed);
  }
}

void LED_off() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    HotSideFanFrequency = atoi(arg);
    ledcSetup(FANS_PWM_NUM,HotSideFanFrequency,8);
    ledcWrite(FANS_PWM_NUM, HotSideFanSpeed);
    Serial.print("New fan frequency is ");
    Serial.println(HotSideFanFrequency);
  }
}

void updateTempSetPoint()
{
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    String tmp = arg;
    float tmpSetPoint = tmp.toFloat();
    if (isnan(tmpSetPoint))
    {
      Serial.print("Invalid number!");
    }
    else
    {
      // Open file for writing
      File file = LittleFS.open(setPointFile, "w");
      if (file)
      {
        file.print(tmp);
      }
      TempSetpoint = tmpSetPoint;
      Serial.print("New temperature setpoint is ");
      Serial.print(TempSetpoint);
      Serial.println("°C.");
      file.close();
    }
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What?");
}

// Replaces placeholder with LED state value in web page
String processor(const String& var){
  String RetVal;
  
  Serial.println(var);
  if(var == "TEMPERATURE"){
    RetVal = String(fCelierTemperature);
    Serial.print(RetVal);
    return RetVal;
  }
  if(var == "SETPOINT"){
    RetVal = String(TempSetpoint);
    Serial.print(RetVal);
    return RetVal;
  }
  return String();
}

void connectToWifi(void)
{
  //***************************************************************
  // Init Wifi by reading SSID.txt and password.txt files
  //***************************************************************
  char wifiSSID[33];  // SSID name holder 32 bytes MAX for SSID names
  char pw[64];        // password name holder
  long TimeoutCounter = millis();
  if ((readTextFile(ssidFile,wifiSSID)) & (readTextFile(passwordFile,pw)))
  {
    WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
    WiFi.hostname(WIFI_NEW_HOSTNAME);
    WiFi.begin(wifiSSID, pw); // Connect to WiFi with SSID and pw from filesystem
    while (WiFi.status() != WL_CONNECTED) // Check if connection is established
    {
      delay(500); // Set delay to 0.5s
      Serial.print("."); // Print dots on serial monitor until connection is established
      if ((millis() - TimeoutCounter) > WIFI_TIMEOUT_MS)
      {
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      ArduinoOTA
        .onStart([]() {
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
        });
    
      ArduinoOTA.begin();
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
    else
    {
      Serial.println("WIFI Timeout reached. No WIFI Support");
    }
  }
  else
  {
    Serial.println("No WIFI support!");
  }
}
