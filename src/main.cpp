#include <Arduino.h>
#include <FS.h>
#include <Ticker.h>
#include <OneButton.h>
#include <TimeLib.h>
#include <NtpClientLib.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Statistic.h>
#include <WS2812FX.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#include <AsyncMqttClient.h>
#include <ArduinoOTA.h>

// #define DEBUG
// #define TEST

#define APP_FW_VER "1.0.0-RC.1"

//***** Button and Relay Pin Definitions *****//
#define BUTTON_PIN		4
#define HEATER_PIN		5
#define LED_PIN       13
#define ONE_WIRE_BUS  14  // DS18B20 pin

//***** Time *****//
#define AUTO_SHUTOFF_TIME_DEFAULT 30
uint8_t shutoffTime = AUTO_SHUTOFF_TIME_DEFAULT;
Ticker shutoffTimer;
void autoShutoff(void);

bool NTPTimeSynced;
bool NTPServerOnline;

Ticker healthTimer;
void getUptimeAndRSSI(void);

//***** Neopixel *****//
#define LED_COUNT 1
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);
uint8_t ledState = 0;
Ticker ledTimer;
Ticker ledStatusControllerTimer;
void updateLed(void);
void ledStatusController(void);

//***** MQTT Definitions *****//
char mqtt_server[40];
char mqtt_port[8];
char mqtt_user[33];
char mqtt_token[33];
bool wifiManagerSaveSettings = false;
bool wifiReady = false;
AsyncMqttClient mqttClient;

String homieVersion = "2.0.0";
String mac = WiFi.macAddress();
String chipID = String(ESP.getChipId());
String localIP;
//String mqttHeader = "home/"+mac;
String mqttHeader = "home/"+chipID;

String mqttHomieVersion = mqttHeader + "/$homie";
String mqttDeviceOnline = mqttHeader + "/$online";
String mqttDeviceName = mqttHeader + "/$name";
String mqttDeviceNameSet = mqttHeader + "/$name/set";
String mqttDeviceIP = mqttHeader + "/$localip";
String mqttDeviceMac = mqttHeader + "/$mac";
String mqttDeviceUptime = mqttHeader + "/$stats/uptime";
String mqttDeviceUptimeInterval = mqttHeader + "/$stats/interval";
String mqttDeviceRSSI = mqttHeader + "/$stats/signal";
String mqttDeviceFWVersion = mqttHeader + "/$fw/name";
String mqttDeviceFWBuild = mqttHeader + "/$fw/version";
String mqttDeviceOTA = mqttHeader + "/$ota";
String mqttDeviceImplementation = mqttHeader + "/$implementation";
String mqttDeviceNodes = mqttHeader + "/$implementation/nodes";
String mqttNodes = "switch,temperature";

String mqttSwitchType = mqttHeader + "/switch/$type";
String mqttSwitchProperties = mqttHeader + "/switch/$properties";
String mqttSwitchState = mqttHeader + "/switch/on";
String mqttSwitchStateSet = mqttHeader+"/switch/on/set";
String mqttSwitchStateTime = mqttHeader+"/switch/time";
String switchProperties = "on:settable[true,false],time";

String mqttTemperatureType = mqttHeader+"/temperature/$type";
String mqttTemperatureProperties = mqttHeader+"/temperature/$properties";
String mqttTemperature = mqttHeader+"/temperature/temperature";
String mqttTemperatureSet = mqttHeader+"/temperature/temperature/set";
String mqttTemperatureUnits = mqttHeader+"/temperature/units";
String mqttTemperatureTime = mqttHeader+"/temperature/time";
String mqttTemperatureAlive = mqttHeader+"/temperature/alive";
String temperatureProperties = "temperature:settable[50-100],units,time,alive";

String mqttCoffeeStatusType = mqttHeader + "/coffee/$type";
String mqttCoffeeStatusProperties = mqttHeader + "/coffee/$properties";
String mqttCoffeeStatusState = mqttHeader + "/coffee/ready";
String mqttCoffeeStatusTime = mqttHeader+"/coffee/time";
String coffeeProperties = "ready,time";

//***** Sensor Variables and Prototypes *****//
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

Ticker temperatureTimer;
Ticker heaterControlTimer;

void getTemperature(void);
void pubTemperature(void);
bool saveTemperatureSettingToFS(void);
bool loadTemperatureSettingFromFS(void);

#define DEFAULT_TEMP_SET	96
// arrays to hold device address
DeviceAddress tempSensor;

bool temperatureIsOk;
float temperature;
char set_temp[8];
uint8_t temperatureSet = DEFAULT_TEMP_SET;
Statistic temperatureTracker;
bool heatingComplete = false;

//***** Relay Variables and Prototypes *****//
bool heaterOn = false;
void pubheaterOn(void);
void heaterController(void);

//***** Button Variables and Prototypes *****//
Ticker tick;
uint32_t buttonHoldTimer;
void tock();
OneButton button(BUTTON_PIN, true);

//***** WiFi Manager *****//
String AP_BASE = "Bunny";
String AP_ID = "01";
String AP_NAME = AP_BASE;

void wifiManagerOpenPortal(void);
bool wifiManagerLoadConfig(void);
bool wifiManagerSaveConfig(void);

//***** PID Variables and Constants *****//
volatile long onTime = 0;

double pidIn;
double pidOut;
double pidSet = DEFAULT_TEMP_SET;

// pid tuning parameters
double Kp = 777;
double Ki = 0.01;
double Kd = 0.1;

//Specify the links and initial tuning parameters
PID PIDcontroller(&pidIn, &pidOut, &pidSet, Kp, Ki, Kd, DIRECT);
// 10 second Time Proportional Output window
int WindowSize = 1000;
unsigned long windowStartTime;

byte ATuneModeRemember=2;
double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

bool pidTuning = false;
PID_ATune aTune(&pidIn, &pidOut);

//***** PID Functions *****//
void controlLoop(void);
void pidFinishAutoTune(void);

//***** ***** ********* *****//
//***** Begin Functions *****//
//***** ***** ********* *****//

void autoShutoff(){
  heaterOn = false;
  heatingComplete = false;
  if (mqttClient.connected()){
    mqttClient.publish(mqttCoffeeStatusState.c_str(), 1, false, "false");
  }
  shutoffTimer.detach();
}

void updateLed()
{
  ws2812fx.service();
}

void ledStatusController()
{
  if (!heaterOn) {
    if (!wifiReady && ledState!=0){
      ledState = 0;
      ws2812fx.setColor(0x00FF00);
      ws2812fx.setMode(FX_MODE_STATIC);
    }
    else if (wifiReady && ledState!=1){
      ledState = 1;
      ws2812fx.setColor(0x40FF00);
      ws2812fx.setMode(FX_MODE_STATIC);
    }
  }
  else if(heaterOn){
    if (!heatingComplete && ledState != 2){
      ledState = 2;
      ws2812fx.setColor(0x40FF00);
      ws2812fx.setMode(FX_MODE_BLINK);
    }
    else if (heatingComplete && ledState != 3){
      ledState = 3;
      ws2812fx.setColor(0xFF0000);
      ws2812fx.setMode(FX_MODE_STATIC);
    }
  }
}

void getTemperature()
{
  #ifdef DEBUG
    // Serial.print("heater State: ");
    // Serial.println(heaterOn);
  #endif

  if (DS18B20.isConnected(tempSensor)) {
    temperatureIsOk = true;
    temperature = DS18B20.getTempC(tempSensor);
    DS18B20.requestTemperatures();

    if (heaterOn){
      temperatureTracker.add(temperature);
      if (temperatureTracker.count() == 10){
        float avgTemp = temperatureTracker.average();
        float stdTemp = temperatureTracker.pop_stdev();

        if(!heatingComplete){
          if (abs(avgTemp-(float)temperatureSet) < 0.50 ) {
            if (stdTemp < 0.15){
              heatingComplete = true;
              shutoffTimer.attach(shutoffTime*60, autoShutoff);
              if (mqttClient.connected()){
                mqttClient.publish(mqttCoffeeStatusState.c_str(), 1, false, "true");
              }
            }
          }
        }
        else{
          if (abs(avgTemp-(float)temperatureSet) > 2.0 ) {
            heatingComplete = false;
          }
        }

        Serial.print("  Average: ");
        Serial.println(avgTemp);
        Serial.print("    pop stdev: ");
        Serial.println(stdTemp);
        temperatureTracker.clear();
      }
    }

  	pubTemperature();
    Serial.print("heater on time: ");
    Serial.println(onTime);
  }
  else
  {
    temperatureIsOk = false;
    // Serial.print("Temperature Sensor Not Found: ");
    // Serial.println(temperatureIsOk);
  }
}

bool saveTemperatureSettingToFS(){
  itoa(temperatureSet,set_temp, 10);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["temperature"] = set_temp;

  File configFile = SPIFFS.open("/coffeeconfig.json", "w");
  if (!configFile) {
    return false;
  }
  json.printTo(configFile);
  configFile.close();
  return true;
}

bool loadTemperatureSettingFromFS(){
  if (SPIFFS.begin()) {
    if (SPIFFS.exists("/coffeeconfig.json")) {
      File configFile = SPIFFS.open("/coffeeconfig.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          strcpy(set_temp, json["temperature"]);
          temperatureSet = atoi(set_temp);
					return true;
        }
				else {
					return false;
        }
      }
    }
    else{
      saveTemperatureSettingToFS();
    }
  }
}

//Health
void getUptimeAndRSSI(void)
{
	if (!NTPTimeSynced){
		NTP.getTimeDateString();
	}

	if(mqttClient.connected()){
		String uptime = String(NTP.getUptime());
		String rssi = String(WiFi.RSSI());

		uint16_t packetIdPubUptime = mqttClient.publish(mqttDeviceUptime.c_str(), 1, true, uptime.c_str());
		uint16_t packetIdPubRSSI = mqttClient.publish(mqttDeviceRSSI.c_str(), 1, true, rssi.c_str());
	}
}

//***** Data Publish Functions *****//
void pubTemperature(void){
  #ifdef DEBUG
    Serial.print("Temperature: ");
    Serial.println(temperature);
  #endif
	if (mqttClient.connected()) {
		String pubTemperature = String(temperature);
  		uint16_t packetIdPubTemperature = mqttClient.publish(mqttTemperature.c_str(), 1, true, pubTemperature.c_str());

		if(NTPTimeSynced)
		{
			String pTime = String(now());
			uint16_t packetIdPubTemperatureTime = mqttClient.publish(mqttTemperatureTime.c_str(), 1, true, pTime.c_str());
		}
	}
}

void pubheaterOn()
{
	if (mqttClient.connected())
	{
	  if (heaterOn) {
	    uint16_t packetIdPubSwitchState = mqttClient.publish(mqttSwitchState.c_str(), 1, true, "true");

			#ifdef DEBUG
				Serial.println("Heater On");
			#endif
	  }
	  else
	  {
	    uint16_t packetIdPubSwitchState = mqttClient.publish(mqttSwitchState.c_str(), 1, true, "false");

			#ifdef DEBUG
				Serial.println("Heater Off");
			#endif
	  }
		if (NTPTimeSynced) {
			String pTime = String(now());
			uint16_t packetIdPubSwitchTime = mqttClient.publish(mqttSwitchStateTime.c_str(), 1, true, pTime.c_str());
		}
	}
}

//***** Ticher and Button Functions *****//
void tock(){
	button.tick();
}

void buttonClick() {
	heaterOn^=1;
  if (!heaterOn) {
    heatingComplete = false;
    if (mqttClient.connected()){
      mqttClient.publish(mqttCoffeeStatusState.c_str(), 1, false, "false");
    }
  }
	pubheaterOn();
}

void buttonHoldStart()
{
	#ifdef DEBUG
		Serial.println("WiFi Credentials Reset");
	#else
    heaterOn = false;
		WiFiManager wifiManager;
		wifiManager.resetSettings();
    wifiReady = false;
	#endif
}

void buttonHoldEnd()
{
	#ifdef DEBUG
		Serial.println("Restarting System");
	#endif
	delay(500);
	ESP.restart();
}

//***** Relay Functions *****//
void heaterController()
{
  controlLoop();

  if (heaterOn && temperatureIsOk) {
    uint64_t now = millis();
    // Set the output
    // "on time" is proportional to the PID output
    if(now - windowStartTime>WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if((onTime > 10) && (onTime > (now - windowStartTime)))
    {
      digitalWrite(HEATER_PIN, HIGH);
    }
    else
    {
      digitalWrite(HEATER_PIN, LOW);
    }
  }
  else{
    digitalWrite(HEATER_PIN, LOW);
  }
}
//***** WiFi Manager Functions *****//
void enterConfigCallback(WiFiManager *myWiFiManager){

}

void saveConfigCallback(){
  wifiManagerSaveSettings=true;
}
bool wifiManagerLoadConfig(){
  if (SPIFFS.begin()) {
    if (SPIFFS.exists("/wificonfig.json")) {
      File configFile = SPIFFS.open("/wificonfig.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_token, json["mqtt_token"]);
					return true;
        }
				else {
					return false;
        }
      }
    }
  }
	else {
		return false;
  }
}
bool wifiManagerSaveConfig(){
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_token"] = mqtt_token;

  File configFile = SPIFFS.open("/wificonfig.json", "w");
  if (!configFile) {
    return false;
  }

  json.printTo(configFile);
  configFile.close();
  return true;
}

void wifiManagerOpenPortal(){
	//WiFiManager
  WiFiManagerParameter mqtt_server_entry("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter mqtt_port_entry("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter mqtt_user_entry("user", "mqtt user", mqtt_user, 33);
  WiFiManagerParameter mqtt_token_entry("mqtt", "mqtt token", mqtt_token, 33);

	WiFiManager wifiManager;
	#ifdef DEBUG
		//wifiManager.resetSettings();
		wifiManager.setDebugOutput(true);
	#else
		wifiManager.setDebugOutput(false);
	#endif
	wifiManager.setTimeout(300);
  wifiManager.setAPCallback(enterConfigCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add misc wifi manager parameters to be stored in FS
  wifiManager.addParameter(&mqtt_server_entry);
  wifiManager.addParameter(&mqtt_port_entry);
  wifiManager.addParameter(&mqtt_user_entry);
  wifiManager.addParameter(&mqtt_token_entry);
	if(!wifiManager.autoConnect(AP_NAME.c_str())){}

  wifiReady = true;

  if (wifiManagerSaveSettings == true)
	{
		if (mqtt_server_entry.getValue()!="") {
			strcpy(mqtt_server, mqtt_server_entry.getValue());
      strcpy(mqtt_port, mqtt_port_entry.getValue());
		  strcpy(mqtt_user, mqtt_user_entry.getValue());
		  strcpy(mqtt_token, mqtt_token_entry.getValue());
			wifiManagerSaveConfig();
		}
	}
}


//***** MQTT Functions *****//
void onMqttConnect() {
  //Serial.println("** Connected to the broker **");
	//PUBLISH SYSTEM PROPERTIES//
	uint16_t packetIdPub$homieverion = mqttClient.publish(mqttHomieVersion.c_str(), 1, true, homieVersion.c_str());
  uint16_t packetIdPub$online = mqttClient.publish(mqttDeviceOnline.c_str(), 1, true, "true");
  uint16_t packetIdPub$localip = mqttClient.publish(mqttDeviceIP.c_str(), 1, true, localIP.c_str());
  uint16_t packetIdPub$uptimeInterval = mqttClient.publish(mqttDeviceUptimeInterval.c_str(), 1, true, "60");
  uint16_t packetIdPub$fwver = mqttClient.publish(mqttDeviceFWVersion.c_str(), 1, true, APP_FW_VER);
	uint16_t packetIdPub$imp = mqttClient.publish(mqttDeviceImplementation.c_str(), 1, true, "ESP8266-Bunn");
	uint16_t packetIdPub$nodes = mqttClient.publish(mqttDeviceNodes.c_str(), 1, true, mqttNodes.c_str());

	//PUBLISH NODE PROEPRTIES//
	//SWITCH//
	uint16_t packetIdPub$swtype = mqttClient.publish(mqttSwitchType.c_str(), 1, true, "Heater");
  uint16_t packetIdPub$swstateset = mqttClient.publish(mqttSwitchStateSet.c_str(), 1, false, "false");
	uint16_t packetIdPub$swprop = mqttClient.publish(mqttSwitchProperties.c_str(), 1, true, switchProperties.c_str());

	//TEMPERATURE//
	uint16_t packetIdPub$temptype = mqttClient.publish(mqttTemperatureType.c_str(), 1, true, "temperature");
	uint16_t packetIdPub$tempprop = mqttClient.publish(mqttTemperatureProperties.c_str(), 1, true, temperatureProperties.c_str());
	uint16_t packetIdPub$tempunit = mqttClient.publish(mqttTemperatureUnits.c_str(), 1, true, "C");

  //COFFEE//
  uint16_t packetIdPub$coffeetype = mqttClient.publish(mqttCoffeeStatusType.c_str(), 1, true, "status");
  uint16_t packetIdPub$coffeeready = mqttClient.publish(mqttCoffeeStatusState.c_str(), 1, false, "false");
	uint16_t packetIdPub$coffeeprop = mqttClient.publish(mqttCoffeeStatusProperties.c_str(), 1, true, coffeeProperties.c_str());

  //SUBSCRIBE//
  uint16_t packetIdSubSwitchStateSet = mqttClient.subscribe(mqttSwitchStateSet.c_str(), 1);
  uint16_t packetIdSubTemperatureSet = mqttClient.subscribe(mqttTemperatureSet.c_str(), 1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
	#ifdef DEBUG
		Serial.println("MQTT Disconnected");
	#endif
  mqttClient.connect();
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {

  String topicReceived = topic;
  String payloadReceived = payload;

  if (topicReceived == mqttSwitchStateSet) {
    bool setState = false;
    if (payloadReceived == "true") {setState = true;}
    else if (payloadReceived == "false") {setState = false;}
    if (heaterOn != setState)
    {
      heaterOn = setState;
      pubheaterOn();
    }
		// #ifdef DEBUG
	  //   Serial.print(setState);
	  //   Serial.print(", ");
	  //   Serial.println(heaterOn);
		// #endif
  }

	if (topicReceived == mqttTemperatureSet) {
		temperatureSet = payloadReceived.toInt();
    saveTemperatureSettingToFS();
		#ifdef DEBUG
			Serial.print("Temperature set to: ");
			Serial.print(temperatureSet);
			Serial.println(" C");
		#endif
	}
}

//***** PID Functions *****//
void controlLoop()
{
  pidIn = temperature;
  if(PIDcontroller.Compute())
  {
      onTime = pidOut;
      //Serial.println("yes");
  }
}

void pidFinishAutoTune(){}
//***** ***** *** **** *****//
//***** setup and loop *****//
//***** ***** *** **** *****//

void setup() {
  loadTemperatureSettingFromFS();
	//SPIFFS.format();
  //***** IO Setup ******//
	pinMode(HEATER_PIN, OUTPUT);
	digitalWrite(HEATER_PIN, LOW);

  //***** NEOPIXEL SETUP ******//
  ws2812fx.init();
  ws2812fx.setBrightness(255);
  ws2812fx.setSpeed(200);
  ws2812fx.setColor(0x00FF00);
  ws2812fx.setMode(FX_MODE_STATIC);
  ws2812fx.start();
  ledStatusControllerTimer.attach_ms(100,ledStatusController);
  ledTimer.attach_ms(10,updateLed);

	//***** TEMPERATURE SENSOR SETUP ******//
	DS18B20.begin();
  if (!DS18B20.getAddress(tempSensor, 0)) {
    temperatureIsOk = false;
  }
  else {
    temperatureIsOk = true;
  }
  DS18B20.setResolution(tempSensor, 12);
  DS18B20.setWaitForConversion(false);
  temperatureTracker.clear();
	temperatureTimer.attach(1,getTemperature);

	//***** HEALTH TIMER SETUP ******//
	healthTimer.attach(60,getUptimeAndRSSI);

  //***** BUTTON PRESS SETUP ******//
	tick.attach_ms(10, tock);
  button.attachClick(buttonClick);
	button.setClickTicks(50);	//50ms to register a click

  //***** BUTTON HOLD SETUP ******//
	button.attachLongPressStart(buttonHoldStart);
	button.attachLongPressStop(buttonHoldEnd);
	button.setPressTicks(10000);	//hold for 10s to reset system

  //***** SERIAL SETUP *****//
	#ifdef DEBUG
		delay(1000);
	  Serial.begin(38400);
    Serial.print("Heater State: ");
    Serial.println(heaterOn);
  #endif

  //***** HEATER TIMER SETUP ******//
  heaterControlTimer.attach_ms(190,heaterController);

  //turn the PID on
  windowStartTime = millis();
  PIDcontroller.SetSampleTime(1000);
  PIDcontroller.SetOutputLimits(0, WindowSize);
  PIDcontroller.SetTunings(Kp,Ki,Kd);
  PIDcontroller.SetMode(AUTOMATIC);

  Serial.print("Heater State: ");
  Serial.println(heaterOn);
  //***** WIFI SETUP *****//
  wifiManagerLoadConfig();
  wifiManagerOpenPortal();
  localIP = String(WiFi.localIP()[0]) + "." + String(WiFi.localIP()[1]) + "." + String(WiFi.localIP()[2]) + "." + String(WiFi.localIP()[3]);

  //***** MQTT SETUP *****//
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  // mqttClient.setServer(IPAddress(192,168,1,64),1883);
  // mqttClient.setServer("192.168.1.64",1883);
  mqttClient.setServer(mqtt_server, atol(mqtt_port));
  mqttClient.setCredentials(mqtt_user, mqtt_token);
  mqttClient.setCleanSession(false).setKeepAlive(60).setWill(mqttDeviceOnline.c_str(), 0, true, "false").setClientId(mac.c_str());
	#ifdef DEBUG
	  Serial.println("Connecting to MQTT...");
	  Serial.println(WiFi.localIP());
    Serial.print("MQTT Server: ");
    Serial.println(mqtt_server);
    Serial.print("MQTT Port: ");
    Serial.println(mqtt_port);
    Serial.print("MQTT User: ");
    Serial.println(mqtt_user);
    Serial.print("MQTT Pass: ");
    Serial.println(mqtt_token);
	#endif
  mqttClient.connect();

	//***** NTP Setup *****//
	NTP.onNTPSyncEvent([](NTPSyncEvent_t error) {
        if (error) {
					#ifdef DEBUG
				  	Serial.println("NTP server not reachable");
					#endif
					NTPServerOnline = false;
        }
        else {
					#ifdef DEBUG
            Serial.print("Got NTP time: ");
            Serial.println(NTP.getTimeDateString(NTP.getLastNTPSync()));
					#endif
					NTPServerOnline = true;
					NTPTimeSynced = true;
        }
    });
  NTP.begin("time.nist.gov", 0, false);

  //***** ARDUINO OTA *****//
  ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });
	ArduinoOTA.begin();
}

void loop() {
	ArduinoOTA.handle();
}
