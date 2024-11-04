#define SENSENET_DEBUG // enable debug on SerialMon
#define SerialMon Serial // if you need DEBUG SerialMon should be defined

#define FIRMWARE_TITLE "Arduino_Data_Collector"
#define FIRMWARE_VERSION "0.3.14"

#include <Arduino.h>
#include "sensenet.h"
#include "WiFi.h"
#include "Wire.h"
#include "SPI.h"
//#include "Preferences.h"
#include "Ticker.h"
#include <esp_task_wdt.h>
#include <ESP32Time.h>

#include "DEV_Config.h"
#include "L76X.h"

#define WIFI_SSID "Sensenet_2.4G"
#define WIFI_PASS "Sensenet123"
#define TB_URL "tb.sensenet.ca"

// for COM7
//#define TOKEN "SGP4xESP32_3"

// for COM8
//#define TOKEN "SGP4xESP32_2"

// for COM10
#define TOKEN "GPSESP32"
/*
AN128_ardunio_cozir CO2 Demonstration code 11/29/2017 Runs on Ardunio UNO, MEGA or MEGA2560
  Written by: Marv Kausch 11/22/2016 or Co2meter.com
  Revised 11/29/17 by John Houck
   
  This sketch connects a COZIR sensor and reports readings back to the host computer over USB.
  The value is stored in a global variable 'co2' and can be used for any number of applications.
   
  pin connections:
   
  Arduino________COZIR Sensor
   GND ------------------ 1 (gnd)
   3.3v------------------- 3 (Vcc)  
    11 -------------------- 5 (Rx)
    12 -------------------- 7 (Tx)


Modified by RM on 04/17/24 to work with strings and CM-200 and LEDs
*/
#include <SoftwareSerial.h>
#include "Adafruit_CCS811.h"
#include "ClosedCube_HDC1080.h"

ESP32Time internalRtc(0);  // offset in seconds GMT
Ticker restartTicker;
//Preferences preferences;
NetworkInterface wifiInterface("wifi", 2, 2);
NetworkInterfacesController networkController;
MQTTController mqttController;
MQTTOTA ota(&mqttController, 5120);

WiFiClient wiFiClient;

int i = 0;

void resetESP() {
    ESP.restart();
}

uint64_t getTimestamp() {
    if (internalRtc.getEpoch() < 946713600)
        return 0;
    uint64_t ts = internalRtc.getEpoch();
    ts = ts * 1000L;
    ts = ts + internalRtc.getMillis();
    return ts;
}

// Sampling interval in seconds
char errorMessage[32];

bool on_message(const String &topic, DynamicJsonDocument json) {
    Serial.print("Topic1: ");
    Serial.println(topic);
    Serial.print("Message1: ");
    Serial.println(json.as<String>());

    if (json.containsKey("shared")) {
        JsonObject sharedKeys = json["shared"].as<JsonObject>();
        for (JsonPair kv: sharedKeys)
            json[kv.key()] = sharedKeys[kv.key()];

    }

    if (json.containsKey("method")) {
        String method = json["method"].as<String>();

        bool handled = false;
        if (method.equalsIgnoreCase("restart_device")) {
            float seconds = 0;
            if (json["params"].containsKey("seconds"))
                seconds = json["params"]["seconds"];
            if (seconds == 0) seconds = 1;
            printDBGln("Device Will Restart in " + String(seconds) + " Seconds");
            restartTicker.once(seconds, resetESP);
            handled = true;
        }

        if (handled) {
            String responseTopic = String(topic);
            responseTopic.replace("request", "response");
            DynamicJsonDocument responsePayload(300);
            responsePayload["result"] = "true";
            mqttController.addToPublishQueue(responseTopic, responsePayload.as<String>(), true);
            return true;
        }
    }

    return false;
}

void connectToNetwork() {
    Serial.println("Added WiFi Interface");
    networkController.addNetworkInterface(&wifiInterface);

    networkController.setAutoReconnect(true, 10000);
    networkController.autoConnectToNetwork();
}

void connectToPlatform(Client &client, const bool enableOTA) {

    Serial.println("Trying to Connect Platform");
    mqttController.connect(client, "esp", TOKEN, "", TB_URL,
                           1883, on_message,
                           nullptr, [&]() {
                Serial.println("Connected To Platform");
                DynamicJsonDocument info(512);
                info["Token"] = TOKEN;
                info.shrinkToFit();
                mqttController.sendAttributes(info, true);
                if (enableOTA)
                    ota.begin(FIRMWARE_TITLE, FIRMWARE_VERSION);
                else
                    ota.stopHandleOTAMessages();

                DynamicJsonDocument requestKeys(512);
                requestKeys["sharedKeys"] = "desiredAllowSleep,desiredDisableIR,desiredSEN55TempOffset";
                requestKeys.shrinkToFit();
                mqttController.requestAttributesJson(requestKeys.as<String>());

                if (getTimestamp() == 0) {
                    DynamicJsonDocument requestTime(512);
                    requestTime["method"] = "requestTimestamp";
                    requestTime.shrinkToFit();
                    mqttController.requestRPC(requestTime.as<String>(),
                                              [](const String &rpcTopic, const DynamicJsonDocument &rpcJson) -> bool {
                                                  Serial.print("Updating Internal RTC to: ");
                                                  Serial.println(rpcJson.as<String>());
                                                  uint64_t tsFromCloud = rpcJson["timestamp"].as<uint64_t>();
                                                  tsFromCloud = tsFromCloud / 1000;
                                                  internalRtc.setTime(tsFromCloud);
                                                  Serial.print("Internal RTC updated to: ");
                                                  Serial.println(internalRtc.getDateTime(true));
                                                  getTimestamp();
                                                  return true;
                                              });
                } else {
                    Serial.print("Internal RTC updated to: ");
                    Serial.println(internalRtc.getDateTime(true));
                }
            });
}

int retry = 0;

void initInterfaces() {
    retry = 0;
    wifiInterface.setTimeoutMs(30000);
    wifiInterface.setConnectInterface([]() -> bool {
        Serial.println(String("Connecting To WiFi ") + WIFI_SSID);
        WiFi.mode(WIFI_MODE_NULL);
        delay(2000);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        return true;
    });
    wifiInterface.setConnectionCheckInterfaceInterface([]() -> bool {
        return WiFi.status() == WL_CONNECTED;
    });
    wifiInterface.OnConnectingEvent([]() {
        Serial.print(".");
    }, 500);
    wifiInterface.OnConnectedEvent([]() {
        retry = 0;
        Serial.println(String("Connected to WIFI with IP: ") + WiFi.localIP().toString());
        connectToPlatform(wiFiClient, true);
        DynamicJsonDocument data(200);
        data["Connection Type"] = "WIFI";
        data["IP"] = WiFi.localIP().toString();
        data.shrinkToFit();
    });
    wifiInterface.OnTimeoutEvent([]() {
        retry++;
        Serial.println("WiFi Connecting Timeout! retrying for " + String(retry) + " Times");
        WiFi.mode(WIFI_MODE_NULL);

//        if (retry >= 20)
//            ESP.restart();
    });
}

SoftwareSerial mySerial(12, 11); // RX, TX pins on Ardunio
Adafruit_CCS811 ccs;
ClosedCube_HDC1080 hdc1080;

// if using a CO2Meter CM-200 Sensor Development Board, set this to 1 if you want the LEDs to flash
// if you're wiring directly to the sensor, set this to 0
#define CM200 0

#if CM200
#define GREEN_LED 4
#define BLUE_LED 5
#define RED_LED 6

uint8_t ind = GREEN_LED;
#endif

// if you want the sensor to stream set this to 1
// if you want to use the sensor in polling mode, set this to 0 and set the delay_ms to desired delay time
#define STREAMING 1
const int delay_ms = 500;

const uint8_t buff_size = 64;
//uint8_t buff[buff_size];
uint8_t index1 = 0;

int fill_buffer();  // function prototypes here
int format_output();
double parse_response(char * str);
void printSerialNumber();
void setupLOX_02_CJMCU8118();
void loopLOX_02_CJMCU8118(DynamicJsonDocument &data) {
  double o2_pct = 0.0;
  double o2_pps = 0.0;
  double o2_tmp = 0.0;
  double o2_prs = 0.0;
  double o2_err = 0.0;
  char buff[buff_size];

#if CM200
  // if using a CM-200 we can cycle through the LEDs to high and low so we can see that we're in the loop
  if (digitalRead(ind) == HIGH)
    digitalWrite(ind, LOW);
  else
    digitalWrite(ind, HIGH);

  ind++;
  if (ind > RED_LED)
    ind = GREEN_LED;
#endif

  mySerial.flush();
#if STREAMING == 0
  mySerial.println("%");  // set streaming mode
#endif

  int timeout = 5000;
  while ( mySerial.available() <= 0 && timeout-- > 0) {
    Serial.print(".");
    delay(10);
  }
  Serial.println("");

  delay(50); //delay 50ms to give the Serial buffer some time to fill
  Serial.print("Sensor Response: ");
  index1 = 0;
  while ( mySerial.available() > 0 ) {
    char tmp = mySerial.read();
    buff[index1] = tmp;
    Serial.print((char)tmp);

    if (tmp == '\n')
      break;

    index1++;
    if (index1 >= buff_size) { //let's not overflow the buffer
      Serial.println("Overflowed buffer... Bail!");
      break;
    }
  }

  Serial.println("");

  char *p = buff;
  char *str;
  
  while ((str = strtok_r(p, " ", &p)) != NULL) {
    if (str[0] == 'O') {
      str = strtok_r(p, " ", &p);
      o2_pps = parse_response(str);
    }
    if (str[0] == '%') {
      str = strtok_r(p, " ", &p);
      o2_pct = parse_response(str);
    }
    if (str[0] == 'T') {
      str = strtok_r(p, " ", &p);
      o2_tmp = parse_response(str);
    }
    if (str[0] == 'P') {
      str = strtok_r(p, " ", &p);
      o2_prs = parse_response(str);
    }
    if (str[0] == 'e') {
      str = strtok_r(p, " ", &p);
      o2_err = parse_response(str);
    }
  }

  Serial.println("");
  Serial.print("***** O2 Percent: ");
  Serial.print(o2_pct);
  data["o2_pct"] = String(o2_pct);
  Serial.print("\r\n");
#if STREAMING   // we don't need to print this out if we're not streaming
  Serial.print("***** O2 Partial Pressure: ");
  Serial.print(o2_pps);
  data["o2_pps"] = String(o2_pps);
  Serial.print("\r\n");

  Serial.print("***** Sensor Temperature: ");
  Serial.print(o2_tmp);
  data["o2_tmp"] = String(o2_tmp);
  Serial.print("\r\n");

  Serial.print("***** Sensor Pressure: ");
  Serial.print(o2_prs);
  data["o2_prs"] = String(o2_prs);
  Serial.print("\r\n");

  Serial.print("***** Sensor Error: ");
  Serial.print(o2_err);
  data["o2_err"] = String(o2_err);
  Serial.print("\r\n");
#endif

  if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("CO2: ");
      uint16_t CO2_ppm = ccs.geteCO2();
      Serial.print(CO2_ppm);
      data["CO2_ppm"] = String(CO2_ppm);
      Serial.print("ppm, TVOC: ");
      uint16_t TVOC = ccs.getTVOC();
      Serial.println(TVOC);
      data["TVOC"] = String(TVOC);
    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
  }
	Serial.print("T=");
  double hdcTemperature = hdc1080.readTemperature();
	Serial.print(hdcTemperature);
  data["hdcTemperature"] = String(hdcTemperature);
	Serial.print("C, RH=");
  double hdcHumidity = hdc1080.readHumidity();
	Serial.print(hdcHumidity);
  data["hdcHumidity"] = String(hdcHumidity);
	Serial.println("%");
  delay(delay_ms);
}

uint64_t lastLOX_02_CJMCU8118 = 0;

void core0Loop(void *parameter) {
    //Dont do anything 1
    esp_task_wdt_init(600, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    //Dont do anything1

    lastLOX_02_CJMCU8118 = Uptime.getMilliseconds();
    uint64_t now = Uptime.getMilliseconds();
    DynamicJsonDocument data(5120);
    if (now - lastLOX_02_CJMCU8118 > 60000) {
        lastLOX_02_CJMCU8118 = now;
        loopLOX_02_CJMCU8118(data);
    }

    if (data.size() > 0 && getTimestamp() > 0) {
        data.shrinkToFit();
        Serial.println("Data: " + data.as<String>());
        mqttController.sendTelemetry(data, true, getTimestamp());
    }
    delayMicroseconds(1);
    esp_task_wdt_reset();
}

void setup() {
    //Dont do anything in setup
    //Add setup SEN55
    internalRtc.setTime(1000);
    btStop();
    esp_task_wdt_init(60, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    esp_task_wdt_reset();

    Serial.begin(9600);
//    preferences.begin("Configs", false);
//    Serial.println("Hello from: " + preferences.getString("token", "not-set"));
    mqttController.init();
    mqttController.sendSystemAttributes(true);
    initInterfaces();
    Wire.begin();
    esp_task_wdt_reset();

    uint16_t error;
    char errorMessage[256];
    delay(1000);  // needed on some Arduino boards in order to have Serial ready

    esp_task_wdt_reset();
    connectToNetwork();
    esp_task_wdt_reset();
    setupLOX_02_CJMCU8118();
    delay(1000);
    xTaskCreatePinnedToCore(
            core0Loop, // Function to implement the task
            "Core0Loop", // Name of the task
            10000, // Stack size in words
            NULL,  // Task input parameter
            0, // Priority of the task
            NULL,  // Task handle.
            0); // Core where the task should run
    esp_task_wdt_reset();
}

uint64_t core1Heartbeat;

void loop() {
    //Dont do anything
    esp_task_wdt_reset();

    if (Serial.available()) {
        if (Serial.readString().indexOf("reboot") >= 0)
            resetESP();
    }

    if (networkController.getCurrentNetworkInterface() != nullptr &&
        networkController.getCurrentNetworkInterface()->lastConnectionStatus()) {
        mqttController.loop();
    }

    networkController.loop();

    if ((Uptime.getSeconds() - core1Heartbeat) > 10) {
        core1Heartbeat = Uptime.getSeconds();
        printDBGln("Core 1 Heartbeat");
    }
}

void setupLOX_02_CJMCU8118() {
  Serial.print("\n\n");
  Serial.println("             AN128 Ardunio to Cozir CO2 Sensor - Demonstration code 11/29/2017\n\n");

#if CM200
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
#endif
  
  mySerial.begin(9600); // Start serial communications with sensor

#if STREAMING
  mySerial.println("M 0");  // set streaming mode
#else
  mySerial.println("M 1");  // set polling mode
#endif


  Serial.println("CCS811 test");

  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  // Wait for the sensor to be ready
  while(!ccs.available());

	Serial.println("ClosedCube HDC1080 Arduino Test");

	// Default settings: 
	//  - Heater off
	//  - 14 bit Temperature and Humidity Measurement Resolutions
	hdc1080.begin(0x40);

	Serial.print("Manufacturer ID=0x");
	Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
	Serial.print("Device ID=0x");
	Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
	
	printSerialNumber();
}

double parse_response(char * str) {
  for (int i = 0; i < strlen(str); i++) { //cycle through char array making sure we're all digits
    if (!isDigit(str[i]) && str[i] != '+' && str[i] != '.') {
     str[i] = '\0';
    }
  }

  return atof(str); // this is not great we could crash here if the first character is not a number
}

void printSerialNumber() {
	Serial.print("Device Serial Number=");
	HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
	char format[12];
	sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
	Serial.println(format);
}