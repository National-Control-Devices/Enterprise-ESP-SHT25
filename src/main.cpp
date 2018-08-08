#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <WiFiClientSecure.h>
#include <SHT25.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <esp_wifi.h>
#include <HIH7120.h>

HIH7120 sensor;

int firmwareVersion = 1;
int nodeID = 0;
float battery = 0.00;
int counter = 0;
int sensorType = 1;
String macAddress;
String packetType;
char moduleIPString[17];
IPAddress moduleIP;

unsigned long sensorInterval = 2000;
unsigned long lastSensorReading = 0;
bool forceOnInterval = true;

double tempChangeLimit = 0.50;
double previousTempReading;

double humChangeLimit = 0.50;
double previousHumReading;

IPAddress apIP(192, 168, 1, 1);
const char* apSSID = "SHT25_Sensor";
const char* password = "password";

//UDP stuff
bool udpEnabled = true;
IPAddress broadcastIP;
WiFiUDP udp;
int udpPort;
int udpPortIndex = 364;

const byte DNS_PORT = 53;
DNSServer dnsServer;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)
//
uint8_t button = 14;
bool previousButtonState;
bool setupMode = false;
bool boot = true;

//ssid
int ssidIndex = 0;
char ssid[50];

//wlan password
int ssidPassIndex = 50;
char ssidPass[50];

int nodeIDIndex = 100;
int intervalIndex = 104;

//MQTT
bool mqttEnabled = true;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//MQTT credentials
char mqttUserName[50];
int mqttUserNameIndex = 108;
char mqttPassword[50];
int mqttPasswordIndex = 158;
char mqttHost[50];
int mqttHostIndex = 208;
int mqttPort;
int mqttPortIndex = 258;
char tempTopic[50];
int tempTopicIndex = 262;
char humTopic[50];
int humTopicIndex = 312;
int udpEnabledIndex = 362;
int mqttEnabledIndex = 363;

//TCP client connection
int tcpServerPort;
int tcpServerPortIndex = 368;
IPAddress tcpServerAddress;
int tcpServerAddressIndex = 372;
char tcpServerAddressString[17];
bool tcpEnabled;
int tcpEnabledIndex = 376;
unsigned long tcpConnectionTimeout;
int tcpConnectionTimeoutIndex = 380;

//TCP server connection
WiFiServer tcpServer;
WiFiClient remoteClient;
int serverPort;
int serverPortIndex = 384;
bool serverEnabled;
int serverEnabledIndex = 388;

bool httpServerEnabled;
int httpServerEnabledIndex = 392;

void configUpdate(AsyncWebServerRequest*);
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void loadSettings();
void onRequest(AsyncWebServerRequest *request);
void handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void storeSettings(String s, bool forceReset);
void processInterval();

void setup(){
  Serial.begin(115200);
  EEPROM.begin(1024);
  loadSettings();
  pinMode(button, INPUT_PULLUP);
  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS Mount Failed");
  }
  if(digitalRead(button) == 0){
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(apSSID, password, 1, 0, 1);
    dnsServer.start(DNS_PORT, "*", apIP);
    setupMode = true;
    Serial.println("Setup Mode");

  }else{
    if(udpEnabled || mqttEnabled || tcpEnabled || serverEnabled){
      Serial.println("Run Mode");
      Serial.printf("Connecting to %s with password %s\n", ssid, ssidPass);
      WiFi.begin(ssid, ssidPass);
      Serial.print("Connecting to WiFi");
      unsigned long wifiConnectTimeout = 10000;
      unsigned long startConnectTime = millis();
      while(WiFi.status() != WL_CONNECTED && millis() < startConnectTime+wifiConnectTimeout){
        Serial.print(".");
        delay(500);
      }
      if(WiFi.status() == WL_CONNECTED){
        Serial.println();
        Serial.print("WiFi Connected to: ");
        Serial.println(ssid);
        Serial.println(WiFi.localIP());
        broadcastIP = WiFi.gatewayIP();
        broadcastIP[3] = 255;
        macAddress = WiFi.macAddress();
        moduleIP = WiFi.localIP();
        sprintf(moduleIPString, "%i.%i.%i.%i", moduleIP[0], moduleIP[1], moduleIP[2], moduleIP[3]);
        if(mqttEnabled){
          mqttClient.setServer(mqttHost, mqttPort);
        }
        if(serverEnabled){
          tcpServer = WiFiServer(serverPort);
          tcpServer.begin();
        }
        if(httpServerEnabled && !setupMode){
          server.onNotFound(onRequest);
          server.begin();
        }
      }else{
        Serial.println("WiFi Connect failed");
      }

    }else{
      Serial.println("Serial only mode");
    }
  }
}

void loop(){
  if(Serial.available() != 0){
    delay(50);
    int serialLen = Serial.available();
    char serialData[serialLen];
    for(int i = 0; i < serialLen; i++){
      serialData[i] = Serial.read();
    }
    storeSettings(String(serialData), false);
  }

  if(setupMode && boot){
    server.on("/update", HTTP_POST, configUpdate, NULL, handleBody);
    server.onNotFound(onRequest);
    server.begin();
    boot = false;
  }
  if(setupMode){
    dnsServer.processNextRequest();
  }else{
    if(millis() > lastSensorReading + sensorInterval || boot){
      lastSensorReading = millis();
      processInterval();
      boot = false;
    }
  }
}

void handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
  if(!index){
    Serial.printf("BodyStart: %u B\n", total);
  }
  for(size_t i=0; i<len; i++){
    Serial.write(data[i]);
  }
  if(index + len == total){
    Serial.printf("BodyEnd: %u B\n", total);
  }
}

void configUpdate(AsyncWebServerRequest *request){
  if(request->args() > 0){
    StaticJsonBuffer<700> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["UDPENABLED"] = request->hasArg("UDPENABLED");
    root["MQTTENABLED"] = request->hasArg("MQTTENABLED");
    root["TCPENABLED"] = request->hasArg("TCPENABLED");
    root["TCPLISTENENABLED"] = request->hasArg("TCPLISTENENABLED");
    root["HTTPENABLED"] = request->hasArg("HTTPENABLED");
    for( uint8_t i = 0; i < request->args(); i++ ){
      if(request->argName(i) != "UDPENABLED" && request->argName(i) != "MQTTENABLED"){
        root[request->argName(i)] = request->arg(i).c_str();
      }
    }
    request->send(200, "text/plain", "Settings updated");
    String jsonString;
    root.printTo(jsonString);
    storeSettings(jsonString, true);
  }
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
  Serial.println("Handling file upload");
  if(request->hasParam("cert_key", true, true)){
    Serial.println("File Uploaded");
  }else{
    Serial.println("No File Uploaded");
  }
}

void loadSettings(){
  memset(ssid, 0x00, 50);
  EEPROM.get(ssidIndex, ssid);
  memset(ssidPass, 0x00, 50);
  EEPROM.get(ssidPassIndex, ssidPass);
  EEPROM.get(nodeIDIndex, nodeID);
  EEPROM.get(intervalIndex, sensorInterval);
  EEPROM.get(udpEnabledIndex, udpEnabled);
  EEPROM.get(mqttEnabledIndex, mqttEnabled);
  EEPROM.get(mqttPortIndex, mqttPort);
  memset(mqttHost, 0x00, 50);
  EEPROM.get(mqttHostIndex, mqttHost);
  memset(mqttUserName, 0x00, 50);
  EEPROM.get(mqttUserNameIndex, mqttUserName);
  memset(mqttPassword, 0x00, 50);
  EEPROM.get(mqttPasswordIndex, mqttPassword);
  memset(tempTopic, 0x00, 50);
  EEPROM.get(tempTopicIndex, tempTopic);
  memset(humTopic, 0x00, 50);
  EEPROM.get(humTopicIndex, humTopic);
  EEPROM.get(udpPortIndex, udpPort);
  EEPROM.get(tcpServerPortIndex, tcpServerPort);
  EEPROM.get(tcpServerAddressIndex, tcpServerAddress);
  sprintf(tcpServerAddressString, "%i.%i.%i.%i", tcpServerAddress[0], tcpServerAddress[1], tcpServerAddress[2], tcpServerAddress[3]);
  EEPROM.get(tcpEnabledIndex, tcpEnabled);
  EEPROM.get(tcpConnectionTimeoutIndex, tcpConnectionTimeout);
  EEPROM.get(serverPortIndex, serverPort);
  EEPROM.get(serverEnabledIndex, serverEnabled);
  EEPROM.get(httpServerEnabledIndex, httpServerEnabled);
}

void onRequest(AsyncWebServerRequest *request){
  //Handle Unknown Request


  Serial.printf("params: %i\n",request->params());
  Serial.println(request->url());
  Serial.println("onRequest fired");
  if(request->url() == "/loadSettings"){
    loadSettings();
    StaticJsonBuffer<700> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["SSID"] = ssid;
    root["PASS"] = ssidPass;
    root["NODEID"] = nodeID;
    root["INTERVAL"] = sensorInterval/1000;
    udpEnabled ? root["UDPENABLED"] = 1 : root["UDPENABLED"] = 0;
    mqttEnabled ? root["MQTTENABLED"] = 1 : root["MQTTENABLED"] = 0;
    root["MQTTHOST"] = mqttHost;
    root["MQTTPORT"] = mqttPort;
    root["MQTTUSERNAME"] = mqttUserName;
    root["MQTTPASSWORD"] = mqttPassword;
    root["MQTTTEMPERATURETOPIC"] = tempTopic;
    root["MQTTHUMIDITYTOPIC"] = humTopic;
    root["UDPPORT"] = udpPort;
    root["TCPENABLED"] = tcpEnabled;
    root["TCPADDRESS"] = tcpServerAddressString;
    root["TCPPORT"] = tcpServerPort;
    root["TCPTIMEOUT"] = tcpConnectionTimeout;
    root["TCPLISTENENABLED"] = serverEnabled;
    root["TCPLISTENPORT"] = serverPort;
    root["HTTPENABLED"] = httpServerEnabled;

    String responseString;
    root.printTo(responseString);

    request->send(200, "text/plain", responseString);
    return;
  }
  if(request->url() == "/read"){
    sensor.update();
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["temperature"] = sensor.fTemp;
    root["humidity"] = sensor.humidity;
    String sensorDataString;
    root.printTo(sensorDataString);
    request->send(200, "text/plain", sensorDataString);
  }
  if(request->url() == "/update"){
    Serial.println("handling /update request");
    return;
  }
  if(setupMode){
    request->send(SPIFFS, "/Config.html");
    return;
  }else{
    if(request->url() == "/"){
      request->send(SPIFFS, "/index.html");
    }
    if(request->url() == "/Config"){
      request->send(SPIFFS, "/Config.html");
      return;
    }
  }

}

void storeSettings(String s, bool forceReset){
  StaticJsonBuffer<700> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(s);

  if (!root.success()) {
    Serial.println("invalid json");
    return;
  }
  if(root.containsKey("SSID")){
    forceReset = true;
    memset(ssid, 0x00, 50);
    strcpy(ssid, root["SSID"]);
    EEPROM.put(ssidIndex, ssid);
    EEPROM.commit();
  }
  if(root.containsKey("PASS")){
    forceReset = true;
    memset(ssidPass, 0x00, 50);
    strcpy(ssidPass, root["PASS"]);
    EEPROM.put(ssidPassIndex, ssidPass);
    EEPROM.commit();
  }
  if(root.containsKey("NODEID")){
    nodeID = root["NODEID"].as<int>();
    EEPROM.put(nodeIDIndex, nodeID);
    EEPROM.commit();
  }
  if(root.containsKey("INTERVAL")){
    sensorInterval = root["INTERVAL"].as<int>()*1000;
    EEPROM.put(intervalIndex, sensorInterval);
    EEPROM.commit();
  }
  if(root.containsKey("UDPENABLED")){
    udpEnabled = root["UDPENABLED"].as<bool>();
    EEPROM.put(udpEnabledIndex, udpEnabled);
    EEPROM.commit();
  }
  if(root.containsKey("MQTTENABLED")){
    forceReset = true;
    mqttEnabled = root["MQTTENABLED"].as<bool>();
    EEPROM.put(mqttEnabledIndex, mqttEnabled);
    EEPROM.commit();
  }
  if(root.containsKey("MQTTHOST")){
    forceReset = true;
    memset(mqttHost, 0x00, 50);
    strcpy(mqttHost, root["MQTTHOST"]);
    EEPROM.put(mqttHostIndex, mqttHost);
    EEPROM.commit();
  }
  if(root.containsKey("MQTTPORT")){
    forceReset = true;
    mqttPort = root["MQTTPORT"].as<int>();
    EEPROM.put(mqttPortIndex, mqttPort);
    EEPROM.commit();
  }
  if(root.containsKey("MQTTUSERNAME")){
    forceReset = true;
    memset(mqttUserName, 0x00, 50);
    strcpy(mqttUserName, root["MQTTUSERNAME"]);
    EEPROM.put(mqttUserNameIndex, mqttUserName);
    EEPROM.commit();
  }
  if(root.containsKey("MQTTPASSWORD")){
    forceReset = true;
    memset(mqttPassword, 0x00, 50);
    strcpy(mqttPassword, root["MQTTPASSWORD"]);
    EEPROM.put(mqttPasswordIndex, mqttPassword);
    EEPROM.commit();
  }
  if(root.containsKey("MQTTTEMPERATURETOPIC")){
    memset(tempTopic, 0x00, 50);
    strcpy(tempTopic, root["MQTTTEMPERATURETOPIC"]);
    EEPROM.put(tempTopicIndex, tempTopic);
    EEPROM.commit();
  }
  if(root.containsKey("MQTTHUMIDITYTOPIC")){
    memset(humTopic, 0x00, 50);
    strcpy(humTopic, root["MQTTHUMIDITYTOPIC"]);
    EEPROM.put(humTopicIndex, humTopic);
    EEPROM.commit();
  }
  if(root.containsKey("UDPPORT")){
    udpPort = root["UDPPORT"].as<int>();
    EEPROM.put(udpPortIndex, udpPort);
    EEPROM.commit();
  }
  if(root.containsKey("TCPADDRESS")){
    String tcpAddressString = root["TCPADDRESS"];
    char addrBuf[tcpAddressString.length()+1];
    tcpAddressString.toCharArray(addrBuf, sizeof(addrBuf));
    char *p = addrBuf;
    char *str;
    int i = 0;
    while ((str = strtok_r(p, ".", &p)) != NULL && i < 4){
      String s = String(str);
      tcpServerAddress[i] = s.toInt();
      i++;
    }
    sprintf(tcpServerAddressString, "%i.%i.%i.%i", tcpServerAddress[0], tcpServerAddress[1], tcpServerAddress[2], tcpServerAddress[3]);
    EEPROM.put(tcpServerAddressIndex, tcpServerAddress);
    EEPROM.commit();
  }
  if(root.containsKey("TCPPORT")){
    tcpServerPort = root["TCPPORT"].as<int>();
    EEPROM.put(tcpServerPortIndex, tcpServerPort);
    EEPROM.commit();
  }
  if(root.containsKey("TCPENABLED")){
    tcpEnabled = root["TCPENABLED"].as<bool>();
    EEPROM.put(tcpEnabledIndex, tcpEnabled);
    EEPROM.commit();
  }
  if(root.containsKey("TCPTIMEOUT")){
    tcpConnectionTimeout = root["TCPTIMEOUT"].as<unsigned long>();
    EEPROM.put(tcpConnectionTimeoutIndex, tcpConnectionTimeout);
    EEPROM.commit();
  }
  if(root.containsKey("TCPLISTENENABLED")){
    forceReset = true;
    serverEnabled = root["TCPLISTENENABLED"].as<bool>();
    EEPROM.put(serverEnabledIndex, serverEnabled);
    EEPROM.commit();
  }
  if(root.containsKey("TCPLISTENPORT")){
    forceReset = true;
    serverPort = root["TCPLISTENPORT"].as<int>();
    EEPROM.put(serverPortIndex, serverPort);
    EEPROM.commit();
  }
  if(root.containsKey("HTTPENABLED")){
    httpServerEnabled = root["HTTPENABLED"].as<bool>();
    EEPROM.put(httpServerEnabledIndex, httpServerEnabled);
    EEPROM.commit();
  }
  if(forceReset){
    delay(1000);
    ESP.restart();
  }

}

void processInterval(){
  sensor.update();
  double tempReading = sensor.fTemp;
  double humReading = sensor.humidity;

  double tempDiff = tempReading > previousTempReading ? tempReading - previousTempReading : previousTempReading - tempReading;
  double humDiff = humReading > previousHumReading ? humReading - previousHumReading : previousHumReading - humReading;

  if(tempDiff > tempChangeLimit || humDiff > humChangeLimit || forceOnInterval){

    previousTempReading = tempReading;
    previousHumReading = humReading;

    counter++;
    packetType = "sensor_data";

    StaticJsonBuffer<700> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& data = root.createNestedObject("data");
    data["addr"] = macAddress;
    data["battery"] = battery;
    data["counter"] = counter;
    data["firmware"] = firmwareVersion;
    data["nodeId"] = nodeID;
    JsonObject& sensorData = data.createNestedObject("sensor_data");
    sensorData["humidity"] = humReading;
    sensorData["temperature"] = tempReading;
    data["sensor_type"] = sensorType;
    data["type"] = packetType;
    JsonObject& configData = data.createNestedObject("config_data");
    configData["SSID"] = ssid;
    configData["NODEID"] = nodeID;
    configData["INTERVAL"] = sensorInterval/1000;
    configData["UDPENABLED"] = udpEnabled;
    configData["UDPPORT"] = udpPort;
    configData["MQTTENABLED"] = mqttEnabled;
    configData["MQTTHOST"] = mqttHost;
    configData["MQTTPORT"] = mqttPort;
    configData["MQTTUSERNAME"] = mqttUserName;
    configData["MQTTTEMPERATURETOPIC"] = tempTopic;
    configData["MQTTHUMIDITYTOPIC"] = humTopic;
    configData["TCPENABLED"] = tcpEnabled;
    configData["TCPADDRESS"] = tcpServerAddressString;
    configData["TCPSERVERPORT"] = tcpServerPort;
    configData["TCPTIMEOUT"] = tcpConnectionTimeout;
    configData["TCPLISTENENABLED"] = serverEnabled;
    configData["TCPLISTENPORT"] = serverPort;
    configData["LOCALIP"] = moduleIPString;

    String sensorDataString;

    root.prettyPrintTo(sensorDataString);
    Serial.println(sensorDataString);

    if(udpEnabled || mqttEnabled || tcpEnabled || serverEnabled){
      if(!WiFi.isConnected()){
        unsigned long wifiConnectTimeout = 10000;
        unsigned long startConnectTime = millis();
        WiFi.begin(ssid, ssidPass);
        Serial.print("Connecting to WiFi");
        while(WiFi.status() != WL_CONNECTED && millis() < startConnectTime+wifiConnectTimeout){
          Serial.print(".");
          delay(500);
        }
        if(WiFi.status() != WL_CONNECTED){
          Serial.println("WiFi reconnect failed");
          return;
        }
        Serial.println();
        Serial.print("WiFi Reconnected to: ");
        Serial.print(ssid);
        Serial.printf(" after %i milliseconds.\n", millis()-startConnectTime);
        Serial.println(WiFi.localIP());
      }
    }else{
      //No need to do anything over WiFi, we are only outputing readings and data over usb.
      if(WiFi.isConnected()){
        WiFi.disconnect(true);
      }
      return;
    }

    //UDP broadcast
    if(WiFi.isConnected() && udpEnabled){
      udp.beginPacket(broadcastIP, udpPort);
      udp.print(sensorDataString);
      udp.endPacket();
    }

    //MQTT publish
    if(WiFi.isConnected()){
      if(!mqttClient.connected() && mqttEnabled){
        wifiClient.readStringUntil('\r');
        char macBytes[19];
        macAddress.toCharArray(macBytes, 19);
        if(!mqttClient.connect(macBytes, mqttUserName, mqttPassword)){
          Serial.printf("Failed to connect MQTT client, code:%i\n",mqttClient.state());
        }
        delay(500);
      }
      if(mqttClient.connected()){
        String tempString(tempReading);
        char tempChars[12];
        tempString.toCharArray(tempChars, 12);
        if(!mqttClient.publish(tempTopic, tempChars)){
          Serial.println("MQTT Temperature publish failed");
        }

        String humString(humReading);
        char humChars[12];
        humString.toCharArray(humChars, 12);
        if(!mqttClient.publish(humTopic, humChars)){
          Serial.println("MQTT Humidity publish failed");
        }
      }
    }

    //TCP connection to remote server
    if(WiFi.isConnected() && tcpEnabled){
      unsigned long start = millis();
      if(wifiClient.connect(tcpServerAddress, tcpServerPort)){
        wifiClient.println(sensorDataString);
        while(millis() < start+tcpConnectionTimeout){
          if(wifiClient.available() != 0){
            delay(100);
            int receiveLen = wifiClient.available();
            char receiveData[receiveLen];
            for(int i = 0; i < receiveLen; i++){
              receiveData[i] = wifiClient.read();
            }
            storeSettings(String(receiveData), false);
          }

        }
        wifiClient.stop();
      }else{
        Serial.println("TCP server connect failed");
      }
    }

    //Accept TCP socket connection from client
    if(WiFi.isConnected() && serverEnabled){
      if(!remoteClient){
        remoteClient = tcpServer.available();
      }
      if (remoteClient) {
        remoteClient.println(sensorDataString);
        if(remoteClient.available() != 0){
          delay(100);
          int receiveLen = remoteClient.available();
          char receiveData[receiveLen];
          for(int i = 0; i < receiveLen; i++){
            receiveData[i] = remoteClient.read();
          }
          storeSettings(String(receiveData), false);
        }
      }
    }

  }
}
