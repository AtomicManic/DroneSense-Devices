#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "secrets.h"
#include "defs.h"
#include "aws.h"
#include "base64.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Flags
bool lastSendStatus = false;
String sentDataType = "";
bool dhtFlag = false;
bool accFlag = false;
bool gpsFlag = false;
bool audioFlag = false;
bool displayMode = false;
volatile bool displayToggleRequested = false; // Changed to volatile

// Data
audio_packet incomingAudioData;
StructAccData lastReceivedAccData;
StructDhtData lastReceivedDhtData;
StructGPSData lastReceivedGPSData;

QueueHandle_t loRaQueue;

// Audio Data Buffer
char *audioDataBuffer = nullptr;
int audioBufferSize = 0;
int audioDataSize = 0;

// Function Decleratrions
void updateDisplay();
void initializeGPSData(StructGPSData &data);
void initializeAccData(StructAccData &data);
void initializeDhtData(StructDhtData &data);
void IRAM_ATTR handleDisplayToggle();
void connectToAWS();
void initializeDisplay();
void connectToWifi();
void publishData(const String &topic, const char *payload, size_t len);
void initLora();
void receiveAndProcessDataTask(void *pvParameters);
void sendDataToAWSTask(void *pvParameters);
String base64ToHex(const String &base64Str);

void setup()
{
    Serial.begin(115200);

    initializeDisplay();
    connectToWifi();
    initLora();
    initializeDhtData(lastReceivedDhtData);
    initializeGPSData(lastReceivedGPSData);
    initializeAccData(lastReceivedAccData);

    connectToAWS();

    mqttClient.setBufferSize(1024);
    // Set up button for display toggle
    pinMode(38, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(38), handleDisplayToggle, FALLING);

        // Create a queue to hold up to 10 LoRa messages
    loRaQueue = xQueueCreate(10, sizeof(LoRaMessage));

    // Create tasks
    xTaskCreate(receiveAndProcessDataTask, "Receive and Process Data Task", 4096, NULL, 2, NULL);
    xTaskCreate(sendDataToAWSTask, "Send Data to AWS Task", 8192, NULL, 1, NULL);
}

void loop()
{
    // Reconnect to AWS if connection is lost
    if (!mqttClient.connected())
    {
        connectToAWS();
    }
    mqttClient.loop();

    // Toggle display mode if button pressed
    if (displayToggleRequested)
    {
        displayToggleRequested = false;
        displayMode = !displayMode; // Toggle display mode
        updateDisplay();            // Now safe to update the display here
    }
    

    delay(1000); // Maintain a loop delay for stability
    updateDisplay();
}

String base64ToHex(const String &base64Str) {
    // Create a mutable copy of the base64 input string
    char mutableBase64Str[base64Str.length() + 1];
    base64Str.toCharArray(mutableBase64Str, base64Str.length() + 1);

    // Calculate the required buffer size for decoding
    int decodedLen = Base64.decodedLength(mutableBase64Str, base64Str.length());
    char decoded[decodedLen];

    // Decode base64 string to bytes
    int actualLen = Base64.decode(decoded, mutableBase64Str, base64Str.length());

    // Convert decoded bytes to hexadecimal string
    String hexStr = "";
    for (int i = 0; i < actualLen; i++) {
        if (decoded[i] < 16) {
            hexStr += "0"; // Append leading zero if necessary
        }
        hexStr += String(decoded[i], HEX);
    }

    return hexStr;
}

// Setup Functions
void initializeGPSData(StructGPSData &data)
{
    data.latitude = -999.0;
    data.longitude = -999.0;
}

void initializeDhtData(StructDhtData &data)
{
    data.temp = -999.0;
    data.humidity = -999.0;
}

void initializeAccData(StructAccData &data)
{
    data.accelX = -999.0;
    data.accelY = -999.0;
    data.accelZ = -999.0;
}

void connectToWifi()
{
    // Set up WiFi
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to the WiFi network");
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.channel());
    delay(1000);
}

void initializeDisplay()
{
    Wire.begin();
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("SSD1306 allocation failed");
        while (1)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    updateDisplay();
}

void connectToAWS()
{
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    mqttClient.setServer(AWS_ENDPOINT, aws_port);

    while (!mqttClient.connect(THING_NAME)) {
    Serial.print(".");
    delay(100);
  }
 
  if (!mqttClient.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  Serial.println("AWS IoT Connected!");
}

// Loop Functions
void publishData(const String &topic, const char *payload, size_t len)
{
    Serial.println("Attempting to publish data of size: " + String(len));
    Serial.println("Payload: " + String(payload));
    if (mqttClient.publish(topic.c_str(), payload, len))
    {
        Serial.println("Published to topic: " + topic);
    }
    else
    {
        Serial.println("Publish failed");
        Serial.println("MQTT state: " + String(mqttClient.state()));
    }
}

void updateDisplay()
{
    display.clearDisplay();

    if (!displayMode) // Status display
    {
        display.setCursor(0, 0);
        display.print("WiFi:");
        display.print(WiFi.status() != WL_CONNECTED ? "X" : "V");
        display.setCursor(0, 10);
        display.print("Battery:");
        display.print("%");
        display.setCursor(0, 20);
        display.print("Data Sent:");
        display.print(sentDataType);
        display.setCursor(0, 30);
        display.print("Sent:");
        int printDataSize;
        if (sentDataType == "DHT")
            printDataSize = sizeof(StructDhtData);
        else if (sentDataType == "ACC")
            printDataSize = sizeof(StructAccData);
        else if (sentDataType == "GPS")
            printDataSize = sizeof(StructGPSData);
        else if (sentDataType == "Audio")
            printDataSize = sizeof(audio_packet);
        else
            printDataSize = 0;
        display.print(String(printDataSize) + " bytes");
    }
    else // Sensor data display
    {
        // Display GPS data
        display.setCursor(0, 0);
        display.print("GPS:");
        display.setCursor(0, 10);
        display.print("LAT:");
        display.print(lastReceivedGPSData.latitude == -999.0 ? "null" : String(lastReceivedGPSData.latitude));
        display.setCursor(0, 20);
        display.print("LON:");
        display.print(lastReceivedGPSData.longitude == -999.0 ? "null" : String(lastReceivedGPSData.longitude));

        // Display Sensor data
        display.setCursor(64, 0);
        display.print("TEMP:");
        display.print(lastReceivedDhtData.temp == -999.0 ? "0C" : String((int)lastReceivedDhtData.temp) + "C");
        display.setCursor(64, 10);
        display.print("HUMI:");
        display.print(lastReceivedDhtData.humidity == -999.0 ? "0%" : String((int)lastReceivedDhtData.humidity) + "%");
        display.setCursor(64, 20);
        display.print("ACC:");
        display.setCursor(64, 30);
        display.print("X:");
        display.print(lastReceivedAccData.accelX == -999.0 ? "0" : String(lastReceivedAccData.accelX));
        display.setCursor(64, 40);
        display.print("Y:");
        display.print(lastReceivedAccData.accelY == -999.0 ? "0" : String(lastReceivedAccData.accelY));
        display.setCursor(64, 50);
        display.print("Z:");
        display.print(lastReceivedAccData.accelZ == -999.0 ? "0" : String(lastReceivedAccData.accelZ));
    }

    display.display(); // Commit changes to display
}

void IRAM_ATTR handleDisplayToggle()
{
    displayMode = !displayMode; // Toggle display mode
}

void initLora(){
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(433E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
}

void receiveAndProcessDataTask(void *pvParameters) {
    while (true) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            Serial.println("Received packet of size " + String(packetSize));
            LoRaMessage message;
            message.size = packetSize;
            LoRa.readBytes(message.data, packetSize);

            if (xQueueSend(loRaQueue, &message, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send to queue");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to allow other tasks to run
    }
}

void sendDataToAWSTask(void *pvParameters)
{
    int64_t firstAudioPacketTime = 0;
    while (true)
    {
        audio_packet *receivedPacket = nullptr;

        LoRaMessage message;
        if (xQueueReceive(loRaQueue, &message, portMAX_DELAY) == pdPASS)
        {
            if (message.size == sizeof(StructDhtData) && ((StructDhtData *)message.data)->type == "DHT")
            {
                memcpy(&lastReceivedDhtData, message.data, sizeof(StructDhtData));
                char payload[200];
                StaticJsonDocument<200> doc;
                doc["temp"] = lastReceivedDhtData.temp;
                doc["humidity"] = lastReceivedDhtData.humidity;
                serializeJson(doc, payload);
                publishData(topic_temp_humidity, payload, strlen(payload));
                Serial.println("Temp:" + String(lastReceivedDhtData.temp));
                Serial.println("Humidity:" + String(lastReceivedDhtData.humidity));
                sentDataType = "DHT";
            }
            else if (message.size == sizeof(StructAccData) && ((StructAccData *)message.data)->type == "ACC")
            {
                memcpy(&lastReceivedAccData, message.data, sizeof(StructAccData));
                Serial.println("Received Acc data: X:" + String(lastReceivedAccData.accelX) + ", Y:" + String(lastReceivedAccData.accelY) + ", Z:" + String(lastReceivedAccData.accelZ) + "\n");
                char payload[200];
                StaticJsonDocument<200> doc;
                doc["accelX"] = lastReceivedAccData.accelX;
                doc["accelY"] = lastReceivedAccData.accelY;
                doc["accelZ"] = lastReceivedAccData.accelZ;
                serializeJson(doc, payload);
                publishData(topic_accelerometer, payload, strlen(payload));
                Serial.println("AccX:" + String(lastReceivedAccData.accelX));
                Serial.println("AccY:" + String(lastReceivedAccData.accelY));
                Serial.println("AccZ:" + String(lastReceivedAccData.accelZ));
                sentDataType = "ACC";
            }
            else if (message.size == sizeof(StructGPSData) && ((StructGPSData *)message.data)->type == "GPS")
            {
                memcpy(&lastReceivedGPSData, message.data, sizeof(StructGPSData));
                char payload[200];
                StaticJsonDocument<200> doc;
                doc["latitude"] = lastReceivedGPSData.latitude;
                doc["longitude"] = lastReceivedGPSData.longitude;
                serializeJson(doc, payload);
                publishData(topic_gps, payload, strlen(payload));
                Serial.println("Lat:" + String(lastReceivedGPSData.latitude));
                Serial.println("Lon:" + String(lastReceivedGPSData.longitude));
                sentDataType = "GPS";
            }
            else if (strncmp(((audio_packet *)message.data)->type, "AUD", 3) == 0)
            {
                memcpy(&incomingAudioData, message.data, sizeof(audio_packet));
                Serial.println("incoming data size for test: " + String(sizeof(incomingAudioData)));
                
                DynamicJsonDocument doc(1000);
                incomingAudioData.audioData[incomingAudioData.audioSize] = '\0';
                for (int i = 0; i < incomingAudioData.audioSize; i++) {
                    Serial.print(incomingAudioData.audioData[i], HEX);
                    Serial.print(" ");
                }

                String audioDataHex = base64ToHex(incomingAudioData.audioData);
                receivedPacket = (audio_packet *)message.data;
                doc["audioSize"] = incomingAudioData.audioSize;
                doc["packetNumber"] = incomingAudioData.packetNumber;
                doc["isFinalPacket"] = incomingAudioData.isFinalPacket;
                doc["startTime"] = incomingAudioData.startTime;
                doc["audioData"] = base64ToHex(incomingAudioData.audioData);
                Serial.println(sizeof(doc["audioData"]));
                if(doc.overflowed()){
                    Serial.println("doc overflow");
                }
                size_t payloadSize = measureJson(doc) + 1;
                char *payload = (char *)malloc(payloadSize);
                if(!payload){
                    Serial.println("Failed to allocate memory for payload");
                    continue;
                }
                Serial.println("Payload size: " + String(payloadSize));
                serializeJson(doc, payload, payloadSize);
                Serial.println("Payload: " + String(payload));
                publishData(topic_audio, payload, strlen(payload));

                free(payload);
            }
            else
            {
                Serial.println("Invalid packet size: " + String(message.size));
            }
            lastSendStatus = true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay between publishes
    }
}