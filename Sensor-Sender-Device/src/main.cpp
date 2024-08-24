#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"
#include <TinyGPS++.h>
#include <WiFi.h>
#include "defs.h"
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Display settings
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Data ready flags
volatile bool gpsFlag = false;
volatile bool accFlag = false;
volatile bool dhtFlag = false;
volatile bool audioFlag = false;

// Data to send
struct_gps_message readyGpsData;
struct_dht_message_received readyDhtData;
struct_acc_message_received readyAccData;
audio_packet readyAudioData;

// Display and variables
bool lastSendStatus = false;
String sentDataType = "";

// GPS and Serial variables
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// Task handlers
TaskHandle_t GPSTaskHandle;
TaskHandle_t SendDataTaskHandle;
TaskHandle_t DisplayTaskHandle;

// Function Declerations
void sendData(uint8_t *data, size_t len);
void updateDisplay();
void initDisplay();
void initNetwork();
void initLora();
void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int len);

// Task Declarations
void GPSTask(void *parameter);
void SendDataTask(void *parameter);
void DisplayTask(void *parameter);

// Queue Handles
QueueHandle_t gpsQueue;
QueueHandle_t dhtQueue;
QueueHandle_t accQueue;
QueueHandle_t audioQueue;



void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for the serial port to connect.
    initDisplay();
    SerialGPS.begin(9600, SERIAL_8N1, 34, -1);
    initNetwork();
    initLora();

    // Create queues
    gpsQueue = xQueueCreate(10, sizeof(struct_gps_message));
    dhtQueue = xQueueCreate(10, sizeof(struct_dht_message_received));
    accQueue = xQueueCreate(10, sizeof(struct_acc_message_received));
    audioQueue = xQueueCreate(20, sizeof(audio_packet));

    // Task creation
    xTaskCreate(GPSTask, "GPS Handling Task", 2048, NULL, 2, &GPSTaskHandle);
    xTaskCreate(SendDataTask, "Data Sending Task", 8192, NULL, 3, &SendDataTaskHandle);
    xTaskCreate(DisplayTask, "Display Update Task", 2048, NULL, 1, &DisplayTaskHandle);
}

void loop()
{
    // Empty loop
}

// Tasks
void GPSTask(void *parameter) {
    struct_gps_message gpsData;
    double lastLat = 0, lastLon = 0;
    while (true) {
        while (SerialGPS.available() > 0) {
            if (gps.encode(SerialGPS.read())) {
                if (gps.location.isValid() && (round(gps.location.lat() * 1000.0) / 1000.0 != lastLat || round(gps.location.lng() * 1000.0) / 1000.0 != lastLon)) {
                    lastLat = round(gps.location.lat() * 1000.0) / 1000.0;
                    lastLon = round(gps.location.lng() * 1000.0) / 1000.0;

                    gpsData.latitude = gps.location.lat();
                    gpsData.longitude = gps.location.lng();
                    gpsData.type = "GPS";
                    xQueueSend(gpsQueue, &gpsData, portMAX_DELAY);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void DisplayTask(void *parameter) {
    while (true) {
        updateDisplay();
        vTaskDelay(pdMS_TO_TICKS(500)); // Refresh every 500ms
    }
}

void SendDataTask(void *parameter) {
    struct_gps_message gpsData;
    struct_dht_message_received dhtData;
    struct_acc_message_received accData;
    audio_packet audioData;
    while (true) {
        if (xQueueReceive(gpsQueue, &gpsData, 0) == pdPASS) {
            sendData((uint8_t *)&gpsData, sizeof(gpsData));
        }
        if (xQueueReceive(dhtQueue, &dhtData, 0) == pdPASS) {
            sendData((uint8_t *)&dhtData, sizeof(dhtData));
        }
        if (xQueueReceive(accQueue, &accData, 0) == pdPASS) {
            sendData((uint8_t *)&accData, sizeof(accData));
        }
        if (xQueueReceive(audioQueue, &audioData, 0) == pdPASS) {
            sendData((uint8_t *)&audioData, sizeof(audioData));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Functions
void updateDisplay()
{
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(WHITE); // Draw white text

    // GPS data headers and values (Top Left)
    display.setCursor(5, 0);
    display.print("GPS:");
    display.setCursor(5, 10);
    display.print("LAT:");
    display.print(gps.location.isValid() ? String(gps.location.lat(), 3) : "null");
    display.setCursor(5, 20);
    display.print("LON:");
    display.print(gps.location.isValid() ? String(gps.location.lng(), 3) : "null");
    display.setCursor(5, 30);
    display.print("MIC:");
    display.print(sentDataType == "Audio" ? "V" : "X");

    // Temperature and humidity data headers and values (Top Right)
    display.setCursor(75, 0); // Adjust the cursor to the top right
    display.print("TEMP:");
    display.print(readyDhtData.temp ? String((int)readyDhtData.temp) + "C" : "null");
    display.setCursor(75, 10);
    display.print("HUMI:");
    display.print(readyDhtData.humidity ? String((int)readyDhtData.humidity) + "%" : "null");

    // Send status and type (Bottom Left)
    display.setCursor(5, 45); // Move cursor to bottom left
    display.print("SENT:");
    display.print(lastSendStatus ? "V" : "X");
    display.setCursor(5, 55);
    display.print("TYPE:");
    display.print(sentDataType);

    // Accelerometer data headers and values (Bottom Right)
    display.setCursor(75, 25); // Move cursor to bottom right
    display.print("ACC:");
    display.setCursor(75, 35);
    display.print("X:");
    display.print(readyAccData.accelX ? String(readyAccData.accelX) : "0");
    display.setCursor(75, 45);
    display.print("Y:");
    display.print(readyAccData.accelY ? String(readyAccData.accelY) : "0");
    display.setCursor(75, 55);
    display.print("Z:");
    display.print(readyAccData.accelZ ? String(readyAccData.accelZ) : "0");

    display.display(); // Actually draw everything on the display
}

void sendData(uint8_t *data, size_t len)
{
    LoRa.beginPacket();
    LoRa.write(data, len);
    LoRa.endPacket();
}

void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len == sizeof(struct_dht_message_received) && ((struct_dht_message_received*)data)->type == "DHT") {
        Serial.println("Received DHT data");
        memcpy(&readyDhtData, data, sizeof(struct_dht_message_received));
        xQueueSend(dhtQueue, &readyDhtData, portMAX_DELAY);
    }
    else if (len == sizeof(struct_acc_message_received) && ((struct_acc_message_received*)data)->type == "ACC") {
        Serial.println("Received ACC data");
        memcpy(&readyAccData, data, sizeof(struct_acc_message_received));
        xQueueSend(accQueue, &readyAccData, portMAX_DELAY);
    }
    else if (len == sizeof(audio_packet) || ((audio_packet*)data)->type == "AUDIO") {
        memcpy(&readyAudioData, data, sizeof(audio_packet));
        xQueueSend(audioQueue, &readyAudioData, portMAX_DELAY);
        Serial.println("Received audio data of " + String(readyAudioData.audioSize) + " bytes, packet number: " + String(readyAudioData.packetNumber));
        if(((audio_packet*)data)->isFinalPacket){
            Serial.println("Final packet received");
        }
    }
    else {
        Serial.println("Received unknown data type");
    }
}

// Init Functions
void initDisplay(){
    Wire.begin(SCREEN_SDA, SCREEN_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        while (1)
            ;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    Serial.println("Display initialized.");
}

void initNetwork(){
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onDataReceive);
}

void initLora(){
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(433E6)) {  // Change the frequency as per your region
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

