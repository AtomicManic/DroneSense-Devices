#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/i2s.h>
#include <time.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "defs.h"
#include <Base64.h>

// Network variables
const uint8_t relayMAC[] = {0x8, 0xF9, 0xE0, 0xF7, 0x4E, 0xD0};

// Sensors variables
Adafruit_MPU6050 mpu;
DHT dht(DHTPIN, DHTTYPE);

acc_message lastAccData = {0, 0, 0};         // Last sent accelerometer data
float accCalX = 0, accCalY = 0, accCalZ = 0; // Calibration offsets
const float accThreshold = 0.2;              // Change threshold for sending data
bool isRecording = false;

// Queue Handles
QueueHandle_t accDataQueue;
QueueHandle_t tempDataQueue;
QueueHandle_t audioDataQueue;

// Task Declarations
void accTask(void *parameter);
void tempTask(void *parameter);
void microphoneTask(void *parameter);
void senderTask(void *parameter);

TaskHandle_t microphoneTaskHandler = NULL;

// Function Declarations
void InitEspNow();
void I2CInit();
void initSensors();
void sendData(const uint8_t *data, size_t len, const uint8_t *peer_addr);
void calibrateAccelerometer();

void setup()
{
    Serial.begin(115200);
    InitEspNow();
    initSensors();
    I2CInit();
    analogReadResolution(12);
    pinMode(SOUND_DETECT_PIN, INPUT);
    pinMode(RECORDING_LED_PIN, OUTPUT);

    // Queue creation
    accDataQueue = xQueueCreate(10, sizeof(acc_message));
    tempDataQueue = xQueueCreate(10, sizeof(dht_message));
    audioDataQueue = xQueueCreate(20, sizeof(audio_packet));

    // Task creation
    xTaskCreate(accTask, "ACC Task", 2048, NULL, 1, NULL);
    xTaskCreate(tempTask, "TEMP Task", 2048, NULL, 1, NULL);
    xTaskCreatePinnedToCore(microphoneTask, "Microphone Task", 8192, NULL, 1, &microphoneTaskHandler, 0);
    xTaskCreate(senderTask, "Sender Task", 4096, NULL, 2, NULL);
}

void loop()
{
    // if (digitalRead(SOUND_DETECT_PIN) == HIGH && !isRecording)
    // {
    //     isRecording = true;
    //     xTaskNotifyGive(microphoneTaskHandler);
    //     Serial.println("Sound detected, starting recording.");
    // }
    delay(100);
}

// Tasks
void accTask(void *parameter)
{
    while (true)
    {
        acc_message data;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Apply calibration offsets
        data.accelX = a.acceleration.x - accCalX;
        data.accelY = a.acceleration.y - accCalY;
        data.accelZ = a.acceleration.z - accCalZ;
        data.type = "ACC";

        if (abs(data.accelX - lastAccData.accelX) > accThreshold ||
            abs(data.accelY - lastAccData.accelY) > accThreshold ||
            abs(data.accelZ - lastAccData.accelZ) > accThreshold)
        {
            Serial.println("Sending ACC data: X-" + String(data.accelX) + " Y-" + String(data.accelY) + " Z-" + String(data.accelZ));
            if (xQueueSend(accDataQueue, &data, portMAX_DELAY) != pdPASS)
            {
                Serial.println("Failed to enqueue ACC sensor data");
            }
            // Update last sent data
            lastAccData = data;
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Sampling rate for sensor data
    }
}

void tempTask(void *parameter)
{
    while (true)
    {
        dht_message data;
        data.temp = dht.readTemperature();
        data.humidity = dht.readHumidity();
        data.type = "DHT";

        if (xQueueSend(tempDataQueue, &data, portMAX_DELAY) != pdPASS)
        {
            Serial.println("Failed to enqueue TEMP sensor data");
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Sampling rate for sensor data
    }
}

void microphoneTask(void *parameter)
{
    uint8_t buffer[I2S_BUFFER_LEN];
    size_t bufferIndex = 0;
    size_t bytesRead;
    uint32_t packetNumber = 0;
    int64_t startTime = 0;
    bool dataBeingCaptured = false; // Flag to track data capture status

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        startTime = esp_timer_get_time();
        dataBeingCaptured = false; // Reset data capture flag at the start of each recording session
        Serial.println("Recording session started...");

        while ((esp_timer_get_time() - startTime) < 10000000L)
        { // Assuming 20 seconds of recording
            if (i2s_read(I2S_NUM, buffer + bufferIndex, sizeof(buffer) - bufferIndex, &bytesRead, portMAX_DELAY) == ESP_OK && bytesRead > 0)
            {
                if (!dataBeingCaptured)
                {
                    digitalWrite(RECORDING_LED_PIN, LOW); // Turn on LED when data starts being captured
                    dataBeingCaptured = true;
                }
                bufferIndex += bytesRead;

                while (bufferIndex >= I2S_BUFFER_LEN)
                { // Ensure we only process chunks that fit into 240 bytes after Base64 encoding
                    audio_packet packet;
                    int base64Length = Base64.encode(packet.audioData, (char *)buffer, I2S_BUFFER_LEN);
                    packet.audioData[base64Length] = 0; // Null-terminate the Base64 string
                    packet.audioSize = base64Length;
                    strncpy(packet.type, "AUD", sizeof(packet.type));
                    packet.packetNumber = packetNumber++;
                    packet.startTime = startTime / 1000000; // Convert microseconds to seconds for simplicity
                    packet.isFinalPacket = false;
                    Serial.println(packet.audioData);
                    xQueueSend(audioDataQueue, &packet, portMAX_DELAY);
                    Serial.printf("Packet #%u encoded and queued for sending. Size: %d bytes\n", packet.packetNumber, base64Length);

                    bufferIndex -= I2S_BUFFER_LEN;
                    memmove(buffer, buffer + I2S_BUFFER_LEN, bufferIndex);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Handle any remaining data after the main recording loop
        if (bufferIndex > 0)
        {
            audio_packet finalPacket;
            int finalBase64Length = Base64.encode(finalPacket.audioData, (char *)buffer, bufferIndex);
            finalPacket.audioData[finalBase64Length] = 0; // Null-terminate the Base64 string
            finalPacket.audioSize = finalBase64Length;
            strncpy(finalPacket.type, "AUD", sizeof(finalPacket.type));
            finalPacket.packetNumber = packetNumber++;
            finalPacket.startTime = startTime / 1000000; // Convert microseconds to seconds for simplicity
            finalPacket.isFinalPacket = true;

            xQueueSend(audioDataQueue, &finalPacket, portMAX_DELAY);
            Serial.printf("Final packet #%u encoded and queued for sending. Size: %d bytes\n", finalPacket.packetNumber, finalBase64Length);
        }
        isRecording = false;

        digitalWrite(RECORDING_LED_PIN, HIGH); // Turn off LED after recording session
        Serial.println("Recording session ended.");

        packetNumber = 0; // Reset packet number for the next session
        bufferIndex = 0;  // Reset buffer index
    }
}

void senderTask(void *parameter)
{
    dht_message dht_data;
    acc_message acc_data;
    audio_packet audio_data;
    esp_err_t result;
    while (true)
    {
        if (xQueueReceive(tempDataQueue, &dht_data, 0) == pdPASS)
        {
            Serial.println("Sending DHT data: temp-" + String(dht_data.temp) + " humid-" + String(dht_data.humidity));

            sendData((uint8_t *)&dht_data, sizeof(dht_message), relayMAC);

            delay(1000); // Delay to prevent packet loss
        }
        if (xQueueReceive(accDataQueue, &acc_data, 0) == pdPASS)
        {
            Serial.println("Sending ACC data: X-" + String(acc_data.accelX) + " Y-" + String(acc_data.accelY) + " Z-" + String(acc_data.accelZ));
            sendData((uint8_t *)&acc_data, sizeof(acc_message), relayMAC);
            delay(1000); // Delay to prevent packet loss
        }
        if (xQueueReceive(audioDataQueue, &audio_data, 0) == pdPASS)
        {
            Serial.println("Sending Audio data: " + String(audio_data.audioSize) + " bytes");
            sendData((uint8_t *)&audio_data, sizeof(audio_packet), relayMAC);
            delay(1000); // Delay to prevent packet loss
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Init
void InitEspNow()
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        while (true)
            ;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, relayMAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Error adding peer");
        while (true)
            ;
    }
}

void I2CInit()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0};
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD};
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
}

void initSensors()
{
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
            delay(1000);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    dht.begin();
}

void sendData(const uint8_t *data, size_t len, const uint8_t *peer_addr)
{
    esp_err_t result = esp_now_send(peer_addr, data, len);
    if (result == ESP_OK)
    {
        Serial.println("Sent with success");
    }
    else
    {
        Serial.println("Error sending the data");
    }
}

void calibrateAccelerometer()
{
    const int numReadings = 10;
    sensors_event_t a, g, temp;
    accCalX = 0, accCalY = 0, accCalZ = 0; // Reset calibration offsets

    for (int i = 0; i < numReadings; i++)
    {
        mpu.getEvent(&a, &g, &temp);
        accCalX += a.acceleration.x;
        accCalY += a.acceleration.y;
        accCalZ += a.acceleration.z;
        delay(100); // Short delay between readings to not overload the sensor bus
    }
    accCalX /= numReadings;
    accCalY /= numReadings;
    accCalZ /= numReadings;
}
