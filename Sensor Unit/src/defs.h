#include <WiFi.h>

// Definitions for MPU6050 and DHT
#define DHTPIN 18
#define DHTTYPE DHT11

// Definitions for I2S
#define I2S_NUM I2S_NUM_0
#define I2S_SAMPLE_RATE 11025
#define I2S_BUFFER_LEN 164
#define AUDIO_DATA_SIZE 219
#define FINAL_AUDIO_PACKET_SIZE 24

// Microphone setup
#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14

// Sound detector setup
#define SOUND_DETECT_PIN 33 // ADC pin connected to the KY-037
#define SOUND_DETECT_VOLT 5.0 // Voltage supplied to KY-037
#define MAX_ADC_VALUE 4095  // Max ADC value for ESP32 (12-bit ADC)
#define RECORDING_LED_PIN 5 // LED pin to indicate recording

// Data structures
typedef struct
{
    float accelX;
    float accelY;
    float accelZ;
    String type;
} acc_message;

typedef struct
{
    float temp;
    float humidity;
    String type;
} dht_message;

typedef struct {
    char audioData[AUDIO_DATA_SIZE];
    uint16_t audioSize;
    char type[3];
    uint32_t packetNumber;
    int64_t startTime;
    bool isFinalPacket;
} audio_packet;

