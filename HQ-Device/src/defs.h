#include <WiFi.h>

// Audio settings
#define AUDIO_DATA_SIZE 219

// OLED display settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

// LoRa pins
#define RADIO_CS_PIN 18
#define RADIO_RST_PIN 23
#define RADIO_DIO0_PIN 26

// Structs
struct __attribute__((packed)) StructDhtData
{
    float temp;
    float humidity;
    String type;
};

struct __attribute__((packed)) StructAccData
{
    float accelX;
    float accelY;
    float accelZ;
    String type;
};

struct __attribute__((packed)) StructGPSData
{
    float latitude;
    float longitude; // Corrected spelling
    String type;
};

typedef struct {
    char audioData[AUDIO_DATA_SIZE];
    uint16_t audioSize;
    char type[3];
    uint32_t packetNumber;
    int64_t startTime;
    bool isFinalPacket;
} audio_packet;

// Structures for Queue
struct LoRaMessage {
    uint8_t data[sizeof(audio_packet)];
    size_t size;
};