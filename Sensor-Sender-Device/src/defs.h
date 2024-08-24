// Display pins and settings
#define OLED_RESET -1
#define SCREEN_SDA 21
#define SCREEN_SCL 22
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// LoRa pins
#define RADIO_CS_PIN 18
#define RADIO_RST_PIN 23
#define RADIO_DIO0_PIN 26

// Audio Buffer Size
#define AUDIO_DATA_SIZE 219

struct __attribute__((packed)) struct_acc_message_received
{
    float accelX;
    float accelY;
    float accelZ;
    String type;
};

struct __attribute__((packed)) struct_dht_message_received
{
    float temp;
    float humidity;
    String type;
};

struct __attribute__((packed)) struct_gps_message
{
    float latitude;
    float longitude;
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