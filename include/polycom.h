
#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiUdp.h>

#define POLYCOM_CHANNEL(x) (25+x)

uint8_t pcmu_encode(int16_t pcm);
void sendStartPackets(uint8_t channel);
void sendEndPacket(uint8_t channel);
void sendAudioPacket(uint8_t* prev, uint8_t* curr, bool first, uint8_t channel);
bool readFrame(uint8_t* ulawBuf);
void streamWav(uint8_t channel);
void broadcastAudio(uint8_t channel);
void populateSerial();
void polycomSetup();

