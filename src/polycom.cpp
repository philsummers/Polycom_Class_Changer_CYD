#include "polycom.h"



IPAddress multicastIP(224,0,1,116);
const int port = 5001;

uint32_t serial = 0x00010203;
char caller[13] = "CLASS-CHANGE";

const int FRAME_SAMPLES = 160;   // 20ms at 8kHz
uint8_t prevFrame[FRAME_SAMPLES];
uint8_t currFrame[FRAME_SAMPLES];

uint32_t rtpTimestamp = 0;

File wavFile;

WiFiUDP udp;

uint8_t pcmu_encode(int16_t pcm)
{
    const uint16_t BIAS = 0x84;
    const uint16_t CLIP = 32635;

    int sign = (pcm >> 8) & 0x80;

    if (sign != 0)
        pcm = -pcm;

    if (pcm > CLIP)
        pcm = CLIP;

    pcm += BIAS;

    int exponent = 7;
    for (int expMask = 0x4000; (pcm & expMask) == 0 && exponent > 0; exponent--, expMask >>= 1);

    int mantissa = (pcm >> ((exponent == 0) ? 4 : (exponent + 3))) & 0x0F;

    uint8_t ulaw = ~(sign | (exponent << 4) | mantissa);

    return ulaw;
}

void sendStartPackets(uint8_t channel)
{
    uint8_t pkt[20];
    memset(pkt,0,sizeof(pkt));

    pkt[0] = 0x0F;
    pkt[1] = channel;

    pkt[2] = (serial>>24)&0xff;
    pkt[3] = (serial>>16)&0xff;
    pkt[4] = (serial>>8)&0xff;
    pkt[5] = serial&0xff;

    pkt[6] = 0x0D;

    memcpy(pkt+7, caller, strlen(caller));

    for(int i=0;i<32;i++)
    {
        udp.beginPacket(multicastIP,port);
        udp.write(pkt,sizeof(pkt));
        udp.endPacket();
        delay(5);
    }

    //logger("Start packets sent");
}

void sendEndPacket(uint8_t channel)
{
    uint8_t pkt[20];
    memset(pkt,0,sizeof(pkt));

    pkt[0] = 0xFF;
    pkt[1] = channel;

    pkt[2] = (serial>>24)&0xff;
    pkt[3] = (serial>>16)&0xff;
    pkt[4] = (serial>>8)&0xff;
    pkt[5] = serial&0xff;

    pkt[6] = 0x0D;

    memcpy(pkt+7, caller, strlen(caller));

    for(int i=0;i<12;i++) {
        udp.beginPacket(multicastIP,port);
        udp.write(pkt,sizeof(pkt));
        udp.endPacket();
        delay(5);
    }

    //logger("End packets sent");
}

void sendAudioPacket(uint8_t* prev, uint8_t* curr, bool first, uint8_t channel)
{
    uint8_t packet[512];
    int idx = 0;

    packet[idx++] = 0x10;
    packet[idx++] = channel;

    packet[idx++] = (serial>>24)&0xff;
    packet[idx++] = (serial>>16)&0xff;
    packet[idx++] = (serial>>8)&0xff;
    packet[idx++] = serial&0xff;

    packet[idx++] = 0x0D;

    memcpy(packet+idx,caller,13);
    idx += 13;

    packet[idx++] = 0x00;
    packet[idx++] = 0x00;

    packet[idx++] = (rtpTimestamp>>24)&0xff;
    packet[idx++] = (rtpTimestamp>>16)&0xff;
    packet[idx++] = (rtpTimestamp>>8)&0xff;
    packet[idx++] = rtpTimestamp&0xff;

    if(!first)
    {
        memcpy(packet+idx, prev, FRAME_SAMPLES);
        idx += FRAME_SAMPLES;
    }

    memcpy(packet+idx, curr, FRAME_SAMPLES);
    idx += FRAME_SAMPLES;

    udp.beginPacket(multicastIP,port);
    udp.write(packet,idx);
    udp.endPacket();
}

bool readFrame(uint8_t* ulawBuf)
{
    int16_t pcm[FRAME_SAMPLES];

    int bytes = wavFile.read((uint8_t*)pcm, FRAME_SAMPLES*2);

    if(bytes < FRAME_SAMPLES*2)
        return false;

    for(int i=0;i<FRAME_SAMPLES;i++)
        ulawBuf[i] = pcmu_encode(pcm[i]);

    return true;
}

void streamWav(uint8_t channel)
{
    bool first=true;

    while(readFrame(currFrame))
    {
        sendAudioPacket(prevFrame,currFrame,first, channel);

        memcpy(prevFrame,currFrame,FRAME_SAMPLES);

        rtpTimestamp += FRAME_SAMPLES;

        first=false;

        delay(20); // 20ms frame
    }
}

void broadcastAudio(uint8_t channel) {
    
    wavFile.seek(44); // skip WAV header

    sendStartPackets(channel);

    streamWav(channel);

    sendEndPacket(channel);

}

void populateSerial() {
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);

    //logger("Setting Client Serial Number");

    if(ret == ESP_OK) {
        serial = (baseMac[2]<<24)+(baseMac[3]<<16)+(baseMac[4]<<8)+baseMac[5];
        Serial.printf("%X\n",serial);
        //WebSerial.printf("Setting client ID to %X\n",serial);

    } else {
        Serial.println("Failed to read MAC address");
        //WebSerial.println("Failed to read MAC address, using defaults");
    }
}

void polycomSetup() {
    populateSerial();

    udp.begin(port);
    if(!LittleFS.begin(true)) {
        Serial.println("LittleFS failed");        
        return;
    }

    wavFile = LittleFS.open("/pager-bell.wav","r");

    if(!wavFile) {
        Serial.println("WAV missing");        
        return;
    }
}
