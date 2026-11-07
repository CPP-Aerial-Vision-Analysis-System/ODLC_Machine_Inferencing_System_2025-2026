#ifndef ZR10_H
#define ZR10_H

#include <cstdint>
#include <string>
#include <vector>

class ZR10
{
public:
    ZR10(
        const std::string& ip = "192.168.144.25",
        int port = 37260
    );

    ~ZR10();

    bool takePhoto();
    bool autoFocus(uint16_t x = 960, uint16_t y = 540);
    bool manualFocus(int8_t direction);
    bool setZoom(double zoom);

    bool zoomIn();
    bool zoomOut();
    bool stopZoom();

    bool toggleRecording();

    bool moveGimbal(
        int8_t yaw,
        int8_t pitch
    );

    bool stopGimbal();
    bool centerGimbal();

private:
    std::string cameraIP;
    int cameraPort;
    int socketFd;

    uint16_t sequence;

    bool sendCommand(
        uint8_t commandID,
        const std::vector<uint8_t>& data
    );

    std::vector<uint8_t> makePacket(
        uint8_t commandID,
        const std::vector<uint8_t>& data
    );

    uint16_t calculateCRC(
        const uint8_t* data,
        std::size_t length
    );
};

#endif