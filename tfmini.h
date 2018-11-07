#include "mbed.h"

#define TFMINI_BAUDRATE 115200
#define TFMINI_DEBUGMODE 0

#define TFMINI_FRAME_SIZE 7

#define TFMINI_MAXBYTESBEFOREHEADER 30
#define TFMINI_MAX_MEASUREMENT_ATTEMPTS 10

#define READY                             0
#define ERROR_SERIAL_NOHEADER             1
#define ERROR_SERIAL_BADCHECKSUM          2
#define ERROR_SERIAL_TOOMANYTRIES         3
#define MEASUREMENT_OK 10

class TFMini {
    public:
        TFMini(Serial* _connection);
        void setSingleScanMode();
        uint16_t getDistance();
        uint16_t getRecentSignalStrength();
        void externalTrigger();

    private:
        Serial* connection;
        int state;
        uint16_t distance;
        uint16_t strength;
        
        void setStandardOutputMode();
        void setConfigMode();
        int takeMeasurement();    
};