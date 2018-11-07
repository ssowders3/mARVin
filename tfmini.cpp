#include "tfmini.h"

TFMini::TFMini(Serial* _connection) {
    connection = _connection;
    
    distance = -1;
    strength = -1;
    state = READY;
    
    setStandardOutputMode();
    
}

void TFMini::setStandardOutputMode() {
    connection->putc((uint8_t)0x42);
    connection->putc((uint8_t)0x57);
    connection->putc((uint8_t)0x02); 
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x01);
    connection->putc((uint8_t)0x06);
}

void TFMini::setConfigMode() {
  // advanced parameter configuration mode
    connection->putc((uint8_t)0x42);
    connection->putc((uint8_t)0x57);
    connection->putc((uint8_t)0x02);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x01);
    connection->putc((uint8_t)0x02);
}

void TFMini::externalTrigger() {
    setConfigMode();
    
    connection->putc((uint8_t)0x42);
    connection->putc((uint8_t)0x57);
    connection->putc((uint8_t)0x02);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x41);
}

int TFMini::takeMeasurement() {
    int numCharsRead = 0;
    uint8_t lastChar = 0x00;
    
    while(1) {
        if (connection->readable()) {
            uint8_t curChar = connection->getc();
            
            if ((curChar == 0x59) && (lastChar == 0x59)) {
                break;
            } else {
                lastChar = curChar;
                numCharsRead += 1;
            }
            
        }
        
        if (numCharsRead > TFMINI_MAXBYTESBEFOREHEADER) {
            state = ERROR_SERIAL_NOHEADER;
            distance = -1;
            strength = -1;     
            if (TFMINI_DEBUGMODE == 1)
                printf("ERROR: no header\n");
            return -1;
        }
    }
    
    uint8_t frame[TFMINI_FRAME_SIZE];
    
    uint8_t checksum = 0x59 + 0x59;
    for (int i=0; i<TFMINI_FRAME_SIZE; i++) {
    
        while (!connection->readable()) {}    
        frame[i] = connection->getc();

        if (i < TFMINI_FRAME_SIZE-2) {
            checksum += frame[i];
        }
    }
    
    uint8_t checksumByte = frame[TFMINI_FRAME_SIZE-1];
    if (checksum != checksumByte) {
        state = ERROR_SERIAL_BADCHECKSUM;
        distance = -1;
        strength = -1;
        if (TFMINI_DEBUGMODE == 1) 
            printf("ERROR: bad checksum\n");
        return -1;
    }
    
    uint16_t dist = (frame[1] << 8) + frame[0];
    uint16_t st = (frame[3] << 8) + frame[2];
    uint8_t reserved = frame[4];
    uint8_t originalSignalQuality = frame[5];
    
    distance = dist;
    strength = st;
    state = MEASUREMENT_OK;

    // Return success
    return 0; 
}

void TFMini::setSingleScanMode() {
    setConfigMode();
    connection->putc((uint8_t)0x42);
    connection->putc((uint8_t)0x57);
    connection->putc((uint8_t)0x02);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x00);
    connection->putc((uint8_t)0x40);
}

uint16_t TFMini::getDistance() {
    int numMeasurementAttempts = 0;
    while (takeMeasurement() != 0) {
        numMeasurementAttempts += 1;
        if (numMeasurementAttempts > TFMINI_MAX_MEASUREMENT_ATTEMPTS) {
            printf ("TF Mini error: too many measurement attempts");
            printf ("Last error:");
            if (state == ERROR_SERIAL_NOHEADER)     printf("ERROR_SERIAL_NOHEADER");
            if (state == ERROR_SERIAL_BADCHECKSUM)  printf("ERROR_SERIAL_BADCHECKSUM");
            if (state == ERROR_SERIAL_TOOMANYTRIES) printf("ERROR_SERIAL_TOOMANYTRIES");      
      
            state = ERROR_SERIAL_TOOMANYTRIES;
            distance = -1;
            strength = -1;
            return -1;      
        }
    }
    
    if (state == MEASUREMENT_OK) {
        return distance;
    } else {
        return -1;
    }
}

uint16_t TFMini::getRecentSignalStrength() {
    return strength;
}
