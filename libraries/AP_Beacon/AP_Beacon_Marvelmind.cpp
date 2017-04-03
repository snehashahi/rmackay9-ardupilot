/*
 * AP_Beacon_Marvelmind.cpp
 *
 *  Created on: 21.03.2017
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctype.h>

#include <AP_HAL/AP_HAL.h>

#include "AP_Beacon_Marvelmind.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_Beacon_Marvelmind::AP_Beacon_Marvelmind(AP_Beacon &frontend,
        AP_SerialManager &serial_manager) :
        AP_Beacon_Backend(frontend) {
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon,
            0);
    if (uart != nullptr) {
        uart->begin(
                serial_manager.find_baudrate(
                        AP_SerialManager::SerialProtocol_Beacon, 0));

    //variables for the receive thread
        hedge = new MarvelmindHedge();
        if(hedge)
        {
            createMarvelmindHedge();
            Parse_State = RECV_HDR; // current state of receive data
            nBytesInBlockReceived = 0; // bytes received
            dataId = 0;
            startMarvelmindHedge();
        }else{
            //Initialising beacon failed
        }
    }

}

void AP_Beacon_Marvelmind::getOrAllocBeacon(struct StationaryBeaconPosition &b,
        uint8_t address) {
    uint8_t i;
    uint8_t n_used = hedge->positionsBeacons.numBeacons;

    if (n_used != 0)
        for (i = 0; i < n_used; i++) {
            if (hedge->positionsBeacons.beacons[i].address == address) {
                b = hedge->positionsBeacons.beacons[i];
                return;
            }
        }

    if (n_used >= (MAX_STATIONARY_BEACONS - 1))
        return;

    hedge->positionsBeacons.numBeacons = (n_used + 1);
    b = hedge->positionsBeacons.beacons[n_used];
}

void AP_Beacon_Marvelmind::process_beacons_positions_datagram(
        struct StationaryBeaconPosition &b) {
    uint8_t n = input_buffer[5]; // number of beacons in packet
    uint8_t i, ofs;
    uint8_t address;
    int16_t x, y, z;

    if ((1 + n * 8) != input_buffer[4])
        return; // incorrect size

    for (i = 0; i < n; i++) {
        ofs = 6 + i * 8;

        address = input_buffer[ofs + 0];
        x = input_buffer[ofs + 1] | (((uint16_t) input_buffer[ofs + 2]) << 8);
        y = input_buffer[ofs + 3] | (((uint16_t) input_buffer[ofs + 4]) << 8);
        z = input_buffer[ofs + 5] | (((uint16_t) input_buffer[ofs + 6]) << 8);

        getOrAllocBeacon(b, address);
        if (b.address != NULL) {
            b.address = address;
            b.x = x * 10; // millimeters
            b.y = y * 10; // millimeters
            b.z = z * 10; // millimeters

            b.highResolution = false;

            hedge->positionsBeacons.updated = true;
        }
    }
}

void AP_Beacon_Marvelmind::process_beacons_positions_highres_datagram(
        struct StationaryBeaconPosition &b) {
    uint8_t n = input_buffer[5]; // number of beacons in packet
    uint8_t i, ofs;
    uint8_t address;
    int32_t x, y, z;

    if ((1 + n * 14) != input_buffer[4])
        return; // incorrect size

    for (i = 0; i < n; i++) {
        ofs = 6 + i * 14;

        address = input_buffer[ofs + 0];
        x = input_buffer[ofs + 1] | (((uint32_t) input_buffer[ofs + 2]) << 8)
                | (((uint32_t) input_buffer[ofs + 3]) << 16)
                | (((uint32_t) input_buffer[ofs + 4]) << 24);
        y = input_buffer[ofs + 5] | (((uint32_t) input_buffer[ofs + 6]) << 8)
                | (((uint32_t) input_buffer[ofs + 7]) << 16)
                | (((uint32_t) input_buffer[ofs + 8]) << 24);
        z = input_buffer[ofs + 9] | (((uint32_t) input_buffer[ofs + 10]) << 8)
                | (((uint32_t) input_buffer[ofs + 11]) << 16)
                | (((uint32_t) input_buffer[ofs + 12]) << 24);

        getOrAllocBeacon(b, address);
        if (b.address != NULL) {
            b.address = address;
            b.x = x;
            b.y = y;
            b.z = z;

            b.highResolution = true;

            hedge->positionsBeacons.updated = true;
        }
    }
}

uint8_t AP_Beacon_Marvelmind::markPositionReady() {
    uint8_t ind = hedge->lastValues_next;
    uint8_t indCur = ind;

    hedge->positionBuffer[ind].ready = true;
    hedge->positionBuffer[ind].processed = false;
    ind++;
    if (ind >= hedge->maxBufferedPositions)
        ind = 0;
    if (hedge->lastValuesCount_ < hedge->maxBufferedPositions)
        hedge->lastValuesCount_++;
    hedge->haveNewValues_ = true;

    hedge->lastValues_next = ind;

    return indCur;
}

void AP_Beacon_Marvelmind::process_position_datagram(struct PositionValue &p) {
    uint8_t ind = hedge->lastValues_next;

    hedge->positionBuffer[ind].address = input_buffer[16];
    hedge->positionBuffer[ind].timestamp = input_buffer[5]
            | (((uint32_t) input_buffer[6]) << 8)
            | (((uint32_t) input_buffer[7]) << 16)
            | (((uint32_t) input_buffer[8]) << 24);

    int16_t vx = input_buffer[9] | (((uint16_t) input_buffer[10]) << 8);
    hedge->positionBuffer[ind].x = vx * 10; // millimeters

    int16_t vy = input_buffer[11] | (((uint16_t) input_buffer[12]) << 8);
    hedge->positionBuffer[ind].y = vy * 10; // millimeters

    int16_t vz = input_buffer[13] | (((uint16_t) input_buffer[14]) << 8);
    hedge->positionBuffer[ind].z = vz * 10; // millimeters

    hedge->positionBuffer[ind].highResolution = false;

    ind = markPositionReady();

    p = hedge->positionBuffer[ind];
}

void AP_Beacon_Marvelmind::process_position_highres_datagram(
        struct PositionValue &p) {
    uint8_t ind = hedge->lastValues_next;

    hedge->positionBuffer[ind].address = input_buffer[22];
    hedge->positionBuffer[ind].timestamp = input_buffer[5]
            | (((uint32_t) input_buffer[6]) << 8)
            | (((uint32_t) input_buffer[7]) << 16)
            | (((uint32_t) input_buffer[8]) << 24);

    int32_t vx = input_buffer[9] | (((uint32_t) input_buffer[10]) << 8)
            | (((uint32_t) input_buffer[11]) << 16)
            | (((uint32_t) input_buffer[12]) << 24);
    hedge->positionBuffer[ind].x = vx;

    int32_t vy = input_buffer[13] | (((uint32_t) input_buffer[14]) << 8)
            | (((uint32_t) input_buffer[15]) << 16)
            | (((uint32_t) input_buffer[16]) << 24);
    hedge->positionBuffer[ind].y = vy;

    int32_t vz = input_buffer[17] | (((uint32_t) input_buffer[18]) << 8)
            | (((uint32_t) input_buffer[19]) << 16)
            | (((uint32_t) input_buffer[20]) << 24);
    hedge->positionBuffer[ind].z = vz;

    hedge->positionBuffer[ind].highResolution = true;

    ind = markPositionReady();

    p = hedge->positionBuffer[ind];
}

uint16_t AP_Beacon_Marvelmind::CalcCrcModbus_(uint8_t * buf, int len) {
    uint16_t crc = 0xFFFF;
    int pos;
    for (pos = 0; pos < len; pos++) {
        crc ^= (uint16_t) buf[pos]; // XOR byte into least sig. byte of crc
        int i;
        for (i = 8; i != 0; i--) // Loop over each bit
                {
            if ((crc & 0x0001) != 0) // If the LSB is set
                    {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else
                // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    return crc;
}

void AP_Beacon_Marvelmind::createMarvelmindHedge() {

    hedge->maxBufferedPositions = 3;
    hedge->positionBuffer = NULL;
    hedge->verbose = false;
    hedge->receiveDataCallback = NULL;
    hedge->lastValuesCount_ = 0;
    hedge->lastValues_next = 0;
    hedge->haveNewValues_ = false;
    hedge->terminationRequired = false;

}

bool AP_Beacon_Marvelmind::healthy() {
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

void AP_Beacon_Marvelmind::startMarvelmindHedge() {
    uint8_t i;
    hedge->positionBuffer = (PositionValue*) malloc(
            sizeof(struct PositionValue) * hedge->maxBufferedPositions);
    if (hedge->positionBuffer == NULL) {
        if (hedge->verbose)
            puts("Not enough memory");
        hedge->terminationRequired = true;
        return;
    }
    for (i = 0; i < hedge->maxBufferedPositions; i++) {
        hedge->positionBuffer[i].ready = false;
        hedge->positionBuffer[i].processed = false;
    }
    hedge->positionsBeacons.numBeacons = 0;
    hedge->positionsBeacons.updated = false;
}

void AP_Beacon_Marvelmind::update(void) {
    static uint8_t counter = 0;
    counter++;
    if (counter > 200) {
        counter = 0;
    }

    if (uart == nullptr) {
        return;
    }

    // read any available characters
    int32_t nBytesRead = uart->available();

    uint8_t receivedChar = 0;
    bool readSuccessed = true;
    if (nBytesRead < 0)
        readSuccessed = false;
    if (readSuccessed) {
        while (nBytesRead-- > 0) {
            bool goodByte = false;
            receivedChar = uart->read();
            input_buffer[nBytesInBlockReceived] = receivedChar;
            switch (Parse_State) {
            case RECV_HDR:
                switch (nBytesInBlockReceived) {
                case 0:
                    goodByte = (receivedChar == 0xff);
                    break;
                case 1:
                    goodByte = (receivedChar == 0x47);
                    break;
                case 2:
                    goodByte = true;
                    break;
                case 3:
                    dataId = (((uint16_t) receivedChar) << 8) + input_buffer[2];
                    goodByte =
                            (dataId == POSITION_DATAGRAM_ID)
                                    || (dataId == BEACONS_POSITIONS_DATAGRAM_ID)
                                    || (dataId == POSITION_DATAGRAM_HIGHRES_ID)
                                    || (dataId
                                            == BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID);
                    break;
                case 4: {
                    switch (dataId) {
                    case POSITION_DATAGRAM_ID: {
                        goodByte = (receivedChar == 0x10);
                    }
                        break;
                    case BEACONS_POSITIONS_DATAGRAM_ID:
                    case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                        goodByte = true;
                        break;
                    case POSITION_DATAGRAM_HIGHRES_ID: {
                        goodByte = (receivedChar == 0x16);
                    }
                        break;
                    }
                    if (goodByte)
                        Parse_State = RECV_DGRAM;
                }
                    break;

                }
                if (goodByte) {
                    // correct header byte
                    nBytesInBlockReceived++;
                } else {
                    // ...or incorrect
                    Parse_State = RECV_HDR;
                    nBytesInBlockReceived = 0;
                }
                break;

            case RECV_DGRAM:
                nBytesInBlockReceived++;
                if (nBytesInBlockReceived >= 7 + input_buffer[4]) {
                    // parse dgram
                    uint16_t blockCrc = CalcCrcModbus_(input_buffer,
                            nBytesInBlockReceived);
                    if (blockCrc == 0) {
                        switch (dataId) {
                        case POSITION_DATAGRAM_ID:
                            // add to positionBuffer
                        {
                            process_position_datagram(curPosition);
                            //Vector3f pos(curPosition.x/1000.0f, curPosition.y/1000.0f, curPosition.z/1000.0f);
                            //set_vehicle_position(pos,0.0f);
                        }
                            break;
                        case BEACONS_POSITIONS_DATAGRAM_ID:
                        {
                            process_beacons_positions_datagram(curBeacon);
                            Vector3f pos(curBeacon.x/1000.0f, curBeacon.y/1000.0f, curBeacon.z/1000.0f);
                            set_beacon_position(curBeacon.address, pos);
                            set_beacon_distance(curBeacon.address,pos.length());
                        }

                            break;
                        case POSITION_DATAGRAM_HIGHRES_ID:
                            // add to positionBuffer
                            process_position_highres_datagram(curPosition);
                            break;
                        case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                            process_beacons_positions_highres_datagram(
                                    curBeacon);
                            break;
                        }

                        // callback
                        if (hedge->receiveDataCallback) {
                            if (dataId == POSITION_DATAGRAM_ID) {
                                hedge->receiveDataCallback(curPosition);
                                Vector3f pos(curPosition.x/1000.0f, curPosition.y/1000.0f, curPosition.z/1000.0f);
                                set_vehicle_position(pos,0.0f); //TODO: Calculate Accuracy of the received signal
                            }
                        }
                    }
                    // and repeat
                    Parse_State = RECV_HDR;
                    nBytesInBlockReceived = 0;
                }
                break;
            }
        }
    }
}


