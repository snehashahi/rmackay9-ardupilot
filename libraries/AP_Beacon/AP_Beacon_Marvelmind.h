/*
 * AP_Beacon_Marvelmind.h
 *
 *  Created on: 21.03.2017
 */

#ifndef AP_BEACON_MARVELMIND_H_
#define AP_BEACON_MARVELMIND_H_

#pragma once

#define MAX_STATIONARY_BEACONS 30
#define POSITION_DATAGRAM_ID 0x0001
#define BEACONS_POSITIONS_DATAGRAM_ID 0x0002
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>

#include "AP_Beacon_Backend.h"

    struct PositionValue
    {
        uint8_t address;
        uint32_t timestamp;
        int32_t x, y, z;// coordinates in millimeters

        bool highResolution;

        bool ready;
        bool processed;
    };

    struct StationaryBeaconPosition
    {
        uint8_t address;
        int32_t x, y, z;// coordinates in millimeters

        bool highResolution;
    };

    struct StationaryBeaconsPositions
    {
        uint8_t numBeacons;
        struct StationaryBeaconPosition beacons[MAX_STATIONARY_BEACONS];

        bool updated;
    };

    struct MarvelmindHedge
    {

    // maximum count of measurements of coordinates stored in buffer
    // default: 3
        uint8_t maxBufferedPositions;

    // buffer of measurements
        struct PositionValue * positionBuffer;

        struct StationaryBeaconsPositions positionsBeacons;

    // verbose flag which activate console output
    //      default: False
        bool verbose;

    //  pause flag. If True, class would not read serial data
        bool pause;

    //  If True, thread would exit from main loop and stop
        bool terminationRequired;

    //  receiveDataCallback is callback function to recieve data
        void (*receiveDataCallback)(struct PositionValue position);

    // private variables
        uint8_t lastValuesCount_;
        uint8_t lastValues_next;
        bool haveNewValues_;
       // pthread_t thread_;
       // pthread_mutex_t lock_;
    };

class AP_Beacon_Marvelmind : public AP_Beacon_Backend
{
public:
    // constructor
    AP_Beacon_Marvelmind(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy();

    // update
    void update();

private:
    enum
    {
        RECV_HDR,
        RECV_DGRAM
    }Parse_State; // current state of receive data

    struct MarvelmindHedge * hedge;
    struct PositionValue curPosition;
    struct StationaryBeaconPosition curBeacon;
    uint8_t input_buffer[256];
    uint8_t nBytesInBlockReceived; // bytes received
    uint16_t dataId;

    struct MarvelmindHedge* m_MarvelmindHedge;
    uint16_t CalcCrcModbus_(uint8_t * buf, int len);
    void getOrAllocBeacon(struct StationaryBeaconPosition &b, uint8_t address);
    uint8_t markPositionReady();
    void process_beacons_positions_datagram(struct StationaryBeaconPosition &b);
    void process_beacons_positions_highres_datagram(struct StationaryBeaconPosition &b);
    void process_position_highres_datagram(struct PositionValue &p);
    void process_position_datagram(struct PositionValue &p);
    void createMarvelmindHedge ();
    void startMarvelmindHedge ();
    bool getPositionFromMarvelmindHedge ( struct PositionValue * position);
    void stopMarvelmindHedge ();

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_update_ms = 0;
};



#endif /* AP_BEACON_MARVELMIND_H_ */
