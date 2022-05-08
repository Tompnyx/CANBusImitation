//
// Created by tompn on 11/04/2022.
//

#ifndef CLIONARDUINO_GENERATEBENIGNDATA_H
#define CLIONARDUINO_GENERATEBENIGNDATA_H
#include <Arduino.h>

// List of ECUs and their priority

//LIGHTS
#define HEADLIGHTS 0x0CF0F000
#define HIGHBEAM 0x0CF0F001
#define FOGLIGHTS 0x0CF0F002
#define RIGHTTURNINGLIGHT 0x0CF0F003
#define LEFTTURNINGLIGHT 0x0CF0F004
#define BREAKLIGHTS 0x0CF0F005
#define TAILLIGHTS 0x0CF0F006

// DOORS
#define TOPRIGHTDOORMODULE 0x0CF00300
#define TOPLEFTDOORMODULE 0x0CF00301
#define BOTTOMRIGHTDOORMODULE 0x0CF00302
#define BOTTOMLEFTDOORMODULE 0x0CF00303
#define TRUNKDOORMODULE 0x0CF00304

// WIPERS
#define FRONTWIPERS 0x0CFF0410
#define TRUNKDOORWIPER 0x0CFF0411

// SEATS
#define DRIVERSEATMODULE 0x0CF0F410
#define PASSENGERSEATMODULE 0x0CF0F411
#define REARSEATMODULE 0x0CF0F412

// ROOF
#define ROOFMODULE 0x0CFF041F

// POWERTRAIN
#define AIRBAGSYSTEM 0x0000FF00
#define TCMTRANSMISSION 0x0000FF01

// CHASSIS
#define STEERINGANGLESENSORMODULE 0x00000FF0
#define POWERSTEERINGCONTROLMODULE 0x00000FF1
#define ALLWHEELDRIVECONTROLMODULE 0x00000FF2

// COMFORT
#define AUDIOSYSTEM 0x0CF00940
#define VENTILATIONSYSTEM 0x0CF00941
#define AIRCONDITIONINGCONTROL 0x0CF00942
#define REARCAMERA 0x0CF00943

// MISC
#define DASHBOARD 0x00005FF0
#define DIAGNOSTICSYSTEM 0x0000FFFF
#define TOUCHSCREEN 0x00005FF1
#define CENTRALBODYCONTROL 0x00000C54



void generate_empty_message(unsigned long &id,
                             byte &ext,
                             byte &rtrBit,
                             byte &len,
                             unsigned char *payload);
void generate_random_id(unsigned long &id, bool isCanFD);
void generate_random_payload(byte &len, unsigned char *payload, bool isRTR);
void generate_random_message(unsigned long &id,
                             byte &ext,
                             byte &rtrBit,
                             byte &len,
                             unsigned char *payload);
void print_can_message_to_monitor(unsigned long canId,
                                  byte len,
                                  unsigned char *buf);
void send_can(bool sendRandom);
void send_can(unsigned long id);
void send_can(unsigned long id, unsigned char *payload, byte len);
void receive_can();

/* CANFD Auxiliary helper */
class CANFD {
public:
    static byte dlc2len(byte dlc);
    static byte len2dlc(byte len);
    static uint32_t BITRATE(uint32_t arbitration, uint8_t factor) {
        return ((uint32_t)factor << 24) | (arbitration & 0xFFFFFUL);
    }
};

#endif //CLIONARDUINO_GENERATEBENIGNDATA_H
