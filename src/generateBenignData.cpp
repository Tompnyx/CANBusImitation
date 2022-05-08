//
// Created by tompnyx on 11/04/2022.
//
#include "generateBenignData.h"
#include <can-serial.h>
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>
#include <SPI.h>

// CONSTANTS ==================================================================
// Sets the maximum payload size - CANFD can carry data up to 64 bytes
#define MAX_DATA_SIZE 64

// PUBLIC VARIABLES ===========================================================
// CAN_2515
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;
// Set CS pin
mcp2515_can CAN(SPI_CS_PIN);
// The time the device started
unsigned long timeStart;
// The CAN ID to filter for (If equal to 0 it will be ignored)
unsigned long filterId = 0;

// ARDUINO FUNCTIONS ==========================================================

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    // wait for Serial
    while (!SERIAL_PORT_MONITOR){}
    // init can bus at a baudrate = 500k
    if (CAN_OK != CAN.begin(CAN_500KBPS)) {
        SERIAL_PORT_MONITOR.println("CAN init fail");
    } else {
        SERIAL_PORT_MONITOR.println("CAN init ok!");
    }
    // Records the time the device started looping
    timeStart = millis();
    // Initialises the seed randomly
    randomSeed(analogRead(0));
}

void loop() {
    send_can(true);
    // Checks to see if there is incoming data
    while (CAN_MSGAVAIL == CAN.checkReceive()) {
        receive_can();
    }
    // send data per 5s
    delay(10);
}

// CREATE MESSAGES ============================================================

void generate_empty_message(unsigned long &id,
                            byte &ext,
                            byte &rtrBit,
                            byte &len,
                            unsigned char *payload) {
    // send data:  id = 0x00, standard frame, data len = Max data size,
    // payload: data buf
    payload[MAX_DATA_SIZE - 1] = payload[MAX_DATA_SIZE - 1] + 1;
    if (payload[MAX_DATA_SIZE - 1] == 100) {
        payload[MAX_DATA_SIZE - 1] = 0;

        payload[MAX_DATA_SIZE - 2] = payload[MAX_DATA_SIZE - 2] + 1;
        if (payload[MAX_DATA_SIZE - 2] == 100) {
            payload[MAX_DATA_SIZE - 2] = 0;

            payload[MAX_DATA_SIZE - 3] = payload[MAX_DATA_SIZE - 3] + 1;
        }
    }
    id = 0x00;
    ext = 0;
    rtrBit = 0;
    len = 8;
    #if MAX_DATA_SIZE > 8
        len = CANFD::len2dlc(len);
    #else
        len = 8;
    #endif
}

void generate_random_id(unsigned long &id, bool isCanFD) {
    if (isCanFD) {
        // A CAN FD ID is now generated - which takes up to 29 bits. As the
        // Arduino AVR only generates up to 16-bit random numbers, a second
        // mask is applied to the first to generate a 29 bit random number.
        id = random(0x1U << 14);
        id |= (uint32_t)random(0x1U << 15) << 14;
    } else {
        // A CAN ID is now generated - which takes up to 11 bits.
        id = random(0x1U << 11);
    }
}

void generate_random_payload(byte &len, unsigned char *payload, bool isRTR) {
    if (isRTR) {
        // Here an RTR message is sent - so the payload has to be empty.
        len = 0;
    } else {
        // Here a normal message is randomly generated.
        len = random(0, MAX_DATA_SIZE + 1);
    }

    // Populate the payload (message buffer) with random values.
    int i;
    for (i = 0; i < len; i++) {
        payload[i] = random(0x100);
    }

    // Pads the rest of the
    for (i = len; i < MAX_DATA_SIZE; i++) {
        payload[i] = 0;
    }

    // Sets the length of the data sent.
    #if MAX_DATA_SIZE > 8
        len = CANFD::len2dlc(len);
    #else
        len = 8;
    #endif
}

void generate_random_message(unsigned long &id,
                             byte &ext,
                             byte &rtrBit,
                             byte &len,
                             unsigned char *payload) {
    // bit0: ext, bit1: rtr
    uint8_t type = random(4);

    // Set values of EXT and RTR bit.

    // EXT is the type of frame that is generated. A EXT bit of 1 means the
    // message sent is a CAN FD message.
    ext = bool(type & 0x1);
    // RTR is the Remote Request Frame. It is a feature that sends empty
    // packages requesting data from the target ID. If a CAN FD message is
    // sent (which has a 29 bit identifier), the 11 most significant bits
    // change to the Substitute Remote Request (SRR).
    rtrBit = bool(type & 0x2);

    // Generates a random CAN ID
    // ext is true if the zero bit - e.g., 000X - is populated.
    generate_random_id(id, ext);
    // Generates a random payload and sets length accordingly
    // rtr is true if the first bit - e.g., 00X0 - is populated
    generate_random_payload(len, payload, rtrBit);
}

// DISPLAY MESSAGES ===========================================================

void print_can_message_to_monitor(unsigned long canId,
                                  byte len,
                                  unsigned char *buf) {
    // Variables used to pad values to fit formatting
    char canIdString[9];
    char time[9];
    // Prints the timestamp to terminal
    SERIAL_PORT_MONITOR.print("Timestamp:\t");
    sprintf(time, "%08lu", millis() - timeStart);
    SERIAL_PORT_MONITOR.print(time);
    // Prints the CAN ID to terminal
    SERIAL_PORT_MONITOR.print("\tID:\t");
    sprintf(canIdString, "%08lx", canId);
    SERIAL_PORT_MONITOR.print(canIdString);
    // Prints the length of the message to terminal
    SERIAL_PORT_MONITOR.print("\tDLC:\t");
    SERIAL_PORT_MONITOR.print(len);
    SERIAL_PORT_MONITOR.print("\t");
    // Prints the message to terminal
    for (int i = 0; i < len; i++) {
        SERIAL_PORT_MONITOR.print(buf[i], HEX);
        if (buf[i] < 16) {
            SERIAL_PORT_MONITOR.print("0");
        }
        SERIAL_PORT_MONITOR.print(" ");
    }
    SERIAL_PORT_MONITOR.println();
}

// SENDING AND RECEIVING ======================================================

void send_can(bool sendRandom) {
    unsigned char payload[MAX_DATA_SIZE] = {0};
    unsigned long id = 0x00;
    byte ext;
    byte rtrBit;
    // Make sure len is unsigned - unsigned bytes are native to c
    byte len;

    if (sendRandom) {
        generate_random_message(id, ext, rtrBit, len, payload);
    } else {
        generate_empty_message(id, ext, rtrBit, len, payload);
    }
    CAN.sendMsgBuf(id, ext, rtrBit, len, payload);
    SERIAL_PORT_MONITOR.print("Sent Message:\t");
    print_can_message_to_monitor(id, len, payload);
}

void send_can(unsigned long id) {
    unsigned char payload[MAX_DATA_SIZE] = {0};
    byte ext = 0x01;
    byte rtrBit = 0x02;
    // Make sure len is unsigned - unsigned bytes are native to c
    byte len;

    generate_random_payload(len, payload, rtrBit);
    CAN.sendMsgBuf(id, ext, rtrBit, len, payload);
    SERIAL_PORT_MONITOR.print("Sent Message:\t");
    print_can_message_to_monitor(id, len, payload);
}

void send_can(unsigned long id, unsigned char *payload, byte len) {
    byte ext = 0x01;
    byte rtrBit = 0x02;

    CAN.sendMsgBuf(id, ext, rtrBit, len, payload);
    SERIAL_PORT_MONITOR.print("Sent Message:\t");
    print_can_message_to_monitor(id, len, payload);
}

void receive_can() {
    unsigned char len = 0;
    unsigned char buf[MAX_DATA_SIZE];

    SERIAL_PORT_MONITOR.print("Received Msg:\t");
    // read data,  len: data length, buf: data buf
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();

    // Checks to see if filter is enabled, and if the message should be let
    // through or not
    if (filterId == 0 || filterId == canId) {
        // Prints the canID to terminal
        print_can_message_to_monitor(canId, len, buf);
    }
}

// END FILE ===================================================================