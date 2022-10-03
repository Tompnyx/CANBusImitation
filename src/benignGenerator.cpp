//
// Created by Tompnyx on 11/04/2022.
//
#include "benignGenerator.h"
#include "Vehicle.h"

#include <Arduino.h>
#include <mcp2515_can.h>
#include <SD.h>

// CONSTANTS ==================================================================
// Sets the maximum payload size - CANFD can carry data up to 64 bytes, whereas
// CAN 2.0 can only carry up to 8 bytes
#define MAX_DATA_SIZE 8
// The baud rate of the board
#define BAUD 115200
// The SPI CS Pin of the Arduino
#define SPI_CS_PIN 10
// The CS Int Pin of the Arduino
#define CAN_INT_PIN 2
// The SD SPI CS PIN of the Arduino (If your Arduino or CAN-BUS shield has SD
// card capabilities
#define SD_SPI_CS_PIN 4
// Definitions for the action values stored in the actions array
#define ACCELERATE 1
#define BREAK 2
#define TURN 3
// Size of the largest character action command
#define MAX_ACTION_SIZE 11

// PUBLIC VARIABLES ===========================================================
// Sets CS pin (as defined above)
mcp2515_can CAN(SPI_CS_PIN);
// The different operations that can be performed
enum Operation { sendRandom, receiveOnly, sendRandomAndReceive, performRoute};
// Change op to set the operation mode
Operation op = performRoute;

// The filename of the json file containing the trip information
const char *filename = "ltrip.txt";
// The file that the SD card is read to
File sdFile;
// Delimiter that separates the different values in the file present on the
// SD card
const char delimiter = '/';
// The comment indicator for the SD card, so that any lines starting with this
// character will be ignored
const char commentIndicator = '#';
// The total amount of actions needed to
unsigned short actionTotal;
// The current action
unsigned short actionCounter = 0;
// The amount of iterations before the next action is performed
signed short actionDelay = 0;
// Array of actions read from sdFile
unsigned short *actions;
// Array of targets read from sdFile
unsigned short *targets;
// Array of delays read from sdFile
int *delays;

// The time the device started
unsigned long timeStart;
// The CAN ID to filter for (If equal to 0 it will be ignored)
unsigned long filterId = 0;
// How many loops in a second. The target is a 'loop' every 0.0002 seconds.
short lis = 5000;
// The vehicle object used by the performRoute option
Vehicle vehicle;

// ARDUINO FUNCTIONS ==========================================================

void setup() {
    // Initialise the board at the specified baud
    SERIAL_PORT_MONITOR.begin(BAUD);
    // Wait for a serial connection
    while (!SERIAL_PORT_MONITOR);
    // Initialise the CAN board at the specified speed
    if (CAN_OK != CAN.begin(CAN_500KBPS)) {
        SERIAL_PORT_MONITOR.println("CAN init fail");
        SERIAL_PORT_MONITOR.flush();
        exit(0);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
    // Records the time the device started looping
    timeStart = millis();
    // Initialises the seed randomly
    randomSeed(analogRead(0));

    // Initialises the performRoute option if selected
    if (op == performRoute) {
        initialiseFile();
    }
}

void loop() {
    // Performs the performRoute option if selected
    if (op == performRoute) {
        overview_vehicle_functionality_loop();
    } else {
        if (op != sendRandom) {
            // Checks to see if there is incoming data
            if (CAN_MSGAVAIL == CAN.checkReceive()) {
                receive_can();
            }
        }
        if (op != receiveOnly) {
            // Sends a random CAN message
            send_can(true);
        }
    }

    if (CAN.checkError() == CAN_CTRLERROR) {
        SERIAL_PORT_MONITOR.println("A CAN Control Error has occurred."
                                    " Stopping processes...");
        SERIAL_PORT_MONITOR.flush();
        exit(0);
    }
}

// SETUP JSON OBJECT ==========================================================

void initialiseFile() {
    // init the sd card
    if (!SD.begin(SD_SPI_CS_PIN)) {
        SERIAL_PORT_MONITOR.println("SD init fail");
        SERIAL_PORT_MONITOR.flush();
        exit(0);
    }
    SERIAL_PORT_MONITOR.println("SD init ok!");

    // Read the json file if needed
    sdFile = SD.open(filename, FILE_READ);
    // Test that the file is available
    if (!sdFile) {
        SERIAL_PORT_MONITOR.println("File failed to open");
        SERIAL_PORT_MONITOR.flush();
        sdFile.close();
        exit(0);
    }
    SERIAL_PORT_MONITOR.println("File opened successfully");

    while (sdFile.available()) {
        // Skip over comment lines until an input line is read
        if (sdFile.peek() == commentIndicator) {
            while (sdFile.available()) {
                if (sdFile.read() == '\n') break;
            }
            continue;
        }

        // Read the actionTotal
        char buffer[MAX_ACTION_SIZE] = {0};
        sdFile.readBytesUntil('\n', buffer, MAX_ACTION_SIZE);
        actionTotal = (unsigned short) strtol(buffer, nullptr, 10);
        break;
    }

    // Assign memory to arrays used to keep the relevant information
    unsigned short counter = 0;
    actions = (unsigned short *) malloc(sizeof(unsigned short) * actionTotal);
    delays = (int *) malloc(sizeof(int) * actionTotal);
    targets = (unsigned short *) malloc(sizeof(unsigned short) * actionTotal);

    while (sdFile.available()) {
        // Skip over comment lines until an input line is read
        if (sdFile.peek() == commentIndicator) {
            while (sdFile.available()) {
                if (sdFile.read() == '\n') break;
            }
            continue;
        }

        // Parse the read line into the needed information kept in the relevant
        // arrays
        char buffer[MAX_ACTION_SIZE] = {0};

        // Read the action input
        sdFile.readBytesUntil(delimiter, buffer, MAX_ACTION_SIZE);
        if (strcmp(buffer, "accelerate") == 0) {
            actions[counter] = ACCELERATE;
        } else if (strcmp(buffer, "break") == 0) {
            actions[counter] = BREAK;
        } else if (strcmp(buffer, "turn") == 0) {
            actions[counter] = TURN;
        }
        memset(buffer, 0, sizeof(buffer));

        // Read the delay input
        sdFile.readBytesUntil(delimiter, buffer, MAX_ACTION_SIZE);
        delays[counter] = (int) strtol(buffer, nullptr, 10);

        // Read the target input
        sdFile.readBytesUntil('\n', buffer, MAX_ACTION_SIZE);
        // Convert the short to unsigned short
        auto signedShort = (short) strtol(buffer, nullptr, 10);
        // As strtol converts the number to a signed value, this is converted
        // back by adding it to the maximum possible value for an unsigned
        // short
        targets[counter] = (unsigned short) (signedShort < 0) ?
                           (65536 + signedShort) : signedShort;

        // Iterate counter
        counter++;
    }

    // Close the file
    sdFile.close();

    // Flush the Serial port to ensure all 'printed' messages have been sent
    SERIAL_PORT_MONITOR.flush();
}

// CREATE MESSAGES ============================================================

void generate_random_id(unsigned long &id, bool ext) {
    if (ext) {
        // An Extended Frame ID is now generated - which takes up to 29 bits.
        // As the Arduino AVR only generates up to 16-bit random numbers, a
        // second mask is applied to the first to generate a 29 bit random
        // number.
        id = random(0x1U << 14);
        id |= (uint32_t)random(0x1U << 15) << 14;
    } else {
        // A Standard Frame ID is now generated - which takes up to 11 bits.
        id = random(0x1U << 11);
    }
}

void generate_random_payload(byte &len, unsigned char *payload, bool ext,
                             bool rtr) {
    if (rtr) {
        // Here an RTR message is sent - so the payload has to be empty.
        len = 0;
    } else {
        // Here a normal message is randomly generated.
        // Remember the condition max in the function random is exclusive to
        // the upper bound.
    #if MAX_DATA_SIZE > 8
        len = ext ? random(16) : random(9);
    #else
        // Generates a random value between 0 and 8.
        len = random(9);
    #endif
    }

    // Populate the payload (message buffer) with random values.
    int i;
    for (i = 0; i < len; i++) {
        payload[i] = random(0x100);
    }
}

void generate_random_message(unsigned long &id,
                             byte &ext,
                             byte &rtr,
                             byte &len,
                             unsigned char *payload) {
    // Here a random number up to 4 is generated. The value of the bit
    // determines the value of ext and rtr.
    // ext is true if the zero bit - e.g., 0b0X - is populated.
    // rtr is true if the first bit - e.g., 0bX0 - is populated.
    // 3 = 0b11 -> ext = true,  rtr = true
    // 2 = 0b10 -> ext = false, rtr = true
    // 1 = 0b01 -> ext = true,  rtr = false
    // 0 = 0b00 -> ext = false, rtr = false
    long type = random(4);

    // Set values of EXT and RTR bit.

    // EXT is the type of frame that is generated. A EXT bit of 1 means the
    // message sent is an extended frame message, whereas a standard frame
    // would have  a bit of 0
    ext = type & 0x1;
    // RTR is the Remote Request Frame. It is a feature that sends empty
    // packages requesting data from the target ID. If a CAN extended message
    // is sent (which has a 29 bit identifier), the 11 most significant bits
    // change to the Substitute Remote Request (SRR).
    rtr = type & 0x2;

    // Generates a random CAN ID
    generate_random_id(id, ext);
    // Generates a random payload and sets length accordingly
    generate_random_payload(len, payload, ext, rtr);
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
        char tmp[3];
        sprintf(tmp, "%.2x", buf[i]);
        SERIAL_PORT_MONITOR.print(tmp);
        SERIAL_PORT_MONITOR.print(" ");
    }
    SERIAL_PORT_MONITOR.println();
}

// SENDING AND RECEIVING ======================================================

bool check_message_sent(int report) {
    if (report == CAN_SENDMSGTIMEOUT) {
        // CAN_SENDMSGTIMEOUT: A timeout has occurred
        SERIAL_PORT_MONITOR.println("A CAN_SENDMSGTIMEOUT has occurred");
    } else if (report == CAN_GETTXBFTIMEOUT) {
        // CAN_GETTXBFTIMEOUT: The program has failed to get the next free
        // buffer. This has most likely occurred due to the buffer being full.
        SERIAL_PORT_MONITOR.println("A CAN_GETTXBFTIMEOUT has occurred");
    } else {
        // CAN_OK: everything is working
        return true;
    }
    return false;
}

void send_can(bool sendRandom) {
    unsigned char payload[MAX_DATA_SIZE] = {0};
    unsigned long id = 0x00;
    byte ext;
    byte rtr;
    // Make sure len is unsigned - unsigned bytes are native to c
    byte len;
    // The return value of sending the message
    int report;

    if (sendRandom) {
        generate_random_message(id, ext, rtr, len, payload);
        report = CAN.sendMsgBuf(id, ext, rtr, len, payload);
    } else {
        report = CAN.sendMsgBuf(0x00, 0, 0, 8, payload);
    }

    if (check_message_sent(report)) {
        SERIAL_PORT_MONITOR.print("Sent Message:\t");
        print_can_message_to_monitor(id, len, payload);
    }
}

void send_can(unsigned long id, bool ext_condition, bool rtr_condition) {
    unsigned char payload[MAX_DATA_SIZE] = {0};
    byte ext = (ext_condition) ? 1 : 0;
    byte rtr = (rtr_condition) ? 1 : 0;
    // Make sure len is unsigned - unsigned bytes are native to c
    byte len;
    // The return value of sending the message
    int report;

    generate_random_payload(len, payload, ext, rtr);
    report = CAN.sendMsgBuf(id, ext, rtr, len, payload);

    if (check_message_sent(report)) {
        SERIAL_PORT_MONITOR.print("Sent Message:\t");
        print_can_message_to_monitor(id, len, payload);
    }
}

void send_can(unsigned long id, bool ext_condition, bool rtr_condition,
              unsigned char *payload, byte len) {
    // The return value of sending the message
    int report;
    byte ext = (ext_condition) ? 1 : 0;
    byte rtr = (rtr_condition) ? 1 : 0;

    report = CAN.sendMsgBuf(id, ext, rtr, len, payload);

    if (check_message_sent(report)) {
        SERIAL_PORT_MONITOR.print("Sent Message:\t");
        print_can_message_to_monitor(id, len, payload);
    }
}

void receive_can() {
    unsigned char len = 0;
    unsigned char buf[MAX_DATA_SIZE];
    unsigned long canId = 0;

    SERIAL_PORT_MONITOR.print("Received Msg:\t");
    // Reads the data from the CAN message
    // CAN ID being the CAN ID
    // len being the length of the message (in bytes)
    // buf being the buffer to store the message
    CAN.readMsgBufID(&canId, &len, buf);

    // Checks to see if filter is enabled, and if the message should be let
    // through or not
    if (filterId == 0 || filterId == canId) {
        // Prints the canID to terminal
        print_can_message_to_monitor(canId, len, buf);
    }
}

// VEHICLE HARDWARE FUNCTIONS =================================================

// The following section is heavily inspired by the work done by DaveBlackH on
// his GitHub repository 'MazdaRX8Arduino'. Accessed 09/05/2022 from
// https://github.com/DaveBlackH/MazdaRX8Arduino

// Hence, certain features and functions will be loosely based on the Mazda
// RX8 model.

void SAS(unsigned short angle) {
    unsigned char payload[8] = {0};
    // Generate random first byte
    payload[0] = random(0xFF);
    // Determines whether the steering wheel is centered
    if (angle == 0) {
        payload[1] = 0xEF;
    } else {
        payload[1] = 0x6F;
        // Checks and sets the angle between the bounds
        if (angle < 542) {
            angle = 542;
        } else if (angle > 64993) {
            angle = 64993;
        }

        payload[2] = angle >> 8; // Divide by 256
        payload[3] = angle & 0xFF; // Modulus by 256
    }

    send_can(SASID, false, false, payload, 4);
}

void ABS(bool DSC, bool ABS, bool breakFailure, bool TC) {
    unsigned char payload[8] = {0};

    if (DSC) {
        payload[3] = 0b00000100;
    }

    if (ABS) {
        payload[4] |= 0b00001000;
    }
    if (breakFailure) {
        payload[4] |= 0b01000000;
    }

    if (TC) {
        payload[5] = 0b00100000;
    } else {
        payload[5] = 0b00010000;
    }

    send_can(ABSID, false, false, payload, 7);
}

void EPS(bool EPSOn) {
    unsigned char payload[8] = {0};

    if (EPSOn) {
        payload[0] = 0b10000000;
    }

    send_can(EPSID, false, false, payload, 1);
}

void WHEEL_SPEEDS(unsigned short speed) {
    unsigned char payload[8] = {0};
    unsigned char payload_10k[8] = {0};

    // As the information is stored in the payload as big endian, the following
    // transformation needs to occur
    for (int i = 0; i < 8; i = i + 2) {
        payload[i] = speed >> 8; // Divide by 256
        payload_10k[i] = (speed + 10000) >> 8;
        payload[i + 1] = speed & 0xFF; // Modulus by 256
        payload_10k[i + 1] = (speed + 10000) & 0xFF;
    }

    send_can(WHEELID, false, false, payload, 8);
    send_can(WHEEL10kID, false, false, payload_10k, 8);
}

void ODOMETER(bool increment) {
    unsigned char payload[8] = {0};
    if (increment) {
        payload[0] = 0b00000001;
    }
    send_can(ODOMETER_ID, false, false, payload, 1);
}

void PCM(double RPM, unsigned short kmph, unsigned short throttle) {
    unsigned char payload[8] = {0};

    // Convert RPM to the modified value and Big Endian
    RPM = RPM * 3.85;
    payload[0] = (short) RPM >> 8;
    payload[1] = (short) RPM & 0xFF;

    // Static bytes
    payload[2] = 0xFF;
    payload[3] = 0xFF;

    // Convert KMPH to the modified value and Big Endian
    kmph = (kmph * 100) + 10000;
    payload[4] = (short) kmph >> 8;
    payload[5] = (short) kmph * 0xFF;

    // Check throttle lies within bounds
    if (throttle > 0xC8) {
        payload[6] = 0xC8;
    } else {
        payload[6] = throttle;
    }

    // Static byte
    payload[7] = 0xFF;

    send_can(PCMID, false, false, payload, 8);
}

void PCM_IC(unsigned short engineTemp, bool odometerIncrement,
            bool oilPressureOK, bool engineLightOn, bool engineLightBlinking,
            bool lowCoolant, bool batteryCharge) {
    unsigned char payload[8] = {0};

    // Check the range of the engine temperature
    if (engineTemp < 128) {
        payload[0] = 128;
    } else if (engineTemp > 192) {
        payload[0] = 192;
    } else {
        payload[0] = engineTemp;
    }

    if (odometerIncrement) {
        payload[1] = 0b00000001;
    }

    if (oilPressureOK) {
        payload[4] = 1;
        // Sets the correct bit in byte 7
        payload[6] |= 0b10000000;
    }

    // Checks to see if the engine light is on or blinking
    if (engineLightBlinking) {
        payload[5] = 0b10000000;
    } else if (engineLightOn) {
        payload[5] = 0b01000000;
    }

    if (lowCoolant) {
        payload[6] |= 0b00000010;
    }
    if (batteryCharge) {
        payload[6] |= 0b01000000;
    }

    send_can(PCM_ICID, false, false, payload, 7);
}

// VEHICLE PROCESSES ==========================================================

// This section includes the processes to control the vehicle e.g.,
// accelerating or breaking the vehicle

void overview_vehicle_functionality_loop() {
    // Check to see if there is a current delay. If not, the next action will
    // be read and performed.
    if (actionDelay <= 0) {
        if (actionCounter >= actionTotal) {
            if (vehicle.vehicleTargetsMet()) {
                // Trip is done
                SERIAL_PORT_MONITOR.flush();
                // Free the relevant arrays
                free(actions);
                free(targets);
                free(delays);
                // Exit
                exit(0);
            } else {
                // Function continues onwards until targets are met
            }
        } else {
            // Act on the read information relevant to the actionCounter
            // position
            if (actions[actionCounter] == ACCELERATE) {
                vehicle.startAccelerating(targets[actionCounter]);
            } else if (actions[actionCounter] == BREAK) {
                vehicle.startBreaking();
            } else if (actions[actionCounter] == TURN) {
                vehicle.startTurning(targets[actionCounter]);
            }

            // Set the delay
            actionDelay = delays[actionCounter];
            // Increase action counter
            actionCounter++;
        }
    }

    // Check Current Action Completion
    vehicle.checkTargets(lis);

    // Debug the simulated vehicle
    // vehicle.debug();

    // Send the needed messages
    bool odoHasIncreased = vehicle.updateOdometer(lis);
    SAS(vehicle.currentSteeringAngle);
    ABS(vehicle.DSC, vehicle.ABS, vehicle.breakFailure, vehicle.TC);
    EPS(vehicle.EPSOn);
    WHEEL_SPEEDS(vehicle.currentSpeedInKilometerPerHour());
    ODOMETER(odoHasIncreased);
    PCM(vehicle.calculateRPM(), vehicle.currentSpeedInKilometerPerHour(),
        vehicle.accelThrottle);
    PCM_IC(vehicle.engineTemp, odoHasIncreased,vehicle.oilPressureOK,
           vehicle.engineLightOn,vehicle.engineLightBlinking,
           vehicle.lowCoolant,vehicle.batteryCharge);

    // Reduce action counter by 1
    actionDelay--;
}

// END FILE ===================================================================