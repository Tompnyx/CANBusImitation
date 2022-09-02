//
// Created by tompnyx on 11/04/2022.
//
#include "benignGenerator.h"
#include "Vehicle.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <mcp2515_can.h>
#include <SD.h>

// CONSTANTS ==================================================================
// Sets the maximum payload size - CANFD can carry data up to 64 bytes, whereas
// CAN 2.0 can only carry up to 8 bytes
#define MAX_DATA_SIZE 8

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
// How many loops in a second. The target is a 'loop' every 0.0002 seconds.
short lis = 5000;

// The different operations that can be performed
enum Operation { sendRandom, receiveOnly, sendRandomAndReceive, performRoute};
// Change op to set the operation mode
Operation op = performRoute;

// The filename of the json file containing the trip information
const char *filename = "trip.json";
// Make sure to set the correct memory pool in bytes (Inside the <> brackets)
// arduinojson.org/v6/assistant is a useful tool for this.
StaticJsonDocument<512> doc;
// The json file that holds the array of actions
JsonArray trip;
unsigned short actionAmount;
// The current action
unsigned short actionCounter = 0;
// The amount of iterations before the next action is performed
int actionDelay;

// ARDUINO FUNCTIONS ==========================================================

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    // wait for Serial
    while (!SERIAL_PORT_MONITOR);
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

    if (op == performRoute) {
        // Read the json file if needed
        File file = SD.open(filename);
        // Deserialize the json file
        DeserializationError jsonError = deserializeJson(doc, file);

        // Test if the parsing succeeded
        if (jsonError) {
            SERIAL_PORT_MONITOR.print("deserializeJson() failed: ");
            SERIAL_PORT_MONITOR.println(jsonError.f_str());
            return;
        }

        trip = doc["trip"];
        actionAmount = doc["actionAmount"];
    }
}

void loop() {
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

/**
 * Steering Angle Sensor
 *
 * len = 4
 * Byte 1: Unknown
 * Byte 2: Indicator if steering is centered
 * Byte 3/4: The angle of the steering wheel
 *
 * @param angle The angle of the steering wheel(0xFDE1 to 0x021E, 64993 to 542,
 * left to right respectively). It is 0x0000 if centered.
 */
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

/**
 * Anti-lock Breaking System
 *
 * len = 7
 * Byte 4 bit 3: DSC
 * Byte 5 bit 4: ABS
 * Byte 5 bit 7: breakFailure
 * Byte 6 bit 5: TC off
 * Byte 6 bit 6: TC on
 *
 * @param DSC If the Dynamic Stability Control is active
 * @param ABS If the Anti-lock Breaking System is active
 * @param breakFailure If the hand break or breaks have failed
 * @param TC If the Traction Control is active or not
 */
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

/**
 * Electronic Power Steering
 *
 * len = 1
 * Byte 1 bit 8: Determines whether the EPS is on or off (1 for on, 0 for off)
 *
 * @param on Determines whether the EPS is on or off
 */
void EPS(bool EPSOn) {
    unsigned char payload[8] = {0};

    if (EPSOn) {
        payload[0] = 0b10000000;
    }

    send_can(EPSID, false, false, payload, 1);
}

/**
 * The four wheel speeds
 *
 * len = 8
 * Each wheel takes up two bytes, with the order being:
 * Front Left, Front Right, Rear Left, Rear Right.
 * Two CAN messages are sent from this module, with the second being the speed
 * of the four wheels plus 10,000
 *
 * @param speed how fast the vehicle is travelling (in kms)
 */
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

/**
 * The odometer
 *
 * len = 1
 * Byte 1: Changes from 1 to 0 when 1km has been travelled. Used to increment
 * the odometer
 *
 * @param increment To increment the odometer or not
 */
void ODOMETER(bool increment) {
    unsigned char payload[8] = {0};
    if (increment) {
        payload[0] = 0b00000001;
    }
    send_can(ODOMETER_ID, false, false, payload, 1);
}

/**
 * Powertrain Control Module
 *
 * len = 8
 * Byte 1/2: The modified value of the RPM sent (Note - the sent RPM is
 * different - Sent RPM = Actual RPM * 3.85
 * Byte 3/4: Static and set to 0xFF
 * Byte 5/6: The modified value of the km per hour speed of the vehicle (Note -
 * the sent KMPH is different - Sent KMPH = (Actual KMPH * 100) + 10,000)
 * Byte 7: The pressure applied to the throttle pedal (Note - range
 * accounts for 0 to 0xC8 in 0.5% increments)
 * Byte 8: Static and set to 0xFF
 *
 * @param RPM The real RPM of the vehicle
 * @param KMPH The real kilometers per hour of the vehicle
 * @param throttle The pressure applied to the throttle pedal (Note - range
 * accounts for 0 to 0xC8 in 0.5% increments)
 */
void PCM(double RPM, unsigned short KMPH, unsigned short throttle) {
    unsigned char payload[8] = {0};

    // Convert RPM to the modified value and Big Endian
    RPM = RPM * 3.85;
    payload[0] = (short) RPM >> 8;
    payload[1] = (short) RPM & 0xFF;

    // Static bytes
    payload[2] = 0xFF;
    payload[3] = 0xFF;

    // Convert KMPH to the modified value and Big Endian
    KMPH = (KMPH * 100) + 10000;
    payload[4] = (short) KMPH >> 8;
    payload[5] = (short) KMPH * 0xFF;

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

/**
 * The Powertrain Control Module Instrument Cluster
 *
 * len = 7
 * Byte 1: Contains the engine temperature
 * Byte 2: Contains the odometer value
 * Byte 3/4: Unknown, set to 0
 * Byte 5: If the oil pressure is okay (0 for fault, >=1 for OK)
 * Byte 6 bit 7: If the engine light is on
 * Byte 6 bit 8: If the engine light is blinking
 * Byte 7 bit 2: If the coolant is low
 * Byte 7 bit 7: If the battery charge is low
 * Byte 7 bit 8: If the oil pressure light is on
 *
 * @param engine_temp What the engine temperature is (Between 128 and 192)
 * @param odometer_increment If the odometer should be incremented
 * @param oilPressureOK If the oil pressure is OK
 * @param engineLightOn If the engine light is on
 * @param engineBlinking If the engine light is blinking
 * @param lowCoolant If the coolant fluid is low
 * @param batteryCharge If the battery has low charge/ no charge
 */
void PCM_IC(unsigned short engineTemp, bool odometer_increment, bool oilPressureOK,
            bool engineLightOn, bool engineLightBlinking, bool lowCoolant,
            bool batteryCharge) {
    unsigned char payload[8] = {0};

    // Check the range of the engine temperature
    if (engineTemp < 128) {
        payload[0] = 128;
    } else if (engineTemp > 192) {
        payload[0] = 192;
    } else {
        payload[0] = engineTemp;
    }

    if (odometer_increment) {
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

/**
 * Needed as there is no multi-threading present with Arduino. Certain
 * processes need to update gradually or at the same time.
 */
void overview_vehicle_functionality_loop() {
    Vehicle vehicle;
    // Check to see if there is a current delay. If not, the next action will
    // be read and performed.

    if (actionDelay == 0) {
        if (actionCounter >= actionAmount) {
            // Trip is done
            exit(0);
        } else {
            if (strncmp(trip[actionCounter]["action"], "accelerate",
                        10) == 0) {
                vehicle.startAccelerating(trip[actionCounter]["target"]);
            } else if (strncmp(trip[actionCounter]["action"], "turn",
                               4) == 0) {
                vehicle.startTurning(trip[actionCounter]["target"]);
            } else if (strncmp(trip[actionCounter]["action"], "break",
                               5) == 0) {
                vehicle.startBreaking();
            }
            // Set delay and increase action counter
            actionDelay = trip[actionCounter]["delay"];
            actionCounter++;
        }
    }

    // Check Current Action Completion
    vehicle.checkTargets(lis);

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
}

// END FILE ===================================================================