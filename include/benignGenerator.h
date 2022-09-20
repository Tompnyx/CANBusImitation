//
// Created by Tompnyx on 11/04/2022.
//

#ifndef benignGenerator_h
#define benignGenerator_h
#include <Arduino.h>
#include "benignGenerator_dfs.h"

/**
 * Initialises the JSON object needed to deserialize the JSON file. This is
 * performed if the performRoute mode is selected.
 */
void initialiseJsonObject();

/**
 * Generates a random CAN ID.
 *
 * @param id The CAN ID variable to change
 * @param ext If the CAN message is using an extended frame
 */
void generate_random_id(unsigned long &id, bool ext);

/**
 * Generates a random payload of a random length.
 *
 * @param len The length of the payload variable to change
 * @param payload The payload variable to change
 * @param ext If the CAN message is using an extended frame
 * @param rtr If the CAN message is a remote request frame
 */
void generate_random_payload(byte &len, unsigned char *payload, bool ext,
                             bool rtr);

/**
 * Generates a random CAN message.
 *
 * @param id The CAN ID variable to change
 * @param ext If the CAN message is using an extended frame
 * @param rtr If the CAN message is a remote request frame
 * @param len The length of the payload variable to change
 * @param payload The payload variable to change
 */
void generate_random_message(unsigned long &id,
                             byte &ext,
                             byte &rtr,
                             byte &len,
                             unsigned char *payload);

/**
 * Prints a CAN message to the SERIAL monitor.
 *
 * @param canId The CAN ID to print
 * @param len The length of the CAN payload
 * @param buf The CAN payload to print
 */
void print_can_message_to_monitor(unsigned long canId,
                                  byte len,
                                  unsigned char *buf);

/**
 * Check to see if the CAN message was sent successfully or not.
 *
 * @param report The report generated when sending the CAN message
 * @return If the message was sent successfully or not
 */
bool check_message_sent(int report);

/**
 * Sends a random or empty CAN message.
 *
 * @param sendRandom True if a random message should be sent, False if an
 * empty message should be sent.
 */
void send_can(bool sendRandom);

/**
 * Sends a CAN message with a randomly generated payload.
 *
 * @param id The ID of the message to send
 * @param ext_condition If the CAN message is using an extended frame
 * @param rtr_condition If the CAN message is a remote request frame
 */
void send_can(unsigned long id, bool ext_condition, bool rtr_condition);

/**
 * Sends a CAN message.
 *
 * @param id The ID of the message to send
 * @param ext_condition If the CAN message is using an extended frame
 * @param rtr_condition If the CAN message is a remote request frame
 * @param payload The payload of the message to send
 * @param len The length of the payload
 */
void send_can(unsigned long id, bool ext_condition, bool rtr_condition,
              unsigned char *payload, byte len);

/**
 * Receive a CAN message and print it to the SERIAL terminal.
 */
void receive_can();

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
void SAS(unsigned short angle);

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
void ABS(bool DSC, bool ABS, bool breakFailure, bool TC);

/**
 * Electronic Power Steering
 *
 * len = 1
 * Byte 1 bit 8: Determines whether the EPS is on or off (1 for on, 0 for off)
 *
 * @param on Determines whether the EPS is on or off
 */
void EPS(bool EPSOn);

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
void WHEEL_SPEEDS(unsigned short speed);

/**
 * The odometer
 *
 * len = 1
 * Byte 1: Changes from 1 to 0 when 1km has been travelled. Used to increment
 * the odometer
 *
 * @param increment To increment the odometer or not
 */
void ODOMETER(bool increment);

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
 * @param kmph The real kilometers per hour of the vehicle
 * @param throttle The pressure applied to the throttle pedal (Note - range
 * accounts for 0 to 0xC8 in 0.5% increments)
 */
void PCM(double RPM, unsigned short kmph, unsigned short throttle);

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
 * @param engineTemp What the engine temperature is (Between 128 and 192)
 * @param odometerIncrement If the odometer should be incremented
 * @param oilPressureOK If the oil pressure is OK
 * @param engineLightOn If the engine light is on
 * @param engineLightBlinking If the engine light is blinking
 * @param lowCoolant If the coolant fluid is low
 * @param batteryCharge If the battery has low charge/ no charge
 */
void PCM_IC(unsigned short engineTemp, bool odometerIncrement,
            bool oilPressureOK, bool engineLightOn, bool engineLightBlinking,
            bool lowCoolant, bool batteryCharge);

/**
 * Needed as there is no multi-threading present with Arduino. Certain
 * processes need to update gradually or at the same time.
 */
void overview_vehicle_functionality_loop();

#endif