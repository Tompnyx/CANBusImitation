//
// Created by tompnyx on 11/04/2022.
//

#ifndef CLIONARDUINO_GENERATEBENIGNDATA_H
#define CLIONARDUINO_GENERATEBENIGNDATA_H
#include <Arduino.h>
#include "generateBenignData_dfs.h"


void generate_random_id(unsigned long &id, bool ext);
void generate_random_payload(byte &len, unsigned char *payload, bool ext,
                             bool rtr);
void generate_random_message(unsigned long &id,
                             byte &ext,
                             byte &rtr,
                             byte &len,
                             unsigned char *payload);
void print_can_message_to_monitor(unsigned long canId,
                                  byte len,
                                  unsigned char *buf);
void send_can(bool sendRandom);
void send_can(unsigned long id, bool ext_condition, bool rtr_condition);
void send_can(unsigned long id, bool ext_condition, bool rtr_condition,
              unsigned char *payload, byte len);
void receive_can();
void SAS(unsigned short angle, bool centered);
void ABS(bool DSC, bool ABS, bool breakFailure, bool TC);
void EPS(bool EPSOn);
void WHEEL_SPEEDS(unsigned short speed);
void ODOMETER(bool increment);
void PCM(double RPM, int KMPH, unsigned short throttle);
void PCM_IC(short engine_temp, byte odometer_increment, bool oilPressureOK,
            bool engineOn, bool engineBlinking, bool lowCoolant,
            bool batteryCharge);
#endif //CLIONARDUINO_GENERATEBENIGNDATA_H
