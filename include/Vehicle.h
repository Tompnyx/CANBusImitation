//
// Created by tompnyx on 20/07/2022.
//

#ifndef Vehicle_h
#define Vehicle_h
#include "Arduino.h"

// List of Car dimensions
#define WHEEL_RADIUS 1 // In meters

class Vehicle {
public:
    Vehicle();

    // If the vehicle is accelerating or not
    bool accelerating;
    // If the vehicle is breaking or not
    bool breaking;
    // If the vehicle is turning or not
    bool turning;

    // The throttle to acceleration ratio
    double accelThrottleRatio;
    // The throttle to breaking ratio
    double breakThrottleRatio;
    // The turning wheel to turning ratio
    double wheelTurnRatio;
    // If the Dynamic Stability Control is active
    bool DSC;
    // If the Anti-lock Breaking System is active
    bool ABS;
    // If the hand break or breaks have failed
    bool breakFailure;
    // If the Traction Control is active or not
    bool TC;
    // If the Electronic Power Steering is on
    bool EPSOn;
    // What the engine temperature is (Between 128 and 192);
    unsigned short engineTemp;
    // If the oil pressure is OK
    bool oilPressureOK;
    // If the engine light is on
    bool engineLightOn;
    // If the engine light is blinking
    bool engineLightBlinking;
    // If the coolant fluid is low
    bool lowCoolant;
    // If the battery has low charge/ no charge
    bool batteryCharge;

    // The current speed of the vehicle
    unsigned short currentSpeed;
    // The angle of the vehicle between 64993 and 542 (left to right). It is 0
    // if centered.
    unsigned short currentSteeringAngle;

    // The accelerating throttle value (between 0 and 200)
    unsigned short accelThrottle;
    // The breaking throttle value (between 0 and 200)
    unsigned short breakThrottle;

    double calculateRPM() const;
    unsigned short currentSpeedInKilometerPerHour() const;
    void checkAccelerationTarget(short lis);
    void checkBreakingTarget(short lis);
    void checkSteeringTarget();
    void checkTargets(short lis);
    void startAccelerating(unsigned short targetSpeed);
    void startBreaking();
    void startTurning(unsigned short angle);
    bool updateOdometer(short lis);

private:
    // The target speed of the vehicle
    unsigned short _accelTargetSpeed;
    // The target speed of the vehicle when breaking
    unsigned short _breakTargetSpeed;
    // The distance travelled in kilometers by the vehicle
    double _odometer;
    // The target angle of the vehicle
    unsigned short _steeringTargetAngle;

    void updateBreaking(short lis);
    void updateSpeed(short lis);
    void updateTurning();
};
#endif