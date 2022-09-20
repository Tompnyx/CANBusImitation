//
// Created by Tompnyx on 20/07/2022.
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

    /**
     * @return The RPM of the vehicle
     */
    double calculateRPM() const;

    /**
     * @return The current speed of the vehicle in kilometers per hour
     */
    unsigned short currentSpeedInKilometerPerHour() const;

    /**
     * Check if the acceleration target has been met.
     *
     * @param lis The number of loops that the arduino performs in a second
     */
    void checkAccelerationTarget(short lis);

    /**
     * Check if the breaking target has been met.
     *
     * @param lis The number of loops that the arduino performs in a second
     */
    void checkBreakingTarget(short lis);

    /**
     * Check if the steering target has been met.
     */
    void checkSteeringTarget();

    /**
     * Check to see if all targets have been met.
     *
     * @param lis The number of loops that the arduino performs in a second
     */
    void checkTargets(short lis);

    /**
     * Used to accelerate the vehicle.
     *
     * @param targetSpeed The target speed to accelerate up to
     */
    void startAccelerating(unsigned short targetSpeed);

    /**
     * Used to break the vehicle.
     */
    void startBreaking();

    /**
     * Used to turn the vehicle.
     *
     * @param angle The angle to turn up to
     */
    void startTurning(unsigned short angle);

    /**
     * Used to update the odometer after a kilometer has passed.
     *
     * @param lis The number of loops that the arduino performs in a second
     * @return If the odometer needed to be incremented or not
     */
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

    /**
     * Updates the breaking value. Used to incrementally break.
     *
     * @param lis The number of loops that the arduino performs in a second
     */
    void updateBreaking(short lis);

    /**
     * Updates the accelerating value. Used to incrementally accelerate.
     *
     * @param lis The number of loops that the arduino performs in a second
     */
    void updateSpeed(short lis);

    /**
     * Updates the turning value. Used to incrementally turn.
     */
    void updateTurning();
};

#endif