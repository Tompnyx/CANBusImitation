//
// Created by Tompnyx on 20/07/2022.
//
#include "Arduino.h"
#include "Vehicle.h"

Vehicle::Vehicle() {
    accelerating = false;
    breaking = false;
    turning = false;

    // Generate a random ratio to equate the throttle to acceleration ratio to
    accelThrottleRatio = ((double) random(1, 95)) / 100;
    // Generate a random ratio to equate the throttle to breaking ratio to
    breakThrottleRatio = ((double) random(1, 95)) / 100;
    // Generate a random ratio to equate turning the wheel to
    wheelTurnRatio = ((double) random(1, 100)) / 100;
    DSC = false;
    ABS = false;
    breakFailure = false;
    TC = false;
    EPSOn = false;

    // Generate random value for the temperature of the engine
    engineTemp = (unsigned short) random(128, 192);
    oilPressureOK = true;
    engineLightOn = false;
    engineLightBlinking = false;
    lowCoolant = false;
    batteryCharge = false;
    currentSpeed = 0;
    accelThrottle = 0;
    breakThrottle = 0;

    currentSpeed = 0;
    currentSteeringAngle = 0;
    _accelTargetSpeed = 0;
    _breakTargetSpeed = 0;
    // Generate a random value for the odometer
    _odometer = (float) random(0, 200000);
    _steeringTargetAngle = 0;
}

double Vehicle::calculateRPM() const {
    return (60 / (2 * PI * WHEEL_RADIUS)) * currentSpeed;
}

void Vehicle::checkAccelerationTarget(short lis) {
    if (currentSpeed >= _accelTargetSpeed) {
        accelerating = false;
        _accelTargetSpeed = 0;
    } else {
        updateSpeed(lis);
    }
}

void Vehicle::checkBreakingTarget(short lis) {
    if (currentSpeed <= _breakTargetSpeed) {
        breaking = false;
        currentSpeed = 0;
    } else {
        updateBreaking(lis);
    }
}

void Vehicle::checkSteeringTarget() {
    if (currentSteeringAngle == 0) currentSteeringAngle = 32225;
    if (currentSteeringAngle == _steeringTargetAngle) {
        turning = false;
        _steeringTargetAngle = 0;
    } else {
        updateTurning();
    }
    if (currentSteeringAngle == 32225) currentSteeringAngle = 0;
}

void Vehicle::checkTargets(short lis) {
    if (accelerating) checkAccelerationTarget(lis);
    if (breaking) checkBreakingTarget(lis);
    if (turning) checkSteeringTarget();
}

void Vehicle::debug() const {
    SERIAL_PORT_MONITOR.println("-----------------------------");
    SERIAL_PORT_MONITOR.print("Accelerating: ");
    SERIAL_PORT_MONITOR.println(accelerating);
    SERIAL_PORT_MONITOR.print("Breaking: ");
    SERIAL_PORT_MONITOR.println(breaking);
    SERIAL_PORT_MONITOR.print("Turning: ");
    SERIAL_PORT_MONITOR.println(turning);
    SERIAL_PORT_MONITOR.print("Current Speed: ");
    SERIAL_PORT_MONITOR.println(currentSpeed);
    SERIAL_PORT_MONITOR.print("Current Turning Angle: ");
    SERIAL_PORT_MONITOR.println(currentSteeringAngle);
    SERIAL_PORT_MONITOR.print("Accelerating goal: ");
    SERIAL_PORT_MONITOR.println(_accelTargetSpeed);
    SERIAL_PORT_MONITOR.print("Turning goal: ");
    SERIAL_PORT_MONITOR.println(_steeringTargetAngle);
}

unsigned short Vehicle::currentSpeedInKilometerPerHour() const {
    return currentSpeed * 60 * 60 / 1000;
}

void Vehicle::startAccelerating(unsigned short targetSpeed) {
    // Generate a random value between 1 and 200 for the accelerating throttle
    // range to be set at
    accelThrottle = (short) random(0x01, 0xC8);
    _accelTargetSpeed = targetSpeed;
    accelerating = true;
    breaking = false;
}

void Vehicle::startBreaking() {
    // Generate a random value between 1 and 200 for the breaking throttle
    // range to be set at
    breakThrottle = (short) random(0x01, 0xC8);
    breaking = true;
    accelerating = false;
    _accelTargetSpeed = 0;
}

void Vehicle::startTurning(unsigned short angle) {
    // The angle of the vehicle has to be between 64993 and 542 (left to
    // right). It is 0 if centered.
    if (angle == 0) {
        angle = 32225;
    } else if (angle < 542) {
        angle = 542;
    } else if (angle > 64993) {
        angle = 64993;
    }

    _steeringTargetAngle = angle;
    turning = true;
}

void Vehicle::updateBreaking(short lis) {
    currentSpeed -= (unsigned short) ceil(currentSpeed * breakThrottle
            * breakThrottleRatio / (0xC8 * lis)) * 5;
}

bool Vehicle::updateOdometer(short lis) {
    double newOdometer = _odometer + ((double) currentSpeed) / lis;
    bool hasIncremented = (floor(_odometer) < floor(newOdometer));
    _odometer = newOdometer;
    return hasIncremented;
}

void Vehicle::updateSpeed(short lis) {
    currentSpeed += (unsigned short)  ceil(_accelTargetSpeed * accelThrottle
            * accelThrottleRatio / (0xC8 * lis)) * 5;
}

void Vehicle::updateTurning() {
    // Turning that speeds up and down depending on distance
    int difference = ((int) _steeringTargetAngle - (int) currentSteeringAngle);
    if (difference > 0) {
        currentSteeringAngle += ceil((double) difference / 10);
    } else {
        currentSteeringAngle -= ceil((double) -difference / 10);
    }
}

bool Vehicle::vehicleTargetsMet() const {
    return (!(turning || breaking || accelerating));
}