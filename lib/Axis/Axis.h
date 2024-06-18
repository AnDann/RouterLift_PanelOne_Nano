#ifndef AXIS_H
#define AXIS_H

#include <AccelStepper.h>  // Include AccelStepper library

// Enumeration for different states of the axis
typedef enum {
    NONE,           // No specific state
    MOVE_TO_TARGET, // Moving to target position
    MOVE_TO_HOME,   // Homing process
    MOVE_TO_PROBE,  // Moving to probe point
    INPOSITION,     // In position reached
    MAX_REACHED,    // Maximum position reached
    MIN_REACHED     // Minimum position reached
} AxisState;

// Enumeration for different homing states
typedef enum {
    NOT_HOMED,  // Not homed
    BACKOFF_1,  // Backoff step 1
    MOVE_FAST,  // Fast movement
    BACKOFF_2,  // Backoff step 2
    MOVE_SLOW,  // Slow movement
    FINISHED,   // Finished homing
    ERROR       // Error occurred
} HomingState;

class Axis {
private:
    AccelStepper stepper;   // AccelStepper object for motor control
    float stepsPerRevolution;   // Steps per revolution of the motor
    float microsteps;           // Microsteps of the motor
    float spindleLead;          // Lead of the motor per revolution in mm
    float minPosition;          // Minimum reachable position of the axis in mm
    float maxPosition;          // Maximum reachable position of the axis in mm
    int endstopMinPin;          // Pin for minimum endstop
    int endstopMaxPin;          // Pin for maximum endstop
    int probingPin;             // Pin for probe point
    long workOffset;            // Work offset of the axis
    AxisState state;            // Current state of the axis
    HomingState homingState;    // Homing state of the axis
    HomingState probingState;   // Probing state of the axis
    long targetPos;             // Target position of the axis

    void setupTimer();  // Private method for timer initialization

public:
    // Constructor of the class
    Axis(int stepPin, int dirPin, int enablePin, float stepsPerRev, float microsteps, float spindleLead, float minPos, float maxPos, int endstopMin, int endstopMax, int probing);

    // Methods for controlling the axis
    void homing();              // Start homing process
    bool isHomed();             // Check if axis is homed
    bool isError();             // Check if error occurred
    bool inPosition();          // Check if axis is in position
    bool getEndstopMax();       // Read maximum endstop
    bool getEndstopMin();       // Read minimum endstop
    bool getProbe();            // Read probe point
    AxisState getState();       // Get current state of the axis
    HomingState getHomingState();// Get current homing state of the axis
    HomingState getProbingState(); // Get current probing state of the axis
    void handle();              // Handle current state of the axis
    void probing();             // Start probing process
    void moveToMax();           // Move axis to maximum position
    void moveToMin();           // Move axis to minimum position
    void moveToWorkpiece();     // Move axis to workpiece (added new method)
    void moveToPos(float position);  // Move axis to a specific position
    float getCurrentPosition(); // Get current position of the axis
    float getTargetPosition();  // Get target position of the axis
    float getWorkoffset();     // Get work offset of the axis
    void setTargetPosition(float targetPos);  // Set target position of the axis
    void moveToTarget();       // Move axis to the target position

private:
    void moveToAbsPos(long position);   // Move axis to an absolute position
    void setAbsTargetPosition(long targetPos);   // Set absolute target position of the axis
    // Private methods for converting mm to steps and vice versa
    long mmToSteps(float mm);
    float stepsToMM(long steps);
};

#endif  // AXIS_H
