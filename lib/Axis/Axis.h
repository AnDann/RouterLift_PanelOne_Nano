#ifndef AXIS_H
#define AXIS_H

#include <AccelStepper.h>

typedef enum
{
    NONE,
    MOVE_TO_TARGET,
    MOVE_TO_HOME,
    MOVE_TO_PROBE,
    INPOSITION,
    MAX_REACHED,
    MIN_REACHED 
} AxisState;

typedef enum
{
    NOT_HOMED,
    BACKOFF_1,
    MOVE_FAST,
    BACKOFF_2,
    MOVE_SLOW,
    FINISHED ,
    ERROR
} HomingState;

class Axis {
private:
    AccelStepper stepper;
    float stepsPerRevolution;
    float microsteps;
    float spindleLead;
    float minPosition;
    float maxPosition;
    int endstopMinPin;
    int endstopMaxPin;
    int probingPin;
    long workOffset;
    AxisState state;
    HomingState homingState;
    HomingState probingState;
    long targetPos;
public:
    Axis(int stepPin, int dirPin, int enablePin, float stepsPerRev, float microsteps, float spindleLead, float minPos, float maxPos, int endstopMin, int endstopMax, int probing);

    void homing();
    bool isHomed();
    bool isError();
    bool inPosition();
    bool getEndstopMax();
    bool getEndstopMin();
    bool getProbe();
    AxisState getState();
    HomingState getHomingState();
    HomingState getProbingState();
    void handle();
    void probing();
    void moveUp(float distance);
    void moveDown(float distance);
    void moveToPos(float position);
    float getCurrentPosition();
    float getTargetPosition();
    float getWorkoffset();
    void setTargetPosition(float targetPos);
    void moveToTarget();

private:
    long mmToSteps(float mm);
    float stepsToMM(long steps);

};

#endif  // AXIS_H