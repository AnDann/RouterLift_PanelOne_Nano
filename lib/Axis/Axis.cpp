#include <AccelStepper.h>
#include "Axis.h"

// Define default speeds in mm/s
#define HOMING_SPEED 15
#define PROBE_SPEED 8
#define MOVE_SPEED 20
#define PLUNGE_SPEED 4
#define ACCELERATION 100

//Define some safe limits
#define MAX_HOME_DISTANCE 120.0
#define MAX_PROBE_DISTANCE 120.0
#define BACKOFF_DISTANCE 3.0

Axis::Axis(int stepPin, int dirPin, int enablePin, float stepsPerRev, float microsteps, float spindleLead, float minPos, float maxPos, int endstopMin, int endstopMax, int probe) : stepper(AccelStepper::DRIVER, stepPin, dirPin) {
  this->stepsPerRevolution = stepsPerRev;
  this->microsteps = microsteps;
  this->spindleLead = spindleLead;
  this->minPosition = mmToSteps(minPos);
  this->maxPosition = mmToSteps(maxPos);
  this->workOffset = 0.0;
  this->endstopMinPin = endstopMin;
  this->endstopMaxPin = endstopMax;
  this->probingPin = probe;
  this->homingState = NOT_HOMED;
  this->probingState = FINISHED;

  pinMode(endstopMinPin, INPUT_PULLUP);
  pinMode(endstopMaxPin, INPUT_PULLUP);
  pinMode(probingPin, INPUT_PULLUP);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // Aktiviere den Treiber

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(mmToSteps(ACCELERATION));
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
}

void Axis::handle() {
  bool endstopMin, endstopMax, probe;
  // Check endstops
  endstopMin = getEndstopMin();
  endstopMax = getEndstopMax();
  probe = getProbe();

  //Check if homeing required
  if(homingState != FINISHED)
  {
    switch(homingState)
    {
      case NOT_HOMED:
        if(endstopMin)
        {
          homingState = BACKOFF_2;
          stepper.setMaxSpeed(mmToSteps(MOVE_SPEED));
          stepper.move(mmToSteps(BACKOFF_DISTANCE));
        }
        else
        {
          homingState = BACKOFF_1;
        }        
        break;
      case BACKOFF_1:
        if(!endstopMin && !stepper.isRunning())
        {
          homingState = MOVE_FAST;
          stepper.setMaxSpeed(mmToSteps(HOMING_SPEED));
          stepper.move(mmToSteps(-MAX_HOME_DISTANCE));
        }
        else if(!stepper.isRunning())
        {
          homingState = ERROR;
        }
        break;
      case MOVE_FAST:
        if(endstopMin && stepper.isRunning())
        {
          stepper.setCurrentPosition(0);
          stepper.moveTo(0);
          stepper.setSpeed(0);
        }
        else if(endstopMin && !stepper.isRunning())
        {
          homingState = BACKOFF_2;
          stepper.setMaxSpeed(mmToSteps(MOVE_SPEED));
          stepper.move(mmToSteps(BACKOFF_DISTANCE));
        }
        else if(!endstopMin && !stepper.isRunning())
        {
          homingState = ERROR;
        }
        break;
      case BACKOFF_2:
        if(!endstopMin && !stepper.isRunning())
        {
          homingState = MOVE_SLOW;
        }
        else if(!stepper.isRunning())
        {
          homingState = ERROR;
        }
        break;
      case MOVE_SLOW:
        if(!endstopMin && !stepper.isRunning())
        {
          stepper.setMaxSpeed(mmToSteps(HOMING_SPEED / 2));
          stepper.move(-1);
        }
        else if(endstopMin)
        {
          homingState = FINISHED;
          probingState = FINISHED;
          targetPos = 0;
          stepper.setCurrentPosition(0);
          stepper.moveTo(0);
          stepper.setSpeed(0);
        }
        break;
      default:
        break;
    }
  }

  //Check if probing required
  if(probingState != FINISHED && homingState == FINISHED)
  {
    switch(probingState)
    {
      case NOT_HOMED:
        if(probe)
        {
          probingState = BACKOFF_2;
          stepper.setMaxSpeed(mmToSteps(MOVE_SPEED));
          stepper.move(mmToSteps(-BACKOFF_DISTANCE));
        }
        else
        {
          probingState = MOVE_FAST;
          stepper.setMaxSpeed(mmToSteps(PROBE_SPEED));
          stepper.move(mmToSteps(MAX_PROBE_DISTANCE));
        }        
        break;
      case MOVE_FAST:
        if(probe && stepper.isRunning())
        {
          stepper.moveTo(stepper.currentPosition());
          stepper.setSpeed(0);
        }
        else if(probe && !stepper.isRunning())
        {
          probingState = BACKOFF_2;
          stepper.setMaxSpeed(mmToSteps(MOVE_SPEED));
          stepper.move(mmToSteps(-BACKOFF_DISTANCE));
        }
        else if(!probe && !stepper.isRunning())
        {
          probingState = ERROR;
        }
        break;
      case BACKOFF_2:
        if(!probe && !stepper.isRunning())
        {
          probingState = MOVE_SLOW;
        }        
        else if(!stepper.isRunning())
        {
          probingState = ERROR;
        }
        break;
      case MOVE_SLOW:
        if(!probe && !stepper.isRunning())
        {
          stepper.setMaxSpeed(mmToSteps(HOMING_SPEED / 2));
          stepper.move(1);
        }
        else if(probe)
        {
          probingState = FINISHED;
          stepper.moveTo(stepper.currentPosition());
          stepper.setSpeed(0);
          stepper.stop();
          workOffset = stepper.currentPosition();
          targetPos = workOffset;
        }
        break;
      default:
        break;
    }
  }

  // Calculate if travel is allowed
  if(homingState == FINISHED && endstopMin && stepper.distanceToGo() < 0) return;
  else if(homingState == FINISHED && endstopMax && stepper.distanceToGo() > 0) return;
  else if(homingState == ERROR) return;
  else if(probingState == ERROR) return;

  stepper.run();  
}

bool Axis::getEndstopMax()
{
  return digitalRead(endstopMaxPin);
}

bool Axis::getEndstopMin()
{
  return digitalRead(endstopMinPin);
}

bool Axis::getProbe()
{
  return !digitalRead(probingPin);
}

bool Axis::inPosition()
{
  return !stepper.isRunning() && stepper.distanceToGo() == 0;
}

bool Axis::isHomed()
{
  return homingState == FINISHED;
}

bool Axis::isError()
{
  return homingState == ERROR || probingState == ERROR || getEndstopMax() || getEndstopMin();
}

AxisState Axis::getState()
{
  if(homingState != FINISHED) return MOVE_TO_HOME;
  else if(probingState != FINISHED) return MOVE_TO_PROBE;
  else if(stepper.distanceToGo() != 0) return MOVE_TO_TARGET;
  else if(homingState == FINISHED && stepper.distanceToGo() == 0) return INPOSITION;
  else if(homingState == FINISHED && getEndstopMax()) return MAX_REACHED;
  else if(homingState == FINISHED && getEndstopMin()) return MIN_REACHED;
  else return NONE;
}

HomingState Axis::getHomingState()
{
  return homingState;
}

HomingState Axis::getProbingState()
{
  return probingState;
}

void Axis::homing() {
  homingState = NOT_HOMED;
}

void Axis::probing() {
  workOffset = 0.0;
  probingState = NOT_HOMED;
}

void Axis::moveToMax() {
  moveToAbsPos(maxPosition);
}

void Axis::moveToMin() {
  moveToAbsPos(minPosition);
}

void Axis::moveToPos(float position) {
  setTargetPosition(position);
  moveToTarget();
}

void Axis::moveToAbsPos(float position) {
  setAbsTargetPosition(position);
  moveToTarget();
}

float Axis::getCurrentPosition() {
  return stepsToMM(stepper.currentPosition() - workOffset);
}

float Axis::getTargetPosition() {
  return stepsToMM(targetPos - workOffset);
}

float Axis::getWorkoffset() {
  return stepsToMM(workOffset);
}

void Axis::setTargetPosition(float newtargetPos) {
  targetPos = mmToSteps(newtargetPos) + workOffset;
  if (targetPos < (minPosition)) {
    targetPos = (minPosition);
  } else if (targetPos > (maxPosition)) {
    targetPos = (maxPosition);
  }
}

void Axis::setAbsTargetPosition(float newtargetPos) {
  targetPos = mmToSteps(newtargetPos);
  if (targetPos < (minPosition)) {
    targetPos = (minPosition);
  } else if (targetPos > (maxPosition)) {
    targetPos = (maxPosition);
  }
}

void Axis::moveToTarget()
{
  if (homingState != FINISHED || probingState != FINISHED) return;
  stepper.setMaxSpeed(mmToSteps(MOVE_SPEED));
  stepper.moveTo(targetPos);
}

long Axis::mmToSteps(float mm) {
  return mm * (stepsPerRevolution * microsteps) / spindleLead;
}

float Axis::stepsToMM(long steps) {
  return steps * spindleLead / (stepsPerRevolution * microsteps);
}