#include "VarSpeedServo.h"

#include <util/atomic.h>
#include "Logger.h"
#include "Arduino.h"

namespace
{
    Logger logger("VarSpeedServo");
}

/**
 *
 */
VarSpeedServo::VarSpeedServo(
    int step,
    int dir,
    int hs,
    float maxAngleVelocity,
    int encA,
    int encB,
    float minRadAngle,
    float maxRadAngle,
    AccelStepperEncoder & _AccelStepper,
    Encoder & _Encoder,
    float homeRadAngle):
    _AccelStepper(_AccelStepper),
    _Encoder(_Encoder)
{
    this->step = step;
    this->dir = dir;
    this->hs = hs;
    this->maxAngleVelocity = maxAngleVelocity; // per s
    this->currentAngleVelocity = maxAngleVelocity;
    this->lastUpdate = micros();

    this->minRadAngle = minRadAngle;
    this->maxRadAngle = maxRadAngle;

    this->_AccelStepper = _AccelStepper;
    this->_Encoder = _Encoder;

    if (minRadAngle > maxRadAngle)
    {
        logger.error("minAngle must be smaller than maxAngle on servo " + String(step) + " initialization (min: " + String(minRadAngle / PI * 180) + " max: " + String(maxRadAngle / PI * 180) + ")");
    }

    if (minRadAngle > maxRadAngle)
    {
        logger.error("minRadAngle must be smaller than maxRadAngle. Servo pin number: " + String(step));
    }

    this->startAngle = homeRadAngle;
    this->currentAngle = homeRadAngle;
    this->targetAngle = homeRadAngle;
    this->homeAngle = homeRadAngle;

    this->encA = encA;
    this->encB = encB;

    logger.info("SM " + String(step) + " " + String(dir));

    if (this->step < 0)
        this->virtualServo = true;

    // if (!this->virtualServo)
    // {
    //     this->servo.attach(pinNumber);
    // }

    this->move(); // drive to 0 position according to min max angle
}

float VarSpeedServo::getHomeRadAngle()
{
    return this->homeAngle;
}

int VarSpeedServo::getStep()
{
    return this->step;
}

int VarSpeedServo::getDir()
{
    return this->dir;
}

int VarSpeedServo::getEncA()
{
    return this->encA;
}

int VarSpeedServo::getEncB()
{
    return this->encB;
}

int VarSpeedServo::getHS()
{
    return this->hs;
}

void VarSpeedServo::setAngleLimits(float minRadAngle, float maxRadAngle)
{
    if (minRadAngle > maxRadAngle)
    {
        logger.error("minAngle must be smaller than maxAngle on servo_num: " + String(this->step) + " (min: " + String(minRadAngle / PI * 180) + " max: " + String(maxRadAngle / PI * 180) + ")");
    }
    this->minRadAngle = minRadAngle;
    this->maxRadAngle = maxRadAngle;
}

float VarSpeedServo::getMinRadAngle()
{
    return this->minRadAngle;
}

float VarSpeedServo::getMaxRadAngle()
{
    return this->maxRadAngle;
}

float VarSpeedServo::getCurrentAngle()
{ // physical angle
    float result;

    // std::cout << "getCurrentAngle "<<currentAngle << '\n';

    // current angle may change whe process interrupt kicks in
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = this->currentAngle;
    }
   
    logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") getCurrentAngle "+String(result));
    return result;
}

void VarSpeedServo::setTargetRadAngle(float angleRad)
{
    this->startAngle = this->currentAngle;
    this->elapsedTime = 0;
    this->targetAngle = angleRad;
    logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") setTargetRadAngle "+ String(this->targetAngle));
    
    this->_AccelStepper.setSpeed(1000);
    if (this->_AccelStepper.currentPosition() < 1000) {
        this->_AccelStepper.moveTo(10000);
    } else {
       this->_AccelStepper.moveTo(-10000);
    }
    this->_AccelStepper.setAcceleration(1000);
    logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") setTargetRadAngle stepper current pos "+String(this->_AccelStepper.currentPosition()));
  }

float VarSpeedServo::getTargetRadAngle()
{
    return this->targetAngle;
}

void VarSpeedServo::setCurrentAngleVelocity(float angleRadVelocity)
{
    if (angleRadVelocity > this->maxAngleVelocity)
    {
        this->currentAngleVelocity = this->maxAngleVelocity;
    }
    else if (angleRadVelocity <= 0)
    {
        this->currentAngleVelocity = 0.1;
    }
    else
    {
        this->currentAngleVelocity = angleRadVelocity;
    }
}

float VarSpeedServo::getCurrentAngleVelocity()
{
    return this->currentAngleVelocity;
}

float VarSpeedServo::getMaxAngleVelocity()
{
    return this->maxAngleVelocity;
}

void VarSpeedServo::runCalibration() {
    this->calibrationMode = 1;
    logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Calibration mode");
    this->_AccelStepper.setMaxSpeed(3000.0);
    this->_AccelStepper.setAcceleration(3000.0);
    this->_AccelStepper.moveTo(200000); 
}

unsigned int VarSpeedServo::process(unsigned int deltaT)
{
    if (this->calibrationMode > 0 && this->calibrationMode < 4) {
        // Calibration mode

        int hallSensorValue = map(analogRead(hs), 0, 1023, 0, 255);
        int hs;
        if (hallSensorValue > 128) {
            hs = 0;
        } else {
            hs = 1;
        }

        if (this->calibrationMode == 1 && hs == 1) {
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - HS");
            this->calibrationMode = 2;
            this->_AccelStepper.moveTo(this->_AccelStepper.currentPosition() - 1000);
        }
        if (this->calibrationMode == 3 && hs == 1) {
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - HS. At home");
            this->calibrationMode = 4;
            this->_AccelStepper.stop();
            this->_AccelStepper.writeEnc(0);
            this->_AccelStepper.synchroniseMotorWithEncoder();    
            this->currentAngle = 0;
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Encoder and current angle reseted");
        } 
        
        if (this->_AccelStepper.distanceToGo() == 0) {
            if (this->calibrationMode == 2) {
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Slowly return back to home");
            this->_AccelStepper.setSpeed(100.0);
            this->calibrationMode = 3;
            }
        }

        if (this->calibrationMode == 1 || this->calibrationMode == 2) {
            this->_AccelStepper.run();
        }
        if (this->calibrationMode == 3) {
            this->_AccelStepper.runSpeed();
        }
    }

    if (this->calibrationMode != 4) {
        return; // not calibrated
    }

    // v = s/t
    this->elapsedTime += deltaT;

    float deltaAngle = this->currentAngleVelocity * this->elapsedTime / 1000.0;

    // logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") process.deltaAngle "+String(deltaAngle));
      // Serial.println(deltaAngle);

    if (fabs(deltaAngle) > fabs(this->targetAngle - this->startAngle))
    {
        this->currentAngle = this->targetAngle;

        // set start to target to not move again on elapsed time. overflow ~50 days?
        this->startAngle = this->targetAngle;
    }
    else
    {
        if (this->targetAngle > this->startAngle)
        {
            this->currentAngle = this->startAngle + deltaAngle;
        }
        else
        {
            this->currentAngle = this->startAngle - deltaAngle;
        }
    }

    // logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") call move");
    return this->move();
}

bool VarSpeedServo::atTargetAngle()
{
    bool atTargetAngle = fabs(this->currentAngle - this->targetAngle) < 0.000001;
    atTargetAngle = this->_AccelStepper.distanceToGo() == 0;
    if(atTargetAngle == 1) {
        logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") atTargetAngle. encoder="
            + String(this->_AccelStepper.readEnc()) + ". deviation=" + String(this->_AccelStepper.computeDeviation()));
    }

    return atTargetAngle;
}

unsigned int VarSpeedServo::move()
{
    
        // unsigned int freq = int(
        // this->map_float(this->currentAngle, this->minRadAngle, this->maxRadAngle, this->minFreq, this->maxFreq));

    // if (!this->virtualServo) this->servo.writeMicroseconds(freq);
    
    //if (!this->_AccelStepper.isRunning()) {
        //logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") move");
        this->_AccelStepper.run();
        //logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") move recorded");
    //}
    return 0;
}

float VarSpeedServo::map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool VarSpeedServo::getOutOfRange()
{
    return this->outOfRange;
}
