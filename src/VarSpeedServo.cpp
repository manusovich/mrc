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
    float homeRadAngle,
    int direction,
    int moveDirection,
    float revSteps,
    float revPulses):
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
    this->direction = direction;
    this->moveDirection = moveDirection;
    this->revSteps = revSteps;
    this->revPulses = revPulses;
    this->encoder_motor_ratio = revSteps / revPulses;
    this->motor_encoder_ratio = revPulses / revSteps;

    if (minRadAngle > maxRadAngle)
    {
        logger.error("minAngle must be smaller than maxAngle on servo " 
            + String(step) + " initialization (min: " + String(minRadAngle / PI * 180) 
            + " max: " + String(maxRadAngle / PI * 180) + ")");
    }

    if (minRadAngle > maxRadAngle)
    {
        logger.error("minRadAngle must be smaller than maxRadAngle. Servo pin number: " 
            + String(step));
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
        logger.error("minAngle must be smaller than maxAngle on servo_num: " 
            + String(this->step) + " (min: " + String(minRadAngle / PI * 180) + " max: " 
            + String(maxRadAngle / PI * 180) + ")");
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
   
    return result;
}

unsigned int VarSpeedServo::getCurrentEncPos()
{ 
    return this->_AccelStepper.readEnc();
}

void VarSpeedServo::setTargetRadAngle(float angleRad)
{
    this->startAngle = this->currentAngle;
    this->elapsedTime = 0;
    this->targetAngle = angleRad;
    this->targetEncPosition = this->revPulses / (2 * PI) * angleRad * this->moveDirection; 
    this->targetSteps = this->targetEncPosition * this->encoder_motor_ratio;

    logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") setTargetRadAngle "
    + String(this->targetAngle) + ", setTargetEncPosition=" + String(this->targetEncPosition)
    +", setTargetMotorSteps=" + String(this->targetSteps) + " encoder_motor_ratio=" + String(this->encoder_motor_ratio));
    
    this->_AccelStepper.moveTo(this->targetSteps);
    this->_AccelStepper.setSpeed(500);
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
    this->_AccelStepper.move(-100000 * this->direction);
    this->_AccelStepper.setSpeed(-2000.0 * this->direction);
}

unsigned int VarSpeedServo::process(unsigned int deltaT)
{
    if (this->calibrationMode > 0 && this->calibrationMode < 5) {
        // Calibration mode

        int hallSensorValue = analogRead(this->hs);
        int hsv = 0;

        if (this->step == 2) {
           if (hallSensorValue < 512) {
                hsv = 1;
            } 
        } else {
            if (hallSensorValue > 512) {
                hsv = 1;
            } 
        }

        if (this->calibrationMode == 1 && hsv == 1) {
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - HS");
            this->calibrationMode = 2;
            this->_AccelStepper.move(1000 * this->direction);
            this->_AccelStepper.setSpeed(1000 * this->direction);
        }
        if (this->calibrationMode == 3 && hsv == 1) {
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - HS. At home");
            this->calibrationMode = 4;
            this->_AccelStepper.stop();
            this->_AccelStepper.setCurrentPosition(0);
            this->currentAngle = 0;
            this->targetAngle = 0;

            if (this->step == 0) {
                // j1
                this->_AccelStepper.move(32000 * this->direction); // 180 deg 
            }
            
            if (this->step == 2) {
                // j2
                this->_AccelStepper.move(10800 * this->direction); // 180deg 
            }

            if (this->step == 4) {
                // j3
                this->_AccelStepper.move(14000 * this->direction); // 45deg 
            }

            if (this->step == 6) {
                // j4
                this->_AccelStepper.move(28500 * this->direction); // 180deg
            }

             if (this->step == 8) {
                // j5
                this->_AccelStepper.move(4500 * this->direction); // 180deg
            }

            if (this->step == 10) {
                // j6
                this->_AccelStepper.move(13650 * this->direction); // 180deg
            }

            this->_AccelStepper.setAcceleration(2000);
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Encoder and current angle reseted");
            this->calibrationMode = 4; 
        } 
        
        if (this->_AccelStepper.distanceToGo() == 0) {
            if (this->calibrationMode == 2) {
                logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Slowly return back to home");
                this->_AccelStepper.move(-1050 * this->direction);
                this->_AccelStepper.setSpeed(-300.0 * this->direction);
                this->calibrationMode = 3;
            }
            if (this->calibrationMode == 4) {          
                this->_AccelStepper.stop();

                logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") atTargetAngle. motor = "+String(this->_AccelStepper.currentPosition())+" encoder="
            + String(this->_AccelStepper.readEnc()) + ". deviation=" + String(this->_AccelStepper.computeDeviation()));  
                this->_AccelStepper.synchroniseMotorWithEncoder();    
                
                // if (this->step == 2) {
                //     float ep = this->revPulses / (2 * PI) * (-1.57) * this->moveDirection; 
                //     float sp = ep * this->encoder_motor_ratio;

                //     this->currentAngle = -1.57; // -90 degress
                //     this->targetAngle = -1.57;
                //     this->_AccelStepper.writeEnc(ep);
                //     this->_AccelStepper.setCurrentPosition(sp);
                // } else {
                    this->_AccelStepper.writeEnc(0);
                    this->_AccelStepper.setCurrentPosition(0);
                    this->currentAngle = 0;
                    this->targetAngle = 0;
                // }
           
                
                logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Encoder and current angle reseted. 0-Position");
                calibrationMode = 5;
            }
        }

        if (this->calibrationMode > 0 && this->calibrationMode < 4) {
            this->_AccelStepper.runSpeed();
        }
        if (this->calibrationMode == 4) {
            this->_AccelStepper.run();
        }
    }

    if (this->calibrationMode != 5) {
        return 0; // not calibrated
    }

    // v = s/t
    // this->elapsedTime += deltaT;

    // float deltaAngle = this->currentAngleVelocity * this->elapsedTime / 1000.0;

    // // logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") process.deltaAngle "+String(deltaAngle));
    //   // Serial.println(deltaAngle);

    // if (fabs(deltaAngle) > fabs(this->targetAngle - this->startAngle))
    // {
    //     this->currentAngle = this->targetAngle;

    //     // set start to target to not move again on elapsed time. overflow ~50 days?
    //     this->startAngle = this->targetAngle;
    // }
    // else
    // {
    //     if (this->targetAngle > this->startAngle)
    //     {
    //         this->currentAngle = this->startAngle + deltaAngle;
    //     }
    //     else
    //     {
    //         this->currentAngle = this->startAngle - deltaAngle;
    //     }
    // }
    
    int mul = 1;
    if (this->step == 2 || this->step == 4) {
        // Reversed angle
        mul = -1;
    }
    this-> currentAngle = 2 * PI / this->revPulses * this->_AccelStepper.readEnc() * mul;
    
    return this->move();
}

bool VarSpeedServo::atTargetAngle()
{
    bool atTargetAngle = fabs(this->currentAngle - this->targetAngle) < 0.01;
  
    lastCurrPosPrint++;
    if (atTargetAngle == 0 && lastCurrPosPrint % 500000 == 0) {
         logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") currAngle=" + String(this-> currentAngle * 1000) + ", taegetAngle="+String(this-> targetAngle * 1000));
         logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") atTargetAngle="+String(atTargetAngle)+". motor = "+String(this->_AccelStepper.currentPosition())+" encoder="
            + String(this->_AccelStepper.readEnc()) + ". deviation=" + String(this->_AccelStepper.computeDeviation()));
    }

    return atTargetAngle;
}

unsigned int VarSpeedServo::move()
{
    this->_AccelStepper.runSpeed();
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
