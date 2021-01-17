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
    unsigned int revSteps,
    unsigned int revPulses):
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
    this->revSteps = revSteps;
    this->revPulses = revPulses;
    this->encoder_motor_ratio = revSteps / revPulses;
    this->motor_encoder_ratio = revPulses / revSteps;

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

     //this->move(); // drive to 0 position according to min max angle
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
    this->targetEncPosition = this->revPulses / (2 * PI) * angleRad; 
    this->targetSteps = this->targetEncPosition * encoder_motor_ratio;

    logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") setTargetRadAngle "+ String(this->targetAngle) + ", setTargetEncPosition=" + String(this->targetEncPosition)+", setTargetMotorSteps=" + String(this->targetSteps));
    
    this->_AccelStepper.moveTo(this->targetSteps);
    this->_AccelStepper.setAcceleration(2000);
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
    this->_AccelStepper.move(-100000);
    this->_AccelStepper.setSpeed(-1500.0);
}

unsigned int VarSpeedServo::process(unsigned int deltaT)
{
    if (this->calibrationMode > 0 && this->calibrationMode < 5) {
        // Calibration mode

        // int hallSensorValue = map(analogRead(hs), 0, 1023, 0, 255);
        int hsv;
        // if (hallSensorValue > 128) {
        if (digitalRead(this->hs) == LOW) {
            hsv = 0;
        } else {
            hsv = 1;
        }

        if (this->calibrationMode == 1 && hsv == 1) {
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - HS");
            this->calibrationMode = 2;
            this->_AccelStepper.move(1000);
            this->_AccelStepper.setSpeed(1000);
        }
        if (this->calibrationMode == 3 && hsv == 1) {
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - HS. At home");
            this->calibrationMode = 4;
            this->_AccelStepper.stop();
            this->_AccelStepper.writeEnc(0);
            this->_AccelStepper.setCurrentPosition(0);
            this->_AccelStepper.synchroniseMotorWithEncoder();    
            this->currentAngle = 0;
            this->targetAngle = 0;
            this->_AccelStepper.move(this->revSteps / 2); 
            this->_AccelStepper.setSpeed(1500);
            logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Encoder and current angle reseted");
            this->calibrationMode = 4; 
        } 
        
        if (this->_AccelStepper.distanceToGo() == 0) {
            if (this->calibrationMode == 2) {
                logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Slowly return back to home");
                this->_AccelStepper.move(-1050);
                this->_AccelStepper.setSpeed(-300.0);
                this->calibrationMode = 3;
            }
            if (this->calibrationMode == 4) {
                this->_AccelStepper.stop();
                this->_AccelStepper.correctDeviation();
                
                this->_AccelStepper.writeEnc(0);
                this->_AccelStepper.setCurrentPosition(0);
                this->_AccelStepper.synchroniseMotorWithEncoder();    
                
                this->currentAngle = 0;
                this->targetAngle = 0;
                
                logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") - Encoder and current angle reseted. 0-Position");
                calibrationMode = 5;
            }
        }

        if (this->calibrationMode > 0 && this->calibrationMode < 5) {
            this->_AccelStepper.runSpeed();
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

    
    this-> currentAngle = 2 * PI / this->revPulses * this->_AccelStepper.readEnc();
    
    if (this->step != 1 && lastCurrPosPrint % 1000000 == 0) {
        logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") currAngle=" + String(this-> currentAngle));
    }
    lastCurrPosPrint++;
    
    return this->move();
}

bool VarSpeedServo::atTargetAngle()
{
    bool atTargetAngle = fabs(this->currentAngle - this->targetAngle) < 0.001;
    // atTargetAngle = this->_AccelStepper.distanceToGo() == 0;

    if (this->step == 0 && atTargetAngle) {
         this->_AccelStepper.correctDeviation();
         logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") atTargetAngle. motor = "+String(this->_AccelStepper.currentPosition())+" encoder="
            + String(this->_AccelStepper.readEnc()) + ". deviation=" + String(this->_AccelStepper.computeDeviation()));

    }

   // logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") atTargetAngle. diff = "+String(diff));
    // atTargetAngle = this->_AccelStepper.distanceToGo() == 0;
    
  //  if (this->step != 1 && lastCurrPosPrint % 1000000 == 0) {
  //       logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") atTargetAngle. motor = "+String(this->_AccelStepper.currentPosition())+" encoder="
  //           + String(this->_AccelStepper.readEnc()) + ". deviation=" + String(this->_AccelStepper.computeDeviation()));
  //  }

    //if (this->step != 1 && atTargetAngle == 1) {
     //    logger.info("XXX (" + String(this->step) + "/" + String(this->dir) +") atTargetAngle. motor = "+String(this->_AccelStepper.currentPosition())+" encoder="
      //       + String(this->_AccelStepper.readEnc()) + ". deviation=" + String(this->_AccelStepper.computeDeviation()));
         
    //}

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
