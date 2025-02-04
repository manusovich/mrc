// AccelStepperEncoder.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepperEncoder.cpp,v 1.17 2013/08/02 01:53:21 mikem Exp mikem $

#include "AccelStepperEncoder.h"

//#define DEBUG

#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
    int i;

    for (i = 0; i < l; i++)
    {
	Serial.print(p[i], HEX);
	Serial.print("");
    }
    Serial.println("");
}
#endif

void AccelStepperEncoder::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
		_targetPos = absolute;
		_encoderTargetPos = encoderPositionForMotor(_targetPos);
		computeNewSpeed();
		// compute new n?
    }
}

float AccelStepperEncoder::encoderPositionForMotor(long motorPos) 
{
	return (float)motorPos / _motorToEncoderRatio;
}

void AccelStepperEncoder::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepperEncoder::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
		return false;

    unsigned long time = micros();
    // Gymnastics to detect wrapping of either the nextStepTime and/or the current time
    unsigned long nextStepTime = _lastStepTime + _stepInterval;
    if (   ((nextStepTime >= _lastStepTime) && ((time >= nextStepTime) || (time < _lastStepTime)))
	|| ((nextStepTime < _lastStepTime) && ((time >= nextStepTime) && (time < _lastStepTime))))
    {
		if (_direction == DIRECTION_CW)
		{
			// Clockwise
			_currentPos += 1;
		}
		else
		{
			// Anticlockwise  
			_currentPos -= 1;
		}
		step(_currentPos);

		_lastStepTime = time;
		return true;
    }
    else
    {
		return false;
    }
}

long AccelStepperEncoder::distanceToGo()
{
    return _targetPos - _currentPos;
}

long AccelStepperEncoder::targetPosition()
{
    return _targetPos;
}

long AccelStepperEncoder::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepperEncoder::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
}

void AccelStepperEncoder::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from c urent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
	_stepInterval = 0;
	_speed = 0.0;
	_n = 0;
	return;
    }

    if (distanceTo > 0)
    {
	// We are anticlockwise from the target
	// Need to go clockwise from here, maybe decelerate now
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
		_n = -_n; // Start accceleration
	}
    }
    else if (distanceTo < 0)
    {
	// We are clockwise from the target
	// Need to go anticlockwise from here, maybe decelerate
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
		_n = -_n; // Start accceleration
	}
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
	// First step from stopped
	_cn = _c0;
	_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
	// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
	_cn = max(_cn, _cmin); 
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
	_speed = -_speed;

#ifdef DEBUG_ACCEL
    Serial.print("_speed: ");
    Serial.println(_speed);
    Serial.print("_acceleration: ");
    Serial.println(_acceleration);
    Serial.print("_cn: ");
    Serial.println(_cn);
    Serial.print("_c0: ");
    Serial.println(_c0);
    Serial.print("_n: ");
    Serial.println(_n);
    Serial.print("_stepInterval: ");
    Serial.println(_stepInterval);
    Serial.print("distanceTo: ");
    Serial.println(distanceTo);
    Serial.print("stepsToStop: ");
    Serial.println(stepsToStop);
    Serial.print("_currentPos: ");
    Serial.println(_currentPos);
	Serial.print("readEnc(): ");
	Serial.println(readEnc());
	
    Serial.println("-----");
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
boolean AccelStepperEncoder::run()
{
    if (runSpeed()) 
	{
		computeNewSpeed();
	}
    return _speed != 0.0 || distanceToGo() != 0;
}

// Look at motor position vs actual position, and correct motor position if necessary.
float AccelStepperEncoder::computeDeviation()
{
	float expectedEncPos = encoderPositionForMotor(_currentPos);
	float actualEncPos = readEnc();
	float deviation = actualEncPos - expectedEncPos;
	
	if (deviation > maxDeviation) maxDeviation = deviation;
	else if (deviation < minDeviation) minDeviation = deviation;

#ifdef DEBUG_DEV
    Serial.print("expectedEncPos: ");
    Serial.println(expectedEncPos);
    Serial.print("actualEncPos: ");
    Serial.println(actualEncPos);
    Serial.print("deviation: ");
    Serial.print(deviation);
    Serial.print(" (min: ");
    Serial.print(minDeviation);
    Serial.print(", max: ");
    Serial.print(maxDeviation);
	Serial.println(")");
    Serial.println("~~~~~");
#endif
	return deviation;
}

float AccelStepperEncoder::correctDeviation()
{
	float deviation = computeDeviation();
	if (abs(deviation) > acceptableDeviation)
	{
    Serial.print("CORRECTING DEVIATION! deviation" + String(deviation));

#ifdef DEBUG
    Serial.print("CORRECTING DEVIATION!");
    Serial.println(deviation);
#endif	
		synchroniseMotorWithEncoder();
	}
	else 
	{
#ifdef DEBUG
    //Serial.print("Deviation is not enough to fix: ");
    //Serial.println(deviation);
#endif	
	}
	return deviation;
}

/*
Resets the motor position to reflect the actual position (as got via encoder)
*/
void AccelStepperEncoder::synchroniseMotorWithEncoder() 
{
#ifdef DEBUG
    Serial.print("CurrentPos was: ");
    Serial.print(_currentPos);
#endif	
	_currentPos = readEnc() * _motorToEncoderRatio;
#ifdef DEBUG
	Serial.print(", is now set to: ");
	Serial.println(_currentPos);
#endif	
	_targetPos = _currentPos;

	//setSpeed(0);
}

AccelStepperEncoder::AccelStepperEncoder(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, bool enable)
{
    _interface = interface;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = pin1;
    _pin[1] = pin2;
    _pin[2] = pin3;
    _pin[3] = pin4;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    int i;
    for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;
    if (enable)
	enableOutputs();
}

AccelStepperEncoder::AccelStepperEncoder(void (*forward)(), void (*backward)())
{
    _interface = 0;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = 0;
    _pin[1] = 0;
    _pin[2] = 0;
    _pin[3] = 0;
    _forward = forward;
    _backward = backward;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    int i;
    for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;
}

void AccelStepperEncoder::setMaxSpeed(float speed)
{
    if (_maxSpeed != speed)
    {
	_maxSpeed = speed;
	_cmin = 1000000.0 / speed;
	// Recompute _n from current speed and adjust speed if accelerating or cruising
	if (_n > 0)
	{
	    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
	    computeNewSpeed();
	}
    }
}

void AccelStepperEncoder::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
	return;
    if (_acceleration != acceleration)
    {
	// Recompute _n per Equation 17
	_n = _n * (_acceleration / acceleration);
	// New c0 per Equation 7
	_c0 = sqrt(2.0 / acceleration) * 1000000.0;
	_acceleration = acceleration;
	computeNewSpeed();
    }
}

void AccelStepperEncoder::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0)
	_stepInterval = 0;
    else
    {
	_stepInterval = fabs(1000000.0 / speed);
	_direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed = speed;
}

float AccelStepperEncoder::speed()
{
    return _speed;
}

// Subclasses can override
void AccelStepperEncoder::step(long step)
{
    switch (_interface)
    {
        case FUNCTION:
            step0(step);
            break;

	case DRIVER:
	    step1(step);
	    break;
    
	case FULL2WIRE:
	    step2(step);
	    break;
    
	case FULL3WIRE:
	    step3(step);
	    break;  

	case FULL4WIRE:
	    step4(step);
	    break;  

	case HALF3WIRE:
	    step6(step);
	    break;  
		
	case HALF4WIRE:
	    step8(step);
	    break;  
    }
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepperEncoder::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
	numpins = 4;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

// 0 pin step function (ie for functional usage)
void AccelStepperEncoder::step0(long step)
{
  if (_speed > 0)
    _forward();
  else
    _backward();
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepperEncoder::step1(long step)
{
    // _pin[0] is step, _pin[1] is direction
    setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
    setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time 
    // Delay the minimum allowed pulse width
    delayMicroseconds(_minPulseWidth);
    setOutputPins(_direction ? 0b10 : 0b00); // step LOW

}


// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepperEncoder::step2(long step)
{
    switch (step & 0x3)
    {
	case 0: /* 01 */
	    setOutputPins(0b10);
	    break;

	case 1: /* 11 */
	    setOutputPins(0b11);
	    break;

	case 2: /* 10 */
	    setOutputPins(0b01);
	    break;

	case 3: /* 00 */
	    setOutputPins(0b00);
	    break;
    }
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepperEncoder::step3(long step)
{
    switch (step % 3)
    {
	case 0:    // 100
	    setOutputPins(0b100);
	    break;

	case 1:    // 001
	    setOutputPins(0b001);
	    break;

	case 2:    //010
	    setOutputPins(0b010);
	    break;
	    
    }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepperEncoder::step4(long step)
{
    switch (step & 0x3)
    {
	case 0:    // 1010
	    setOutputPins(0b0101);
	    break;

	case 1:    // 0110
	    setOutputPins(0b0110);
	    break;

	case 2:    //0101
	    setOutputPins(0b1010);
	    break;

	case 3:    //1001
	    setOutputPins(0b1001);
	    break;
    }
}

// 3 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepperEncoder::step6(long step)
{
    switch (step % 6)
    {
	case 0:    // 100
	    setOutputPins(0b100);
            break;
	    
        case 1:    // 101
	    setOutputPins(0b101);
            break;
	    
	case 2:    // 001
	    setOutputPins(0b001);
            break;
	    
        case 3:    // 011
	    setOutputPins(0b011);
            break;
	    
	case 4:    // 010
	    setOutputPins(0b010);
            break;
	    
	case 5:    // 011
	    setOutputPins(0b110);
            break;
	    
    }
}

// 4 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepperEncoder::step8(long step)
{
    switch (step & 0x7)
    {
	case 0:    // 1000
	    setOutputPins(0b0001);
            break;
	    
        case 1:    // 1010
	    setOutputPins(0b0101);
            break;
	    
	case 2:    // 0010
	    setOutputPins(0b0100);
            break;
	    
        case 3:    // 0110
	    setOutputPins(0b0110);
            break;
	    
	case 4:    // 0100
	    setOutputPins(0b0010);
            break;
	    
        case 5:    //0101
	    setOutputPins(0b1010);
            break;
	    
	case 6:    // 0001
	    setOutputPins(0b1000);
            break;
	    
        case 7:    //1001
	    setOutputPins(0b1001);
            break;
    }
}
    
// Prevents power consumption on the outputs
void    AccelStepperEncoder::disableOutputs()
{   
    if (! _interface) return;

    setOutputPins(0); // Handles inversion automatically
    if (_enablePin != 0xff)
        digitalWrite(_enablePin, LOW ^ _enableInverted);
}

void    AccelStepperEncoder::enableOutputs()
{
    if (! _interface) 
	return;

    pinMode(_pin[0], OUTPUT);
    pinMode(_pin[1], OUTPUT);
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
    {
        pinMode(_pin[2], OUTPUT);
        pinMode(_pin[3], OUTPUT);
    }

    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void AccelStepperEncoder::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void AccelStepperEncoder::setEnablePin(uint8_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void AccelStepperEncoder::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void AccelStepperEncoder::setPinsInverted(bool pin1Invert, bool pin2Invert, bool pin3Invert, bool pin4Invert, bool enableInvert)
{    
    _pinInverted[0] = pin1Invert;
    _pinInverted[1] = pin2Invert;
    _pinInverted[2] = pin3Invert;
    _pinInverted[3] = pin4Invert;
    _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void AccelStepperEncoder::runToPosition()
{
    while (run())
	;
}

boolean AccelStepperEncoder::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
	return false;
    if (_targetPos >_currentPos)
	_direction = DIRECTION_CW;
    else
	_direction = DIRECTION_CCW;
    return runSpeed();
}

// Blocks until the new target position is reached
void AccelStepperEncoder::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void AccelStepperEncoder::stop()
{
    if (_speed != 0.0)
    {    
	long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
	if (_speed > 0)
	    move(stepsToStop);
	else
	    move(-stepsToStop);
    }
}

void AccelStepperEncoder::addEncoder(Encoder *enc, float ratio)
{
	_motorToEncoderRatio = ratio;
	_enc = enc;
	writeEnc(0);
}
Encoder* AccelStepperEncoder::getEncoder()
{
	return _enc;
}
long AccelStepperEncoder::readEnc()
{
	return _enc->read();
}
void AccelStepperEncoder::writeEnc(long value)
{
	_enc->write(value);
}