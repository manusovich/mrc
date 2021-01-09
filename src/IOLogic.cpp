#include "IOLogic.h"

#include "fastRW.h"
#include "Logger.h"


namespace {
Logger logger("IOLogic");
}


void IOLogic::addCondition(unsigned int pin, unsigned int state) {
    logger.info("setting pin as input. logical " + String(pin) + " physical " + String(pinMap[pin]));
    pin = pinMap[pin];

    if (state == IOLogic::IO_HIGH) {
        pinMode(pin, INPUT_PULLDOWN);
    } else {
        pinMode(pin, INPUT_PULLUP);
    }

    this->conditionBuffer[this->conditionBufferLength][0] = pin;
    this->conditionBuffer[this->conditionBufferLength][1] = state;
    this->conditionBufferLength++;
}

void IOLogic::setOutput(unsigned int pin, unsigned int state) {
    pin = pinMap[pin];
    pinAsOutput(pin);

    logger.info("setting pin ");
    logger.info(pin);
    logger.info("State");
    logger.info(state);

    switch (state) {
    case IOLogic::IO_HIGH:

        digitalHigh(pin);
        break;

    case IOLogic::IO_LOW:
        digitalLow(pin);
        break;

    default:
        logger.warning("expected state [0,1], given: " + String(state));
    }
}

bool IOLogic::isDone()                                           {
    if (this->conditionBufferLength == 0) {
        return true;
    } else {
        for (unsigned char i = 0; i < this->conditionBufferLength; i++) {
            unsigned int pin   = this->conditionBuffer[i][0];
            unsigned int state = this->conditionBuffer[i][1];

            /*
                  IO.transmit("waitung for pin - state ");
                  IO.transmit(pin);
                  IO.transmit(" - ");
                  logger.info(state);
                  delay(2000);
             */
            switch (state) {
            case IOLogic::IO_HIGH:

                if (isLow(pin)) {
                    return false;
                }
                break;

            case IOLogic::IO_LOW:

                if (isHigh(pin)) {
                    return false;
                }
                break;

            default:
                // logger.warning("expected state [0,1], given: " + String(state));
                break;
            }
        }

        // all done, reset the buffer
        this->conditionBufferLength = 0;
        return true;
    }
}

unsigned int IOLogic::getTargetState(unsigned int pin) {
    pin = pinMap[pin];

    for (size_t i = 0; i < this->conditionBufferLength; i++) {
        if (this->conditionBuffer[i][0] == pin) {
            return this->conditionBuffer[i][1];
        }
    }
    return 2;
}
