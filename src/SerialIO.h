
#ifndef SERIALIO_H
#define SERIALIO_H 1

#include "Arduino.h"
#include "CommunicationInterface.h"

#define READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE 30
#define BAUD_RATE 912600


class SerialIO : public CommunicationInterface {
public:

    SerialIO()  {
      Serial.begin(BAUD_RATE);
    }

    void process() {
        unsigned int bytesRead = 0;

        while (Serial.available() && bytesRead < READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE) {
            // todo check if is nullpointer?
            this->dataCallback(Serial.read());

            bytesRead++;
        }
    }

    void onData(void (*dataCallback)(char))
    {
        this->dataCallback = dataCallback;
    }

    // may want to implement an internal serial buffer and only send on process? May be RAM intesive
    void transmit(char message[], int length) {
        message[length] = 0;
        this->transmit(message);
    }

    void transmit(const char string[]) {
        Serial.print(string);
    }

    void transmit(char c) {

        Serial.print(c);
        if (c ==  '\r') {
          Serial.print('\n');
        }
    }

    void transmit(String string) {
        Serial.print(string);
    }

private:

    void (*dataCallback)(char);
};



static SerialIO IO;

#endif
