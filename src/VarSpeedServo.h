#ifndef MY_SERVO_H
#define MY_SERVO_H 1

#ifndef MOCK_VIRTUAL // used for setting methods to virtual in test environment
# define MOCK_VIRTUAL
#endif // ifndef MOCK_VIRTUAL

#include <Arduino.h>
#include <AccelStepperEncoder.h>
#include <Encoder.h>

#define DELTA_T 15 // go at 66hz to best match the servo freq

class VarSpeedServo {
public:

    VarSpeedServo(
            int          step,
            int          dir,
            int          hs,
            float        maxAngleVelocity,
            int          encA,
            int          encB,
            float        minRadAngle,
            float        maxRadAngle,
            AccelStepperEncoder & _AccelStepper,
            Encoder      & _Encoder,
            float        homeRadAngle,
            int          direction,
            unsigned int revSteps,
            unsigned int revPulses);

    MOCK_VIRTUAL int          getStep();
    MOCK_VIRTUAL int          getDir();
    MOCK_VIRTUAL int          getHS();

    MOCK_VIRTUAL void         setAngleLimits(float minRadAngle,
                                             float maxRadAngle);
    MOCK_VIRTUAL float        getMinRadAngle();
    MOCK_VIRTUAL float        getMaxRadAngle();
    MOCK_VIRTUAL float        getCurrentAngle();

    MOCK_VIRTUAL void         runCalibration();

    MOCK_VIRTUAL void         setTargetRadAngle(float angleRad);
    MOCK_VIRTUAL float        getTargetRadAngle();

    MOCK_VIRTUAL void         setCurrentAngleVelocity(float angleRadVelocity);
    MOCK_VIRTUAL float        getCurrentAngleVelocity();

    MOCK_VIRTUAL int          getEncA();
    MOCK_VIRTUAL int          getEncB();

    MOCK_VIRTUAL float        getMaxAngleVelocity();

    MOCK_VIRTUAL bool         getOutOfRange();

    MOCK_VIRTUAL bool         atTargetAngle();

    MOCK_VIRTUAL unsigned int process(unsigned int deltaT = DELTA_T);

    MOCK_VIRTUAL unsigned int getCurrentEncPos();

    MOCK_VIRTUAL float        getHomeRadAngle();

private:

    bool virtualServo = false;

    int step;
    int dir;
    int hs;

    float homeAngle;

    int encA;
    int encB;

    /*
    0 - Not calibrated
    1 - Calibration - Moving to home towards hall sensor
    2 - Calibration - Moving back  
    3 - Calibration - Moving to home slowly
    4 - Calibration - Moving to 0 position
    5 - Calibrated
    */
    int calibrationMode = 0; 

    int direction; 

    unsigned long elapsedTime = 0;

    float startAngle = 0;
    volatile float currentAngle;
    float targetAngle = 0;
    float targetEncPosition = 0;
    float targetSteps = 0;
    float revSteps;
    float revPulses;
    float encoder_motor_ratio;
    float motor_encoder_ratio;

    unsigned long lastUpdate;
    unsigned long lastCurrPosPrint = 0;

    float minRadAngle;
    float maxRadAngle;

    float currentAngleVelocity;
    float maxAngleVelocity;

    bool outOfRange = false;

    AccelStepperEncoder &_AccelStepper;
    Encoder &_Encoder;

    unsigned int move();

    static float map_float(float x,
                           float in_min,
                           float in_max,
                           float out_min,
                           float out_max);
};
#endif /* ifndef MY_SERVO_H */
