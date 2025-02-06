#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

class motor{
public:
    
    motor(uint8_t pwmLPin,uint8_t pwmRPin,uint8_t encAPin, uint8_t encBPin);
    
    void init();

    void setSpeed(int16_t speed);

    void updatePI();

    void stop();

    void setPI(float kp ,float ki);

    void forward(int16_t pwmVal);

    void reverse(int16_t pwmVal);
    
    float getVelocity();
    
    int getEncoderCounts();
    
    float getControlValues();

    float getSetpoint();

    float getEintegral();

    static void encoderISR();

    int encoderTicksPerRevolution ;
    
    int getVelocityFiltered();
    
    void setOscillatingSetpoint(int rpm);

    void debug();

private:
    uint8_t _pwmLPin; 
    uint8_t _pwmRPin;
    uint8_t _encAPin;
    uint8_t _encBPin;
    int16_t pwmVal;

    // PID constants
    float kp;
    float ki;
    float u ;

    // PID variables
    float prevE;
    float error;
    float eIntegral;
    float setpoint;

    // Encoder variables
    volatile int pos;
    volatile int posPrev;
    volatile float velocity;
    volatile float velocity_con;
    volatile int _encoderTicks;
    volatile bool encoderUpdated ;
    int velocity_RPM;
    long prevT;
    float vel_filt;
    float vel_filt_prev;
    
    // Motor control method
    void readVelocity();

    
};

extern motor* motorInstance;


#endif