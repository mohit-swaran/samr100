#ifndef MICRODRIVE_H
#define MICRODRIVE_H

#define DEBUG_ALL        0xFF  // Print everything
#define DEBUG_POSITION   0x01  // Print encoder position
#define DEBUG_TICKS      0x02  // Print encoder ticks
#define DEBUG_VELOCITY   0x04  // Print filtered velocity
#define DEBUG_SETPOINT   0x08  // Print velocity setpoint
#define DEBUG_CONTROL    0x10  // Print PI control value
#define DEBUG_PWM        0x20  // Print PWM output
#define DEBUG_ERROR      0x40  // Print error and integral error

#include <Arduino.h>

class motor {
public:
    motor(uint8_t pwmLPin, uint8_t pwmRPin, uint8_t encAPin, uint8_t encBPin);

    void init(int motorIndex);  // Modified to accept motor index
    void updatePI();
    void setSpeed(int16_t speed);
    void forward(int16_t pwmVal);
    void reverse(int16_t pwmVal);
    void stop();
    void setPI(float kp, float ki);
    int encoderTicksPerRevolution;
    float getVelocity();
    int getEncoderCounts();
    float getControlValues();
    float getSetpoint();
    int getVelocityFiltered();
    float getEintegral();

    void debug(uint8_t flags);

private:
    uint8_t _pwmLPin, _pwmRPin, _encAPin, _encBPin;
    
    volatile int pos;
    int posPrev;
    float vel_filt, vel_filt_prev;
    float kp, ki;
    float error, eIntegral;
    float u;
    int pwmVal;
    long prevT;
    int motorIndex;

    float setpoint;

    void readVelocity();

    // Separate static ISRs for each motor
    static void encoderISR1();
    static void encoderISR2();
    static void encoderISR3();
};

// Global array of motor instances
extern motor* motorInstances[3];

#endif