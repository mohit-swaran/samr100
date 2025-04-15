#include "microDrive.h"

// Define an array to hold motor instances
motor* motorInstances[3] = {nullptr, nullptr, nullptr};

// Constructor
motor::motor(uint8_t pwmLPin, uint8_t pwmRPin, uint8_t encAPin, uint8_t encBPin)
{
    this->_pwmLPin = pwmLPin;
    this->_pwmRPin = pwmRPin;
    this->_encAPin = encAPin;
    this->_encBPin = encBPin; 
    
    kp = 0.0;
    ki = 0.0;
    prevT = 0;
    vel_filt_prev = 0;
    pos = 0;
    posPrev = 0;
    error = 0;
    eIntegral = 0;
}

// Initialize motor and attach individual ISRs
void motor::init(int motorIndex)
{
    pinMode(_pwmLPin, OUTPUT);
    pinMode(_pwmRPin, OUTPUT);
    pinMode(_encAPin, INPUT_PULLUP);
    pinMode(_encBPin, INPUT_PULLUP);

    if (motorIndex >= 0 && motorIndex < 3) {
        motorInstances[motorIndex] = this;
    }

    // Attach separate ISRs for each motor
    switch (motorIndex) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(_encAPin), encoderISR1, RISING);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(_encAPin), encoderISR2, RISING);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(_encAPin), encoderISR3, RISING);
            break;
    }

    stop();
}

// Separate encoder ISRs for each motor
void motor::encoderISR1() { if (motorInstances[0]) motorInstances[0]->readVelocity(); }
void motor::encoderISR2() { if (motorInstances[1]) motorInstances[1]->readVelocity(); }
void motor::encoderISR3() { if (motorInstances[2]) motorInstances[2]->readVelocity(); }

void motor::readVelocity()
{
    int b = digitalRead(_encBPin);
    int increment = (b > 0) ? 1 : -1;
    
    noInterrupts();
    pos += increment;  // Still track short-term ticks
    interrupts();
}

void motor::updatePI()
{
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;

    noInterrupts();
    int ticks = pos;  // Get short-term ticks
    pos = 0;  // Reset for next cycle to prevent overflow
    interrupts();

    if (ticks == 0) {
        Serial.println("[ERROR] No encoder ticks! Motor might be stalled or ISR not working. Check the Wiring");
    }

    if (deltaT <= 0) {
        Serial.println("[ERROR] deltaT is zero or negative!");
        return;
    }

    float velocity = (float)ticks / deltaT;
    float velocity_RPM = (velocity / encoderTicksPerRevolution) * 60.0;

    if (isnan(velocity_RPM) || isinf(velocity_RPM)) {
        Serial.println("[ERROR] Velocity is NaN or INF! Ensure encoderTicksPerRevolution is Given");
        return;
    }

    vel_filt = 0.707 * vel_filt + 0.146 * velocity_RPM + 0.146 * vel_filt_prev;
    vel_filt_prev = velocity_RPM;

    error = setpoint - vel_filt;
    eIntegral += error * deltaT;
    eIntegral = constrain(eIntegral, -15.0, 15.0);

    u = kp * error + ki * eIntegral;
    pwmVal = constrain(abs(u), 0, 255);

    if (setpoint == 0) {
        stop();
    } else if (u > 0) {
        forward(pwmVal);
    } else {
        reverse(pwmVal);
    }
}



void motor::setSpeed(int16_t speed) {
    setpoint = speed;
}

void motor::forward(int16_t pwmVal){
    analogWrite(_pwmLPin, pwmVal);
    analogWrite(_pwmRPin, 0);
}

void motor::reverse(int16_t pwmVal){
    analogWrite(_pwmLPin, 0);
    analogWrite(_pwmRPin, pwmVal);
}

void motor::stop() {
    analogWrite(_pwmLPin, 0);
    analogWrite(_pwmRPin, 0);
}

void motor::setPI(float kp, float ki) {
    this->kp = kp;
    this->ki = ki;
}

float motor::getVelocity() {
    return vel_filt;
}

int motor::getEncoderCounts() {
    return pos;
}

float motor::getControlValues() {
    return u;
}

float motor::getSetpoint() {
    return setpoint;
}

int motor::getVelocityFiltered() {
    return vel_filt;
}

float motor::getEintegral() {
    return eIntegral;
}

void motor::debug(uint8_t flags = DEBUG_ALL) {
    Serial.println("[DEBUG] Motor State:");

    if (flags & DEBUG_POSITION) {
        Serial.print("  Encoder Position: "); Serial.println(pos);
    }
    if (flags & DEBUG_TICKS) {
        Serial.print("  Encoder Ticks: "); Serial.println(pos - posPrev);
    }
    if (flags & DEBUG_VELOCITY) {
        Serial.print("  Velocity (Filtered): "); Serial.print(vel_filt); Serial.println(" RPM");
    }
    if (flags & DEBUG_SETPOINT) {
        Serial.print("  Setpoint: "); Serial.println(setpoint);
    }
    if (flags & DEBUG_CONTROL) {
        Serial.print("  PI Control (u): "); Serial.println(u);
    }
    if (flags & DEBUG_PWM) {
        Serial.print("  PWM Output: "); Serial.println(pwmVal);
    }
    if (flags & DEBUG_ERROR) {
        Serial.print("  Error: "); Serial.println(error);
        Serial.print("  Integral Error: "); Serial.println(eIntegral);
    }

}