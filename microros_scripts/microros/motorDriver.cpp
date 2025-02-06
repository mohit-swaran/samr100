#include "motorDriver.h"

motor* motorInstance = nullptr;

motor::motor(uint8_t pwmLPin, uint8_t pwmRPin, uint8_t encAPin, uint8_t encBPin)
{
    this->_pwmLPin = pwmLPin;
    this->_pwmRPin = pwmRPin;
    this->_encAPin = encAPin;
    this->_encBPin = encBPin; 
    
    kp = 0.0;
    ki = 0.0;
    prevE = 0;
    eIntegral = 0; 
    prevT = 0;
    vel_filt_prev = 0;
    encoderUpdated = false;
    pos = 0;
}

void motor::init()
{
    pinMode(_pwmLPin, OUTPUT);
    pinMode(_pwmRPin, OUTPUT);
    pinMode(_encAPin, INPUT_PULLUP);
    pinMode(_encBPin, INPUT_PULLUP);

    // Set the global motorInstance to the current motor (this)
    motorInstance = this;

    // Attach interrupt for encoder A pin
    attachInterrupt(digitalPinToInterrupt(_encAPin), motor::encoderISR, RISING);  // Call static ISR method

    stop();
}

void motor::updatePI()
{
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;

    int posCurrent = pos;
    noInterrupts();
    int ticks = posCurrent - posPrev;
    posPrev = posCurrent;
    interrupts();

    float velocity = (float)ticks / deltaT;
    float velocity_RPM = (velocity / encoderTicksPerRevolution) * 60.0;

    vel_filt = 0.707 * vel_filt + 0.146 * velocity_RPM + 0.146 * vel_filt_prev;
    vel_filt_prev = velocity_RPM;

    error = setpoint - vel_filt;
    eIntegral += error * deltaT;
    eIntegral = constrain(eIntegral, -10.0, 10.0);

    u = kp * error + ki * eIntegral;
    pwmVal = constrain(abs(u), 0, 255);

    if (setpoint == 0)
        stop();
    else if (u > 0)
        forward(pwmVal);
    else
        reverse(pwmVal);
}

void motor::setSpeed(int16_t speed)
{
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

void motor::stop()
{
    // Stop the motor by setting PWM to 0
    analogWrite(_pwmLPin, 0);
    analogWrite(_pwmRPin, 0);
}

void motor::setPI(float kp, float ki)
{
    // Set PID values
    this->kp = kp;
    this->ki = ki;
}

void motor::setOscillatingSetpoint(int rpm) {
    long currT = micros();  // Get current time
    float oscillatingValue = rpm * (sin(currT / 1e6) > 0 ? 1 : -1);  // Oscillate between positive and negative RPM
    setSpeed(oscillatingValue);  // Call the setSpeed function with oscillating value
}
// Internal methods

void motor::encoderISR() {
    if (motorInstance != nullptr) {
        motorInstance->readVelocity(); // Call instance-specific method
        motorInstance->encoderUpdated = true; // Set the flag on the specific instance
    }
}

void motor::readVelocity() {
    // Read encoder B state to determine direction
    int b = digitalRead(_encBPin);
    int increment = (b > 0) ? 1 : -1;
    pos += increment;  // Update position based on direction
}

float motor::getVelocity(){
    return vel_filt;  // Return filtered velocity in RPM
}

int motor::getEncoderCounts(){
    return pos;
}

float motor::getControlValues(){
    return u;
}

float motor::getSetpoint(){
    return setpoint;
}

int motor::getVelocityFiltered(){
    return vel_filt;
}

float motor::getEintegral(){
    return eIntegral;
}


void motor::debug() {
    // Log general information for debugging
    Serial.println("----- Motor Debug Log -----");
    Serial.print(">Setpoint: ");
    Serial.println(setpoint);

    Serial.print(">Current Velocity (RPM): ");
    Serial.println(vel_filt);

    Serial.print(">Filtered Velocity (RPM): ");
    Serial.println(vel_filt);

    Serial.print(">Control Signal (u): ");
    Serial.println(u);

    Serial.print(">PWM Value: ");
    Serial.println(pwmVal);

    Serial.print("Encoder Position: ");
    Serial.println(pos);

    Serial.print(">Integral Term (eIntegral): ");
    Serial.println(eIntegral);

    // Event-based logging
    if (setpoint == 0 && pwmVal == 0) {
        Serial.println("Motor is stopped.");
    } else if (u > 0) {
        Serial.println("Motor is running forward.");
    } else if (u < 0) {
        Serial.println("Motor is running in reverse.");
    }

    // Warning for integral windup
    if (abs(eIntegral) > 1000) { // Arbitrary threshold; adjust as needed
        Serial.println("Warning: Integral windup detected!");
    }

    // Log boundary condition for PWM
    if (pwmVal == 255) {
        Serial.println("PWM value is at the maximum limit.");
    } else if (pwmVal == 0) {
        Serial.println("PWM value is at the minimum limit.");
    }

    Serial.println("----------------------------");
}
