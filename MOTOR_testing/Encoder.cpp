// Purpose: Handles quadrature encoder signals, commonly used for measuring rotational position and speed.
// Functionality: The code initializes the GPIO pins for the encoder, sets up interrupt callbacks for 
// signal changes, and processes the encoder signals to update the angle and state of the encoder.

#include "Encoder.h"
#include <pigpio.h>
#include <iostream>

Encoder::Encoder(int pinA, int pinB, std::queue<IntPair>& EncoderQueue, int& angle) :
    pinA(pinA), pinB(pinB), EncoderQueue(EncoderQueue), angle(angle), running(false) {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
    }
    gpioSetMode(pinA, PI_INPUT);
    gpioSetMode(pinB, PI_INPUT);
    gpioSetPullUpDown(pinA, PI_PUD_UP);
    gpioSetPullUpDown(pinB, PI_PUD_UP);
}

void Encoder::start() {
    running = true;
    gpioSetAlertFuncEx(pinA, encoderCallback, this);
}

void Encoder::stop() {
    running = false;
    gpioSetAlertFuncEx(pinA, nullptr, nullptr);
    gpioTerminate();
}

void Encoder::encoderCallback(int gpio, int level, uint32_t tick, void* userdata) {
    Encoder* enc = static_cast<Encoder*>(userdata);
    int stateB = gpioRead(enc->pinB);
    if (level == 1) {
        if (stateB) {
            enc->angle++;
        } else {
            enc->angle--;
        }
        enc->EncoderQueue.push({enc->angle, stateB});
    }
}
