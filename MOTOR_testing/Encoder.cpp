#include "Encoder.h"
#include <wiringPi.h>
#include <iostream>
#include <chrono>
#include <thread>

Encoder::Encoder(int pinA, int pinB, std::queue<IntPair>& EncoderQueue, int& angle) :
    pinA(pinA), pinB(pinB), EncoderQueue(EncoderQueue), angle(angle), running(false) {
    wiringPiSetupGpio(); // Setup GPIO using Broadcom numbering
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pullUpDnControl(pinA, PUD_UP);
    pullUpDnControl(pinB, PUD_UP);
}

void Encoder::start() {
    running = true;
    encoderThread = std::thread(&Encoder::readEncoder, this);
}

void Encoder::stop() {
    running = false;
    if (encoderThread.joinable()) {
        encoderThread.join();
    }
}

void Encoder::readEncoder() {
    int lastStateA = digitalRead(pinA);
    while (running) {
        int currentStateA = digitalRead(pinA);
        int currentStateB = digitalRead(pinB);
        if (currentStateA != lastStateA) {
            if (digitalRead(pinB) != currentStateA) {
                angle++;
            } else {
                angle--;
            }
            EncoderQueue.push({angle, currentStateB});
        }
        lastStateA = currentStateA;
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Debounce delay
    }
}
