#ifndef ENCODER_H
#define ENCODER_H

#include <wiringPi.h>
#include <queue>
#include <utility>
#include <iostream>
#include <thread>
#include <atomic>

using IntPair = std::pair<int, int>;

class Encoder {
public:
    Encoder(int pinA, int pinB, std::queue<IntPair>& EncoderQueue, int& angle);
    void start(); // Start reading the encoder
    void stop();  // Stop reading the encoder
    
private:
    void readEncoder();
    int pinA, pinB;
    std::queue<IntPair>& EncoderQueue;
    int& angle;
    std::atomic<bool> running;
    std::thread encoderThread;
};

#endif
