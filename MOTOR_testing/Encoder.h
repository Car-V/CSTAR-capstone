#ifndef ENCODER_H
#define ENCODER_H

#include <pigpio.h>
#include <queue>
#include <utility>
#include <iostream>
#include <thread>
#include <atomic>

using IntPair = std::pair<int, int>;

class Encoder {
public:
    Encoder(int pinA, int pinB, std::queue<IntPair>& EncoderQueue, int& angle);
    void start();
    void stop();
    
private:
    static void encoderCallback(int gpio, int level, uint32_t tick, void* userdata);
    int pinA, pinB;
    std::queue<IntPair>& EncoderQueue;
    int& angle;
    std::atomic<bool> running;
};

#endif
