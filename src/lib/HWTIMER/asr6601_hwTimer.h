#pragma once

#include <stdio.h>
#include "targets.h"

#include <functional>

using callback_function_t = std::function<void(void)>;

#define TimerIntervalUSDefault 20000

class hwTimer
{
public:
    static volatile uint32_t HWtimerInterval;
    static volatile bool isTick;
    static volatile int32_t PhaseShift;
    static volatile int32_t FreqOffset;
    static volatile uint32_t PauseDuration;
    static bool running;
    static bool alreadyInit;

    static void init();
    static void stop();
    static void pause(uint32_t duration);
    static void resume();
    static void callback(void);
    static void updateInterval(uint32_t newTimerInterval);
    static void resetFreqOffset();
    static void incFreqOffset();
    static void decFreqOffset();
    static void phaseShift(int32_t newPhaseShift);

    static void inline nullCallback(void);
    static void (*callbackTick)();
    static void (*callbackTock)();
    static void attachInterrupt(callback_function_t callback); // Attach interrupt callback which will be called upon update event (timer rollover)
    static void detachInterrupt();  // remove interrupt callback which was attached to update event
};