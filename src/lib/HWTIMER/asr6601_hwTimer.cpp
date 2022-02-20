#include "asr6601_hwTimer.h"
#include "logging.h"

void inline hwTimer::nullCallback(void) {}

void (*hwTimer::callbackTick)() = &nullCallback;
void (*hwTimer::callbackTock)() = &nullCallback;

volatile uint32_t hwTimer::HWtimerInterval = TimerIntervalUSDefault;
volatile bool hwTimer::isTick = false;
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;
volatile uint32_t hwTimer::PauseDuration = 0;
uint32_t NextTimeout = 0;
bool hwTimer::running = false;
bool hwTimer::alreadyInit = false;

callback_function_t callbacks[4];
	
#define OneShot_Delay(cnt)                        {BSTIMER1->ARR = cnt;\
													BSTIMER1->CNT = 0; \
                                                   bstimer_cmd(BSTIMER1, 1);}	
	
void hwTimer::attachInterrupt(callback_function_t callback)
{
    callbacks[0] = callback;
}

/**
  * @brief  Dettach interrupt callback on Capture/Compare event
  * @param  channel: Arduino channel [1..4]
  * @retval None
  */
void hwTimer::detachInterrupt()
{
  callbacks[0] = NULL;
}

void hwTimer::init()
{
  bstimer_init_t bstimer_init_config;
    if (!alreadyInit)
    {
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_BSTIMER1,true);

  bstimer_init_config.prescaler = 47;
  bstimer_init_config.bstimer_mms = BSTIMER_MMS_ENABLE;
  bstimer_init_config.period = 0;
  bstimer_init_config.autoreload_preload = 0;
  bstimer_init(BSTIMER1, &bstimer_init_config);
  bstimer_config_overflow_update(BSTIMER1, 1);
  bstimer_generate_event(BSTIMER1, BSTIMER_EGR_UG, 1);
  bstimer_config_one_pulse(BSTIMER1, 1);
  bstimer_config_interrupt(BSTIMER1, 1);
  bstimer_cmd(BSTIMER1, 0);

  NVIC_EnableIRQ(BSTIMER1_IRQn);
  //NVIC_SetPriority(BSTIMER1_IRQn,254);
  //NVIC_SetPriority(BSTIMER1_IRQn,64);
  alreadyInit = true;
	}
}

void hwTimer::stop()
{
    if (running)
    {			bstimer_cmd(BSTIMER1, 0);
				NVIC_DisableIRQ(BSTIMER1_IRQn);
				BSTIMER1->CNT = 0;
				hwTimer::detachInterrupt();
        running = false;
    }
}

void ICACHE_RAM_ATTR hwTimer::resume()
{
    if (!running)
    {
        noInterrupts();
        hwTimer::attachInterrupt(hwTimer::callback);
        // The STM32 timer fires tock() ASAP after enabling, so mimic that behavior
        // tock() should always be the first event to maintain consistency
        isTick = false;
        // Fire the timer in 2us to get it started close to now
        //NextTimeout = (2);
		//		OneShot_Delay(NextTimeout);
		NVIC_EnableIRQ(BSTIMER1_IRQn);
        interrupts();
		//bstimer_cmd(BSTIMER1, 1);
        running = true;
		if (callbacks[0])
			callbacks[0]();
    }
}

void hwTimer::updateInterval(uint32_t newTimerInterval)
{
    // timer should not be running when updateInterval() is called
    hwTimer::HWtimerInterval = newTimerInterval;
}

void hwTimer::resetFreqOffset()
{
    FreqOffset = 0;
}

void hwTimer::incFreqOffset()
{
    FreqOffset++;
}

void hwTimer::decFreqOffset()
{
    FreqOffset--;
}

void hwTimer::phaseShift(int32_t newPhaseShift)
{
    int32_t minVal = -(hwTimer::HWtimerInterval >> 2);
    int32_t maxVal = (hwTimer::HWtimerInterval >> 2);
    hwTimer::PhaseShift = constrain(newPhaseShift, minVal, maxVal);
}

void ICACHE_RAM_ATTR hwTimer::callback()
{
	//INFOLN("T %d - %d\r\n", micros(), hwTimer::HWtimerInterval+FreqOffset);
	//INFOLN("tm %d int %d\r\n", micros(),);
    if (!running)
    {
        return;
    }
    NextTimeout = (hwTimer::HWtimerInterval >> 1) + (FreqOffset);
	//uint32_t tm = micros();
	//INFOLN("T %d - %d\r\n", tm, NextTimeout);
    if (hwTimer::isTick)
    {
				OneShot_Delay(NextTimeout);
        hwTimer::callbackTick();
    }
    else
    {
        NextTimeout += hwTimer::PhaseShift;
				OneShot_Delay(NextTimeout);
        hwTimer::PhaseShift = 0;
        hwTimer::callbackTock();
    }
    hwTimer::isTick = !hwTimer::isTick;
}

extern "C" {
void BSTIMER1_IRQHandler()
{
  if ( bstimer_get_status(BSTIMER1, BSTIMER_SR_UIF) )
  {
	if (callbacks[0])
		callbacks[0]();
  }
}
}
