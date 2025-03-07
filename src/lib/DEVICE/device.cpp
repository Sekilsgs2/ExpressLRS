#include "targets.h"
#include "common.h"
#include "logging.h"
#include "helpers.h"
#include "device.h"

///////////////////////////////////////
// Even though we aren't using anything this keeps the PIO dependency analyzer happy!

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868)  || defined(Regulatory_Domain_IN_866) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#if defined(PLATFORM_ASR6601)
#include "SX126xDriver.h"
#else
#include "SX127xDriver.h"
#endif
#endif

#if defined(Regulatory_Domain_ISM_2400)
#include "SX1280Driver.h"
#endif

///////////////////////////////////////

static device_affinity_t *uiDevices;
static uint8_t deviceCount;

static bool eventFired[2] = {false, false};
static connectionState_e lastConnectionState[2] = {disconnected, disconnected};

static unsigned long deviceTimeout[16] = {0};

#if defined(PLATFORM_ESP32)
static TaskHandle_t xDeviceTask = NULL;
static SemaphoreHandle_t taskSemaphore;
static SemaphoreHandle_t completeSemaphore;
static void deviceTask(void *pvArgs);
#define CURRENT_CORE  xPortGetCoreID()
#else
#define CURRENT_CORE -1
#endif

void devicesRegister(device_affinity_t *devices, uint8_t count)
{
    uiDevices = devices;
    deviceCount = count;

    #if defined(PLATFORM_ESP32)
        taskSemaphore = xSemaphoreCreateBinary();
        completeSemaphore = xSemaphoreCreateBinary();
        disableCore0WDT();
        xTaskCreatePinnedToCore(deviceTask, "deviceTask", 3000, NULL, 0, &xDeviceTask, 0);
    #endif
}

void devicesInit()
{
    int32_t core = CURRENT_CORE;

    for(size_t i=0 ; i<deviceCount ; i++) {
        if (uiDevices[i].core == core || core == -1) {
            if (uiDevices[i].device->initialize) {
                (uiDevices[i].device->initialize)();
            }
        }
    }
    #if defined(PLATFORM_ESP32)
    if (core == 1)
    {
        xSemaphoreGive(taskSemaphore);
        xSemaphoreTake(completeSemaphore, portMAX_DELAY);
    }
    #endif
}

void devicesStart()
{
    int32_t core = CURRENT_CORE;
    unsigned long now = millis();

    for(size_t i=0 ; i<deviceCount ; i++)
    {
        if (uiDevices[i].core == core || core == -1) {
            deviceTimeout[i] = 0xFFFFFFFF;
            if (uiDevices[i].device->start)
            {
                int delay = (uiDevices[i].device->start)();
                deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
            }
        }
    }
    #if defined(PLATFORM_ESP32)
    if (core == 1)
    {
        xSemaphoreGive(taskSemaphore);
        xSemaphoreTake(completeSemaphore, portMAX_DELAY);
    }
    #endif
}

void devicesStop()
{
    #if defined(PLATFORM_ESP32)
    vTaskDelete(xDeviceTask);
    #endif
}

void devicesTriggerEvent()
{
    eventFired[0] = true;
    eventFired[1] = true;
}

void devicesUpdate(unsigned long now)
{
    int32_t core = CURRENT_CORE;

    bool handleEvents = eventFired[core==-1?0:core];
    eventFired[core==-1?0:core] = false;

    for(size_t i=0 ; i<deviceCount ; i++)
    {
        if (uiDevices[i].core == core || core == -1) {
            if ((handleEvents || lastConnectionState[core==-1?0:core] != connectionState) && uiDevices[i].device->event)
            {
                int delay = (uiDevices[i].device->event)();
                if (delay != DURATION_IGNORE)
                {
                    deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
                }
            }
        }
    }
    lastConnectionState[core==-1?0:core] = connectionState;

    for(size_t i=0 ; i<deviceCount ; i++)
    {
        if (uiDevices[i].core == core || core == -1) {
            if (uiDevices[i].device->timeout && now >= deviceTimeout[i])
            {
                int delay = (uiDevices[i].device->timeout)();
                deviceTimeout[i] = delay == DURATION_NEVER ? 0xFFFFFFFF : now + delay;
            }
        }
    }
}

#if defined(PLATFORM_ESP32)
static void deviceTask(void *pvArgs)
{
    xSemaphoreTake(taskSemaphore, portMAX_DELAY);
    devicesInit();
    xSemaphoreGive(completeSemaphore);
    xSemaphoreTake(taskSemaphore, portMAX_DELAY);
    devicesStart();
    xSemaphoreGive(completeSemaphore);
    for (;;)
    {
        devicesUpdate(millis());
    }
}
#endif
