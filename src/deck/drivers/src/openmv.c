#define DEBUG_MODULE "OpenMV"

#include "debug.h"
#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "deck.h"
#include "param.h"
#include "commander.h"

#define OMV_PIN DECK_GPIO_IO1

static bool isInit = false;
static xTimerHandle timer;

static void setHoverSetpoint(setpoint_t* setpoint)
{
    setpoint->mode.z = modeAbs;
    setpoint->position.z = 1;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = 0;
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = 0;
    setpoint->velocity.y = 0;
    setpoint->velocity_body = true;
}

static void omvTimer(xTimerHandle timer)
{
    int status = digitalRead(OMV_PIN);
    setpoint_t setpoint;
    if (status) {
        DEBUG_PRINT("Object detected?\n");
        setHoverSetpoint(&setpoint);
    } else {
        memset(&setpoint, 0, sizeof(setpoint_t));
    }
    commanderSetSetpoint(&setpoint, 3);
}

static void omvInit()
{
    if (isInit) return;

    pinMode(OMV_PIN, INPUT);
    timer = xTimerCreate("omvTimer", M2T(200), pdTRUE, NULL, omvTimer);
    xTimerStart(timer, 100);

    isInit = true;
    DEBUG_PRINT("OpenMV connected.\n");
}

static bool omvTest()
{
    DEBUG_PRINT("Mission accomplished!\n");
    return true;
}

static const DeckDriver omvDriver = {
    .vid = 0,
    .pid = 0,
    .name = "openmv",
    .init = omvInit,
    .test = omvTest,
    .usedGpio = DECK_USING_IO_1,
};

DECK_DRIVER(omvDriver);