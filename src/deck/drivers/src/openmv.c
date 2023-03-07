#define DEBUG_MODULE "OpenMV"

#include "debug.h"
#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "deck.h"
#include "param.h"
#include "commander.h"
#include "uart1.h"
#include "motors.h"

#include <math.h>

//#define OMV_PIN DECK_GPIO_IO1
#define OMV_PIN_IN UART1_GPIO_RX_PIN
#define OMV_PIN_OUT UART1_GPIO_TX_PIN

static bool isInit = false;
static xTimerHandle timer;

/*static void setHoverSetpoint(setpoint_t* setpoint, float z)
{
    setpoint->mode.z = modeVelocity;
    setpoint->position.z = 0;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = 0;
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = 0;
    setpoint->velocity.y = 0;
    setpoint->velocity.z = z;
    setpoint->velocity_body = true;
}*/

// uint16_t freqLookup(uint16_t in)
// {

// }

void omvSetMotorSpeed(uint8_t z)
{
    //float new_val = 100.0f * ((float) z) / 255.0f;
    //DEBUG_PRINT("New Z vel: %f\n", (double) new_val);

    //setpoint_t setpoint;
    //memset(&setpoint, 0, sizeof(setpoint_t));
    //setHoverSetpoint(&setpoint, new_val);
    //commanderSetSetpoint(&setpoint, 3);

    uint16_t freq = (uint16_t) (440.0 * pow(2, ((double)z-1) / 12.0));
    if (z > 0) {
        DEBUG_PRINT("Freq: %d\n", (unsigned int) freq);
        motorsBeep(MOTOR_M1, true, freq, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
        vTaskDelay(M2T(EIGHTS));
        //motorsBeep(MOTOR_M2, true, freq, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / freq)/ 20);
        //motorsBeep(MOTOR_M3, true, freq, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / freq)/ 20);
        //motorsBeep(MOTOR_M4, true, freq, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / freq)/ 20);
    } else {
        //motorsBeep(MOTOR_M1, false, freq, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / freq)/ 20);
        //motorsBeep(MOTOR_M2, false, 0, 0);
        //motorsBeep(MOTOR_M3, false, 0, 0);
        //motorsBeep(MOTOR_M4, false, 0, 0);
    }
}

static void omvTimer(xTimerHandle timer)
{
    //int status = digitalRead(OMV_PIN);
    
    // get serial data from UART
    // 1. loop until start byte is found
    // 2. get all 64 bytes, including start byte
    uint8_t buf[8];
    buf[0] = 0;
    bool res;
    do {
        res = uart1GetDataWithTimeout(&buf[0], 100);
        if (!res) {
            DEBUG_PRINT("OMV: connection timed out (first byte)\n");
            return;
        }
    } while (buf[0] != 0x45);

    for (int i = 1; i < sizeof(buf); i++) {
        res = uart1GetDataWithTimeout(&buf[i], 100);
        if (!res) {
            DEBUG_PRINT("OMV: connection timed out\n");
            return;
        }
    }

    // parse message
    switch (buf[1]) {
    case 0: // pass
        break;
    case 1: // set motor speed
        DEBUG_PRINT("[%d, %d, %d]\n", buf[0], buf[1], buf[2]);
        omvSetMotorSpeed(buf[2]);
        break;
    default:
        break;
    }
    
    return;
}

static void omvInit()
{
    if (isInit) return;

    // initialize UART1 comms
    //pinMode(OMV_PIN, INPUT);
    uart1InitWithParity(115200, uart1ParityNone);
    if (!uart1Test()) {
        DEBUG_PRINT("Could not use UART1\n");
        return;
    }

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
    .usedPeriph = DECK_USING_UART1,
};

DECK_DRIVER(omvDriver);