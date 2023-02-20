#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

// queue for input
static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 1, sizeof(int));

// task call stack
static void openmvTask(void*);
STATIC_MEM_TASK_ALLOC(openmvTask, OPENMV_TASK_STACKSIZE);

static bool isInit = false;
static bool isListening = false;

void openmvTaskInit()
{
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);
    STATIC_MEM_TASK_CREATE(openmvTask, openmvTask, OPENMV_TASK_NAME, NULL, OPENMV_TASK_PRI);
    pinMode(DECK_GPIO_RX1, INPUT);
    isInit = true;
}

bool openmvTaskTest()
{
    return isInit;
}

static void openmvTask(void* params)
{
    DEBUG_PRINT("OpenMV task loop is running!");
    while (true) {
        int input;
        if (pdTrue == xQueueReceive(inputQueue, &input, portMAX_DELAY)) {
            // 0 : disable control
            // 1 : enable control
            // 2 : get status from OpenMV
            switch (input) {
            case 0:
                isListening = false;
                break;
            case 1:
                isListening = true;
                break;
            case 2:
                if (isListening) {
                    digitalRead()
                }
                break;
            default:
                break;
            }
        }
    }
}