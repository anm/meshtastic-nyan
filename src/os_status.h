#pragma once

/* FreeRTOS monitoring / introspection. */

// This seems to have been added to the Arduino build of FreeRTOS so don't
// need to add it anymore.
//void vTaskGetRunTimeStats(signed char *pcWriteBuffer);

/* Return number of tasks. */
unsigned int task_count();
