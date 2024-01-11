#pragma once

/* FreeRTOS monitoring / introspection. */

void vTaskGetRunTimeStats(signed char *pcWriteBuffer);

/* Return number of tasks. */
unsigned int task_count();
