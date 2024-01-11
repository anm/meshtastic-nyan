#include <cstdio>
#include <cstring>

#include "freertosinc.h"
#include "os_status.h"

unsigned int task_count() {
  return uxTaskGetNumberOfTasks();
}

/* Some functions here need a configuration option set in the build of the
   ESP-IDF, normally done in menu config. Unfortunatly when using Arduino,
   Arduino does this build with this option set, so would need to make a
   custom arduino build to use it.
 */

#ifdef USE_TRACE_FACILITY
/* Code from https://www.freertos.org/uxTaskGetSystemState.html */

/* This example demonstrates how a human readable table of run time stats
information is generated from raw data provided by uxTaskGetSystemState().
The human readable table is written to pcWriteBuffer.  (see the vTaskList()
API function which actually does just this). */
void vTaskGetRunTimeStats(char *pcWriteBuffer)
{
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  uint32_t ulTotalRunTime, ulStatsAsPercentage;

   /* Make sure the write buffer does not contain a string. */
   *pcWriteBuffer = 0x00;

   /* Take a snapshot of the number of tasks in case it changes while this
   function is executing. */
   uxArraySize = uxTaskGetNumberOfTasks();

   /* Allocate a TaskStatus_t structure for each task.  An array could be
   allocated statically at compile time. */
   pxTaskStatusArray = (TaskStatus_t *) pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL )
   {
      /* Generate raw status information about each task. */
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

      /* For percentage calculations. */
      ulTotalRunTime /= 100UL;

      /* Avoid divide by zero errors. */
      if( ulTotalRunTime > 0 )
      {
         /* For each populated position in the pxTaskStatusArray array,
         format the raw data as human readable ASCII data. */
         for( x = 0; x < uxArraySize; x++ )
         {
            /* What percentage of the total run time has the task used?
            This will always be rounded down to the nearest integer.
            ulTotalRunTimeDiv100 has already been divided by 100. */
            ulStatsAsPercentage =
                  pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

            if( ulStatsAsPercentage > 0UL )
            {
               sprintf( pcWriteBuffer, "%stt%lutt%lu%%rn",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter,
                                 ulStatsAsPercentage );
            }
            else
            {
               /* If the percentage is zero here then the task has
               consumed less than 1% of the total run time. */
               sprintf( pcWriteBuffer, "%stt%lutt<1%%rn",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter );
            }

            pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
         }
      }

      /* The array is no longer needed, free the memory it consumes. */
      vPortFree( pxTaskStatusArray );
   }
}
#endif
