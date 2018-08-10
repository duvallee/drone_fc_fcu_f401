/*
 * File: scheduler.c
 *
 * Written by duvallee in 2018
 *
*/
#include "main.h"

// ---------------------------------------------------------------------------
#define MAX_SCHEDULER_COUNT                              1                          // max 8
#define SCHEDULER_FLAG_SIZE                              32                         // does not modified

#define NO_TIMER                                         0
#define NO_EVENT                                         0

#define RESET_EVENT_FLAG                                 1
#define SET_EVENT_FLAG                                   1

#define UNLOCK                                           0
#define LOCK                                             1
// ---------------------------------------------------------------------------
typedef struct SOFT_TIMER_STRUCT
{
   uint32_t cycle_ms;                                    // 
   uint32_t elapse_ms;
   uint8_t expire_count;
   volatile uint8_t lock;

   TIMER_FN fn;
} SOFT_TIMER;

typedef struct WAIT_EVENT_STRUCT
{
   uint8_t event;

   volatile uint8_t event_flag;
   volatile uint8_t duplicate_event_flag;
   volatile uint8_t lock;

   uint8_t status;
   WAIT_EVENT_FN fn;

   uint16_t wait_ms;
   uint32_t elapse_ms;
} WAIT_EVENT;

typedef struct SCHEDULER_STRUCT
{
   uint32_t sheduler_time_ms;

   uint32_t soft_timer_flag[MAX_SCHEDULER_COUNT];
   uint32_t event_flag[MAX_SCHEDULER_COUNT];

   SOFT_TIMER soft_timer[MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE];
   WAIT_EVENT event[MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE];
} SCHEDULER;

static SCHEDULER g_scheduler;

// ***************************************************************************
// Fuction      : HAL_SYSTICK_Callback()
// Description  : 
// 
//
// ***************************************************************************
void HAL_SYSTICK_Callback()
{
   uint32_t index                                        = 0;
   uint32_t bits                                         = 0;
   unsigned char i;
   g_scheduler.sheduler_time_ms++;

   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      if (g_scheduler.soft_timer[i].cycle_ms != 0)
      {
         if (g_scheduler.soft_timer[i].cycle_ms <= g_scheduler.soft_timer[i].elapse_ms)
         {
             g_scheduler.soft_timer[i].elapse_ms         = 1;
            index                                        = i / SCHEDULER_FLAG_SIZE;
            bits                                         = i % SCHEDULER_FLAG_SIZE;
            g_scheduler.soft_timer_flag[index]           |= (0x1 << bits);
         }
         else
         {
             g_scheduler.soft_timer[i].elapse_ms++;
         }
      }
      if (g_scheduler.event[i].event != NO_EVENT)
      {
         if (g_scheduler.event[i].event_flag == SET_EVENT_FLAG)
         {
            if (g_scheduler.event[i].wait_ms != 0)
            {
               if (g_scheduler.event[i].wait_ms <= g_scheduler.event[i].elapse_ms)
               {
                  g_scheduler.event[i].elapse_ms         = 1;
                  index                                  = i / SCHEDULER_FLAG_SIZE;
                  bits                                   = i % SCHEDULER_FLAG_SIZE;
                  g_scheduler.event_flag[index]          |= (0x1 << bits);
               }
            }
         }
       }
   }

}

// ***************************************************************************
// Fuction      : scheduler_run()
// Description  : 
// 
//
// ***************************************************************************
void scheduler_run()
{
   uint32_t index                                        = 0;
   uint32_t bits                                         = 0;
   unsigned char i;
   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      index                                              = i / SCHEDULER_FLAG_SIZE;
      bits                                               = i % SCHEDULER_FLAG_SIZE;
      if ((g_scheduler.event_flag[index] & (0x1 << bits)) != 0)
      {
         // call
         g_scheduler.event_flag[index]                   &= ~(0x1 << bits);
      }
      if ((g_scheduler.soft_timer_flag[index] & (0x1 << bits)) != 0)
      {
         // call
         g_scheduler.soft_timer_flag[index]              &= ~(0x1 << bits);
      }
   }
}




// ***************************************************************************
// Fuction      : scheduler_init()
// Description  : 
// 
//
// ***************************************************************************
void scheduler_init()
{
   memset(&g_scheduler, 0, sizeof(g_scheduler));
}







