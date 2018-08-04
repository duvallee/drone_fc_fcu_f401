/*
 * File: scheduler.h
 *
 * Written by duvallee in 2018
 *
*/

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__
#ifdef __cplusplus
extern "C" {
#endif

typedef void (*TIMER_FN)(uint32_t system_ms);
typedef void (*WAIT_EVENT_FN)(uint8_t status);


void scheduler_init();
void scheduler_run();


#ifdef __cplusplus
}
#endif
#endif


