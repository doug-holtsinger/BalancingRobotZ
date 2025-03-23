
#ifndef __THREAD_H__
#define __THREAD_H__

/* normal running priority for a thread */
#define NORMAL_THREAD_PRIORITY 4

/* Initial thread priorities, 
  changed to normal thread priorities after initial execution 
  of the thread */
#define IMU_THREAD_PRIORITY (NORMAL_THREAD_PRIORITY-2)
#define MOTOR_DRIVER_THREAD_PRIORITY (NORMAL_THREAD_PRIORITY-1)

/* Lowest priority thread -- blink the LED */
#define LED_THREAD_PRIORITY NORMAL_THREAD_PRIORITY

#endif

