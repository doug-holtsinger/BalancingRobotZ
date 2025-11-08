#ifndef __PERF__H
#define __PERF__H

#define NUM_TIME_MEASUREMENTS 5

// #define MEASURE_TIME_DELAYS

#ifdef MEASURE_TIME_DELAYS
void perf_ready(int idx, float val, float threshold);
void perf_start(int idx, float val, float threshold);
void perf_end(int idx, float val, float threshold);
void perf_end_int(int idx, int16_t val, int16_t threshold);
void perf_print();
#endif

#endif
