#include <cstdio>
#include <zephyr/logging/log.h>
#include <perf.h>
#include <stdlib.h>
#include <math.h>

LOG_MODULE_REGISTER(PERF, CONFIG_SENSOR_LOG_LEVEL);

#ifdef MEASURE_TIME_DELAYS
// 0 -- time between abs accel uncalibrated Y transition from < 25 to > 50 to calibrated Y transition from < 25 to > 50
// 1 -- time between accel calibrated Y transition from > 50  to Q1 quaternion transition > 0.01
// 2 -- time between Q1 quaternion transition from >0.01 to roll > 1
// 3 -- time between roll > 1 and driver > 500
static bool perf_above_thresh[2][NUM_TIME_MEASUREMENTS] = {{false, false, false, false, false}, {false, false, false, false, false}};
static bool perf_above_thresh_prev[2][NUM_TIME_MEASUREMENTS] = {{true, true, true, true, true}, {true, true, true, true, true}};
static bool time_start_valid[NUM_TIME_MEASUREMENTS] = {false, false, false, false, false};
static int32_t time_start[NUM_TIME_MEASUREMENTS] = {0L,0L,0L,0L,0L};
static bool time_dif_valid[NUM_TIME_MEASUREMENTS] = {false, false, false, false, false};
static int32_t time_dif[NUM_TIME_MEASUREMENTS] = {0L,0L,0L,0L,0L};

void perf_threshold_check_int(int idx, int start_end, int16_t val, int16_t threshold)
{
    perf_above_thresh_prev[start_end][idx] = perf_above_thresh[start_end][idx];
    if (abs(val) >= threshold)
    {
        perf_above_thresh[start_end][idx] = true;
    } else
    {
        perf_above_thresh[start_end][idx] = false;
	time_start_valid[idx] = false;
    }
}

void perf_threshold_check(int idx, int start_end, float val, float threshold)
{
    perf_above_thresh_prev[start_end][idx] = perf_above_thresh[start_end][idx];
    if (fabsf(val) >= threshold)
    {
        perf_above_thresh[start_end][idx] = true;
    } else
    {
        perf_above_thresh[start_end][idx] = false;
	time_start_valid[idx] = false;
    }
}

void perf_start(int idx, float val, float threshold)
{
    perf_threshold_check(idx, 0, val, threshold);
    // start measurement when the value crosses the threshold positively and there is no active measurement.
    if (!perf_above_thresh_prev[0][idx] && perf_above_thresh[0][idx] && !time_start_valid[idx] && !time_dif_valid[idx]) 
    {
        LOG_DBG("start i=%d t=%d pt=%d sv=%d %f\n", idx, perf_above_thresh_prev[0][idx], perf_above_thresh[0][idx], time_start_valid[idx], val);
        time_start[idx] = sys_clock_cycle_get_32();
        time_start_valid[idx] = true;
    }
}
void perf_end(int idx, float val, float threshold)
{
    perf_threshold_check(idx, 1, val, threshold);
    if (time_start_valid[idx] && !(perf_above_thresh[1][idx] && !time_dif_valid[idx]))
    {
        LOG_DBG("endts %d %d %d %f\n", idx, perf_above_thresh[1][idx], time_dif_valid[idx], val);
    }
    if (perf_above_thresh[1][idx] && time_start_valid[idx] && !time_dif_valid[idx])
    {
        time_dif[idx] = sys_clock_cycle_get_32() - time_start[idx];
        time_dif_valid[idx] = true;
        LOG_DBG("end %d %f\n", idx, val);
    }
}
void perf_end_int(int idx, int16_t val, int16_t threshold)
{
    perf_threshold_check_int(idx, 1, val, threshold);
    if (time_start_valid[idx] && !(perf_above_thresh[1][idx] && !time_dif_valid[idx]))
    {
        LOG_DBG("endts %d %d %d %f\n", idx, perf_above_thresh[1][idx], time_dif_valid[idx], val);
    }
    if (perf_above_thresh[1][idx] && time_start_valid[idx] && !time_dif_valid[idx])
    {
        time_dif[idx] = sys_clock_cycle_get_32() - time_start[idx];
        time_dif_valid[idx] = true;
        LOG_DBG("end %d %f\n", idx, val);
    }
}
void perf_print()
{
    for (int32_t i=0 ; i < NUM_TIME_MEASUREMENTS ; i++)
    {
	if (time_dif_valid[i])
	{
            LOG_DBG("time %d dif %d\n", i, time_dif[i]);
            time_start_valid[i] = false;
	    time_dif_valid[i] = false;
	}
    }
}
#endif

