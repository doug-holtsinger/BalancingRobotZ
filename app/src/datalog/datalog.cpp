#include <cstdio>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <datalog.h>
#include <stdlib.h>
#include <math.h>

#ifdef DATALOG_ENABLED
LOG_MODULE_REGISTER(DATALOG, CONFIG_SENSOR_LOG_LEVEL);

#define DATALOG_ARRAY_SIZE 8192
#define SLEEP_TIME_MS 100

typedef struct {
        int32_t timestamp;
        union {
            int16_t accel_uncalibrated[3];
            int16_t accel_calibrated[3];
            int32_t gyro_uncalibrated[3];
            float gyro_calibrated[3];
            float quaternion[4];
            float euler_angle[3];
            float pid_kp;
            float pid_ki;
            float pid_kd;
	    float roll;
	    float wheel_speed;
	    float speed_control_sp;
            int16_t motor_driver;
        } u;
        DATALOG_RECORD_t record_type;
        // DATALOG_THREAD_t thread_active;
} DATALOG_ENTRY_t;

static bool stop_collection = false;
static bool trigger_dump = false;
static uint32_t datalog_idx = 0;
static DATALOG_ENTRY_t arr[DATALOG_ARRAY_SIZE];

float last_val;

void datalog_stop_collection()
{
    stop_collection = true;
}

void datalog_trigger_dump()
{
    trigger_dump = true;
}

void datalog_dump()
{
    uint32_t idx_start = (datalog_idx + 1) % DATALOG_ARRAY_SIZE;

    for (int j=0 ; j < DATALOG_ARRAY_SIZE; j++)
    {
        uint32_t i = (idx_start + j) % DATALOG_ARRAY_SIZE;
        switch (arr[i].record_type)
        {
            case DATALOG_ACCEL_UNCAL_RECORD:
                        LOG_DBG("%u,%d,%hd,%hd,%hd", arr[i].timestamp, 
                                arr[i].record_type,
                                arr[i].u.accel_uncalibrated[0],
                                arr[i].u.accel_uncalibrated[1],
                                arr[i].u.accel_uncalibrated[2]);
                break;
            case DATALOG_GYRO_UNCAL_RECORD:
                        LOG_DBG("%u,%d,%d,%d,%d", arr[i].timestamp, 
                                arr[i].record_type,
                                arr[i].u.gyro_uncalibrated[0],
                                arr[i].u.gyro_uncalibrated[1],
                                arr[i].u.gyro_uncalibrated[2]);
                break;
            case DATALOG_ACCEL_CAL_RECORD:
                        LOG_DBG("%u,%d,%d,%d,%d", arr[i].timestamp, 
                                arr[i].record_type,
                                arr[i].u.accel_calibrated[0],
                                arr[i].u.accel_calibrated[1],
                                arr[i].u.accel_calibrated[2]);
                break;
            case DATALOG_GYRO_CAL_RECORD:
                        LOG_DBG("%u,%d,%f,%f,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.gyro_calibrated[0],
                                (double)arr[i].u.gyro_calibrated[1],
                                (double)arr[i].u.gyro_calibrated[2]);
                break;
            case DATALOG_QUAT_RECORD:
                        LOG_DBG("%u,%d,%f,%f,%f,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.quaternion[0],
                                (double)arr[i].u.quaternion[1],
                                (double)arr[i].u.quaternion[2],
                                (double)arr[i].u.quaternion[3]
				);
                break;
            case DATALOG_EULER_ANGLES_RECORD:
                        LOG_DBG("%u,%d,%f,%f,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.euler_angle[0],
                                (double)arr[i].u.euler_angle[1],
                                (double)arr[i].u.euler_angle[2]);
                break;
	    case DATALOG_PID_KP_RECORD:
                        LOG_DBG("%u,%d,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.pid_kp);
                break;
	    case DATALOG_PID_KI_RECORD:
                        LOG_DBG("%u,%d,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.pid_ki);
                break;
	    case DATALOG_PID_KD_RECORD:
                        LOG_DBG("%u,%d,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.pid_kd);
                break;

	    case DATALOG_ROLL:
                        LOG_DBG("%u,%d,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.roll);
                break;
	    case DATALOG_WHEEL_SPEED:
                        LOG_DBG("%u,%d,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.wheel_speed);
                break;
	    case DATALOG_SPEED_CONTROL_SP:
                        LOG_DBG("%u,%d,%f", arr[i].timestamp, 
                                arr[i].record_type,
                                (double)arr[i].u.speed_control_sp);
                break;
            case DATALOG_MOTOR_RECORD:
                        LOG_DBG("%u,%d,%hd", arr[i].timestamp, 
                                arr[i].record_type,
                                arr[i].u.motor_driver);
                break;
        }
        k_msleep(SLEEP_TIME_MS);
    }
}

void datalog_record(DATALOG_RECORD_t record_type, float *record, int32_t *recordi)
{
	if (trigger_dump)
	{
	    datalog_dump();
	    trigger_dump = false;
	    return;
        }

	if (stop_collection)
	{
	    return;
        }

        arr[datalog_idx].timestamp = sys_clock_cycle_get_32();
        arr[datalog_idx].record_type = record_type;
        switch (record_type)
        {
            case DATALOG_ACCEL_UNCAL_RECORD:
                for (int j=0 ; j < 3; j++)
                {
                    arr[datalog_idx].u.accel_uncalibrated[j] = recordi[j];
                }
                break;
            case DATALOG_GYRO_UNCAL_RECORD:
                for (int j=0 ; j < 3; j++)
                {
                    arr[datalog_idx].u.gyro_uncalibrated[j] = recordi[j]; 
                }
                break;
            case DATALOG_ACCEL_CAL_RECORD:
                for (int j=0 ; j < 3; j++)
                {
                    arr[datalog_idx].u.accel_calibrated[j] = recordi[j];
                }
                break;
            case DATALOG_GYRO_CAL_RECORD:
                for (int j=0 ; j < 3; j++)
                {
                    arr[datalog_idx].u.gyro_calibrated[j] = record[j]; 
                }
                break;
            case DATALOG_QUAT_RECORD:
                for (int j=0 ; j < 4; j++)
                {
                    arr[datalog_idx].u.quaternion[j] = record[j];
                }
                break;
            case DATALOG_EULER_ANGLES_RECORD:
                for (int j=0 ; j < 3; j++)
                {
                    arr[datalog_idx].u.euler_angle[j] = record[j];
                }
                break;
	    case DATALOG_PID_KP_RECORD:
                arr[datalog_idx].u.pid_kp = *record;
		break;
	    case DATALOG_PID_KI_RECORD:
                arr[datalog_idx].u.pid_ki = *record;
		break;
	    case DATALOG_PID_KD_RECORD:
                arr[datalog_idx].u.pid_kd = *record;
		break;
	    case DATALOG_ROLL:
                arr[datalog_idx].u.roll = *record;
		break;
	    case DATALOG_WHEEL_SPEED:
                arr[datalog_idx].u.wheel_speed = *record;
		break;
	    case DATALOG_SPEED_CONTROL_SP:
                arr[datalog_idx].u.speed_control_sp = *record;
		break;
            case DATALOG_MOTOR_RECORD:
                arr[datalog_idx].u.motor_driver = *recordi;
                break;
        }
        datalog_idx = (datalog_idx + 1) % DATALOG_ARRAY_SIZE;
}

#endif

