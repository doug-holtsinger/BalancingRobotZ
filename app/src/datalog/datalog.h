#ifndef __DATALOG__H
#define __DATALOG__H

#define DATALOG_ENABLED

typedef enum
{
	DATALOG_MAIN_THREAD,
	DATALOG_IMU_THREAD,
	DATALOG_MOTOR_THREAD
} DATALOG_THREAD_t;

typedef enum
{
	DATALOG_ACCEL_UNCAL_RECORD,   // 0
	DATALOG_ACCEL_CAL_RECORD,     // 1
	DATALOG_GYRO_UNCAL_RECORD,    // 2
	DATALOG_GYRO_CAL_RECORD,      // 3
	DATALOG_QUAT_RECORD,          // 4
	DATALOG_EULER_ANGLES_RECORD,  // 5
	DATALOG_PID_KP_RECORD,        // 6
	DATALOG_PID_KI_RECORD,        // 7
	DATALOG_PID_KD_RECORD,        // 8
	DATALOG_ROLL,                 // 9
	DATALOG_WHEEL_SPEED,          // 10
	DATALOG_SPEED_CONTROL_SP,     // 11
	DATALOG_MOTOR_RECORD          // 12
} DATALOG_RECORD_t;

void datalog_stop_collection();
void datalog_dump();
void datalog_trigger_dump();
void datalog_record(DATALOG_RECORD_t record_type, float *record, int32_t *recordi);

#endif
