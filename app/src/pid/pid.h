
#ifndef __PID__H
#define __PID__H

#include <cstdio>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "notify.h"
#include "logdef.h"
#include "ble_svcs.h"

#include <vector>
#include "PIDCmd.h"

#include "param_store.h"
#include "datalog.h"
#include "pid_num.h"

#define PID_KP_DEFAULT 1.0
#define PID_KI_DEFAULT 0.0
#define PID_KD_DEFAULT 0.0
#define PID_SP_DEFAULT 0.0

#define PID_KP_VALID	0x0001
#define PID_KI_VALID	0x0002
#define PID_KD_VALID	0x0004
#define PID_PARAM_VALID	0x0007

constexpr uint32_t PID_ERROR_HISTORY_DEPTH = 6;

typedef struct 
{
    float KP {PID_KP_DEFAULT};
    float KI {PID_KI_DEFAULT};
    float KD {PID_KD_DEFAULT};
    float SP {PID_SP_DEFAULT};
} pidParameters_t;

template <typename T>
class PID {
    public:
        PID(const pidParameters_t i_param, const pidParameters_t i_increment, 
			const T i_pidControlMax, const uint16_t paramRecordKey, 
			const uint16_t i_pidNum, const bool i_reverse_output = false,
			const bool i_low_pass_filter = false) :
		pidParams(i_param),
		pidParamsInit(i_param),
		pidIncrement(i_increment),
		controlSettingMax(i_pidControlMax),
		param_store(paramRecordKey),
		pidNum(i_pidNum),
		reverseOutput(i_reverse_output),
		lowPassFilter(i_low_pass_filter),
		l_PV(0.0)
	{
	}

	void init()
	{
            pidParameters_t pp;
	    // FIXME -- changing pidParams if flash is already valid does not change param_store.get() values
            param_store.init(&pidParams, true);
            pp = param_store.get();
	    init_params(pp);
	}

	void init_params(pidParameters_t params)
	{
            pidParams = params;
	}

	void params_save()
	{
            param_store.set(&pidParams); 
	}

	void params_reset()
	{
            param_store.set(&pidParamsInit);
            pidParams = pidParamsInit;
	}

        void cmd(const PID_CMD_t i_cmd)
        {
            switch (i_cmd)
            {
		case PID_CMD_t::PID_KP_UP:
                    pidParams.KP += pidIncrement.KP;
                    break;
		case PID_CMD_t::PID_KP_DOWN:
                    pidParams.KP -= pidIncrement.KP;
		    if (pidParams.KP < 0.0f)
		    {
                        pidParams.KP = 0.0f;
		    }
                    break;
		case PID_CMD_t::PID_KI_UP:
                    pidParams.KI += pidIncrement.KI;
                    break;
		case PID_CMD_t::PID_KI_DOWN:
                    pidParams.KI -= pidIncrement.KI;
		    if (pidParams.KI < 0.0f)
		    {
                        pidParams.KI = 0.0f;
		    }
                    break;
		case PID_CMD_t::PID_KD_UP:
                    pidParams.KD += pidIncrement.KD;
                    break;
		case PID_CMD_t::PID_KD_DOWN:
                    pidParams.KD -= pidIncrement.KD;
		    if (pidParams.KD < 0.0f)
		    {
                        pidParams.KD = 0.0f;
		    }
                    break;
		case PID_CMD_t::PID_SP_UP:
                    pidParams.SP += pidIncrement.SP;
                    break;
		case PID_CMD_t::PID_SP_DOWN:
                    pidParams.SP -= pidIncrement.SP;
                    break;
		case PID_CMD_t::PID_PARAMS_SAVE:
		    params_save();
                    break;
		case PID_CMD_t::PID_PARAMS_RESET:
		    params_reset();
                    break;
                default:
                    break;
            }
        }

	void send_all_client_data()
	{
            char s[NOTIFY_PRINT_STR_MAX_LEN];

	    if ( !ble_svcs_connected() ) {
                return;
            }
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_KP, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.KP));
            send_client_data(s);

            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_KI, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.KI));
            send_client_data(s);

            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_KD, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.KD));
            send_client_data(s);

            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_SP, pidNum),
                PRINTF_FLOAT_VALUE2(pidParams.SP));
            send_client_data(s);

	    // For PID=1 (speed control), this is the motor encoding in floating point
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT,
                PID_NOTIFY(PID_PV, pidNum),
                PRINTF_FLOAT_VALUE(l_PV_save));
            send_client_data(s);

	    // For PID=1 (speed control), this becomes the setpoint for the Motor PID
            snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2,
                PID_NOTIFY(PID_OUTPUT, pidNum),
                PRINTF_FLOAT_VALUE2(controlSetting));
            send_client_data(s);
	}

	void setParameters(const pidParameters_t i_param, const uint16_t flags)
	{
            if (flags & PID_KP_VALID)
	    {
                pidParams.KP = i_param.KP;
	    }
            if (flags & PID_KI_VALID)
	    {
                pidParams.KI = i_param.KI;
	    }
            if (flags & PID_KD_VALID)
	    {
                pidParams.KD = i_param.KD;
	    }
	}

        void setSP(const float i_SP)
	{
            pidParams.SP = i_SP;
	}

        float getSP()
	{
            return pidParams.SP; 
	}

	T update(const float i_PV)
	{
#ifdef DATALOG_ENABLED
	    float controlSetFloatContrib = 0.0;
#endif
            if (lowPassFilter)
	    {
                l_PV = 0.8f * l_PV + 0.2f * i_PV;
	    } else {
                l_PV = i_PV; 
	    }
	    l_PV_save = i_PV;
            const float errorDiff = l_PV - pidParams.SP;

	    // error history
	    errorHistory.push_back(errorDiff);
	    if (errorHistory.size() > PID_ERROR_HISTORY_DEPTH)
	    {
	        errorHistory.erase(errorHistory.cbegin());
	    }

            // KP contribution
	    float controlSetFloat = pidParams.KP * errorDiff;
#ifdef DATALOG_ENABLED
	    if (pidNum == MOTOR_PID_NUM)
	    {
                controlSetFloatContrib = controlSetFloat;
                datalog_record(DATALOG_PID_KP_RECORD, &controlSetFloatContrib, nullptr);
	    }
#endif
	    //
            // KI contribution
	    if (pidParams.KI > 0.0f)
	    {
	        for (short unsigned int i=0 ; i < errorHistory.size() ; ++i)
	        {
		    controlSetFloat += (pidParams.KI * errorHistory[i]);
	        }
	    }

#ifdef DATALOG_ENABLED
            if (pidNum == MOTOR_PID_NUM)
            {
	        controlSetFloatContrib = controlSetFloat - controlSetFloatContrib;
                datalog_record(DATALOG_PID_KI_RECORD, &controlSetFloatContrib, nullptr);
            }
            controlSetFloatContrib = controlSetFloat;
#endif

	    // KD contribution
	    if (pidParams.KD > 0.0f)
	    {
                size_t errLen = errorHistory.size();
	        if (errLen >= 2)
	        {
	            controlSetFloat += (pidParams.KD * (errorHistory[errLen-1] - errorHistory[errLen-2]));
	        }
	    }
#ifdef DATALOG_ENABLED
            if (pidNum == MOTOR_PID_NUM)
            {
                controlSetFloatContrib = controlSetFloat - controlSetFloatContrib;
                datalog_record(DATALOG_PID_KD_RECORD, &controlSetFloatContrib, nullptr);
	    }
#endif

	    // Max setting
	    if (controlSetFloat > controlSettingMax)
	    {
                controlSetFloat = controlSettingMax;
	    } else if (controlSetFloat < -controlSettingMax)
	    {
                controlSetFloat = -controlSettingMax;
	    } 

	    if (typeid(controlSetting) != typeid(float)) {
		// round to integer
	        // controlSetting = floor(controlSetFloat + 0.5f);
	        controlSetting = int(controlSetFloat + 0.5f);
	    } else {
	        controlSetting = controlSetFloat;
	    }

	    if (reverseOutput)
	    {
                controlSetting = -controlSetting;
	    }

            return controlSetting;
	}
    private:
	pidParameters_t pidParams;		// local copy of PID parameters
	pidParameters_t pidParamsInit;		// local copy of PID parameters passed in from init
	const pidParameters_t pidIncrement;		// local copy of PID Increment values
	const T controlSettingMax {0};
	T controlSetting {0};
        ParamStore<pidParameters_t> param_store;
	std::vector<float> errorHistory;
        const uint16_t pidNum;
	const bool reverseOutput;
	const bool lowPassFilter;

	float l_PV;				// process variable
	float l_PV_save;
	float controlSettingSave;
};
#endif

