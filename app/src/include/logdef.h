/**
 * @brief Header file for global definitions 
 *
 */


#ifndef __LOGDEF_H__
#define __LOGDEF_H__

/* print defines */
#define PRINTF_FLOAT_FORMATI " %d"
#define PRINTF_FLOAT_VALUEI(val) (int32_t)(val)




#define PRINTF_FLOAT_FORMAT " %s%d.%01d"
#define PRINTF_FLOAT_VALUE(val)  (((val) < 0 && (val) > -1.0f) ? "-" : ""),       \
	                         (int32_t)(val),                                  \
                                 (int32_t)((((val) > 0) ? (val) - (int32_t)(val)  \
                                                : (int32_t)(val) - (val))*10)

#define PRINTF_FLOAT_FORMAT2 " %s%d.%02d"
#define PRINTF_FLOAT_VALUE2(val) (((val) < 0 && (val) > -1.0f) ? "-" : ""),   \
                                 (int32_t)(val),                                       \
                                 (int32_t)((((val) > 0) ? (val) - (int32_t)(val)      \
                                                : (int32_t)(val) - (val))*100)

#define PRINTF_FLOAT_FORMAT3 " %d.%03d"
#define PRINTF_FLOAT_VALUE3(val) (int32_t)(val),                                \
                           (int32_t)((((val) >= 0) ? (val) - (int32_t)(val)      \
                                                : (int32_t)(val) - (val))*1000)

#define PRINTF_FLOAT_FORMAT4 " %d.%04d"
#define PRINTF_FLOAT_VALUE4(val) (int32_t)(val),                                 \
                           (int32_t)((((val) >= 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*10000)

#define PRINTF_FLOAT_FORMAT7 " %d.%07d"
#define PRINTF_FLOAT_VALUE7(val) (int32_t)(val),                                 \
                           (int32_t)((((val) >= 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*10000000)



#endif
