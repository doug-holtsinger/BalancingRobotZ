
#include <app_demux.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(APPHANDLER, CONFIG_SENSOR_LOG_LEVEL);

static AppHandler app_handlers[APP_HANDLERS_MAX];
static APP_CMD_t cmd_min_next = 0; 
static size_t handler_idx = 0;

void appDemuxExecHandler(const APP_CMD_t cmd)
{
    for (size_t i=0 ; i<APP_HANDLERS_MAX ; i++) 
    {
        if (cmd >= app_handlers[i].cmd_min && cmd <= app_handlers[i].cmd_max)
	{
            LOG_DBG("cmd %d %d %d", i, cmd , app_handlers[i].cmd_min);
	    // FIXME -- wake a thread before calling handler?
            app_handlers[i].handler(cmd - app_handlers[i].cmd_min);
	    break;
	}
    }
}

void appDemuxRegisterHandler(const std::function<void(const APP_CMD_t cmd)>& handler, const APP_CMD_t cmd_max)
{
    if (handler_idx >= APP_HANDLERS_MAX)
    {
        LOG_ERR("Too many app handlers registered");
        return;
    }
    const APP_CMD_t app_cmd_max = cmd_min_next + cmd_max;
    AppHandler app_handler{handler, cmd_min_next, app_cmd_max};
    LOG_DBG("reg %d %d %d", handler_idx, cmd_min_next, app_cmd_max);
    app_handlers[handler_idx++] = app_handler;
    cmd_min_next = app_cmd_max + 1; 
}

