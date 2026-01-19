/**
 *
 */

#ifndef __PARAM_STORE_H__
#define __PARAM_STORE_H__

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_version.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#define NVS_PARTITION           storage_partition
#define NVS_PARTITION_DEVICE    FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET    FIXED_PARTITION_OFFSET(NVS_PARTITION)

// Logging is controlled by the module that includes this file.

template <typename T>
class ParamStore {
public:
    ParamStore(const uint16_t data_id) :
        nvs_data_id(data_id)
    {
    }

    int init(T *param_store_data_ptr, bool writeStore)
    {
	int rc = 0;
	int bytes_read = 0;
        fs.flash_device = NVS_PARTITION_DEVICE;
        if (!device_is_ready(fs.flash_device)) {
            LOG_ERR("Flash device %s is not ready", fs.flash_device->name);
            return 1;
        }

        fs.offset = NVS_PARTITION_OFFSET;
	LOG_INF("NVS offset 0x%lx", fs.offset);
        rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
        if (rc) {
            LOG_ERR("Flash can't get page info: %d", rc);
            return rc;
        }
        fs.sector_size = info.size;
        fs.sector_count = 3U;

        rc = nvs_mount(&fs);
        if (rc) {
            LOG_ERR("Flash Init failed: %d", rc);
            return rc;
        }

	bytes_read = nvs_read(&fs, nvs_data_id, &param_data, sizeof(param_data));
        LOG_DBG("Flash record %d bytes read: %d", nvs_data_id, bytes_read);
	if (bytes_read <= 0 || writeStore)
	{
            /* record not found, initialize it */
            rc = this->set(param_store_data_ptr);
	}
        return rc;
    }

    T get(void)
    {
        return param_data;
    }

    int set(T* param_store_data_ptr)
    {
	int bytes_write;
	bytes_write = nvs_write(&fs, nvs_data_id, param_store_data_ptr, sizeof(param_data));
	if (bytes_write != sizeof(param_data))
	{
            LOG_ERR("Flash write failed: %d %d", bytes_write, sizeof(param_data));
            return -1;
	} 
        LOG_DBG("Flash record %d written", nvs_data_id);
        param_data = *param_store_data_ptr;
	return 0;
    }

private:
    struct nvs_fs fs;
    struct flash_pages_info info;
    uint16_t nvs_data_id;
    T param_data;
};

#endif



