#ifndef PROTOCOL_CRSF_DEVICE_H
#define PROTOCOL_CRSF_DEVICE_H

#include <freertos/FreeRTOS.h>

#include "protocol_crsf.h"
#include "protocol_crsf_device_param.h"

#define CRSF_DEVICE_FRAME_QUEUE_SIZE 4

class CrsfDevice {
    public:
        CrsfDevice(CrsfDeviceFolder *root);

        void init();
        void deinit();

        void devicePing(const crsf_payload_device_ping_t &payload);
        void paramRead(const crsf_payload_parameter_read_t &payload);
        void paramWrite(const crsf_payload_parameter_write_t &payload);

        crsf_frame_t *dequeueFrame();

        bool isActive() const;

    private:
        void _compose_device_info(crsf_payload_device_info_t &payload);
        void _compose_param_entry(crsf_payload_parameter_entry_t &payload, uint8_t index);
        void _compose_elrs_status(crsf_payload_elrs_status_t &payload);

        crsf_frame_t _frame_queue[CRSF_DEVICE_FRAME_QUEUE_SIZE];
        uint8_t _frame_queue_cursor;
        bool _is_active;
        int64_t _last_activity_us;
        CrsfDeviceFolder *_root;
};

#endif
