#ifndef PROTOCOL_CRSF_DEVICE_PARAM_H
#define PROTOCOL_CRSF_DEVICE_PARAM_H

#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <sys/param.h>

#include "protocol_crsf.h"

#define CRSF_DEVICE_PARAM_MAX_COUNT 128

static const char *TAG_CRSF_DEVICE_PARAM = "CRSF_DEV_PARAM";

class CrsfDeviceFolder;

class CrsfDeviceParam {
public:
    CrsfDeviceParam(CrsfDeviceFolder *folder): _index(_index_counter++), _folder(folder) {
        assert(_index_counter <= CRSF_DEVICE_PARAM_MAX_COUNT);
        _index_param[_index] = this;
    }
    virtual ~CrsfDeviceParam() {}

    uint8_t getIndex() const { return _index; }
    uint8_t getParentIndex() const;

    virtual void render(crsf_payload_parameter_entry_t &payload) = 0;
    virtual void setValue(crsf_payload_parameter_value_t value) = 0;

    static CrsfDeviceParam *getParam(uint8_t index);
    static uint8_t getParamCount();

protected:
    static CrsfDeviceParam *_index_param[CRSF_DEVICE_PARAM_MAX_COUNT];
    static uint8_t _index_counter;
    uint8_t _index;
    CrsfDeviceFolder *_folder;
};

class CrsfDeviceFolder : public CrsfDeviceParam {
public:
    CrsfDeviceFolder(CrsfDeviceFolder *parent_folder): CrsfDeviceParam(parent_folder) {}
    virtual ~CrsfDeviceFolder() {}

    void render(crsf_payload_parameter_entry_t &payload) {
        payload.type = CRSF_PARAM_TYPE_FOLDER;
        payload.label = getName();
        payload.value.value_folder = { .name = getName() };
    }
    void setValue(crsf_payload_parameter_value_t value) { }

    virtual const char *getName() const = 0;
};

class CrsfDeviceParamCommand : public CrsfDeviceParam {
public:
    CrsfDeviceParamCommand(CrsfDeviceFolder *parent) : CrsfDeviceParam(parent),
        _step(CRSF_COMMAND_STEP_IDLE) {}
    virtual ~CrsfDeviceParamCommand() {}

    virtual void onExecute() = 0;
    virtual void onCancel() = 0;
    virtual bool isRunning() = 0;

    void render(crsf_payload_parameter_entry_t &payload) {
        payload.type = CRSF_PARAM_TYPE_COMMAND;
        payload.label = getName();
        payload.value.value_command = {
            .step = _step,
            .timeout = 50,
            .status = getStatus(_step),
        };
    }

    void setValue(crsf_payload_parameter_value_t value) {
        switch (value) {
            case CRSF_COMMAND_STEP_IDLE:
                break;
            case CRSF_COMMAND_STEP_CLICK:
                _step = CRSF_COMMAND_STEP_ASKCONFIRM;
                break;
            case CRSF_COMMAND_STEP_CONFIRMED:
                _step = CRSF_COMMAND_STEP_EXECUTING;
                onExecute();
                break;
            case CRSF_COMMAND_STEP_CANCEL:
                _step = CRSF_COMMAND_STEP_IDLE;
                onCancel();
                break;
            case CRSF_COMMAND_STEP_QUERY:
                if (_step == CRSF_COMMAND_STEP_EXECUTING && !isRunning()) {
                    _step = CRSF_COMMAND_STEP_IDLE;
                }
                break;
            default:
            case CRSF_COMMAND_STEP_EXECUTING:
            case CRSF_COMMAND_STEP_ASKCONFIRM:
                assert(false);
        }
    }
protected:
    virtual const char *getName() const = 0;
    virtual const char *getStatus(crsf_device_command_step_t step) const = 0;

private:
    crsf_device_command_step_t _step;
};

class CrsfDeviceParamInfo : public CrsfDeviceParam {
public:
    CrsfDeviceParamInfo(CrsfDeviceFolder *folder, const char *name, const char *info): CrsfDeviceParam(folder), _name(name), _info(info) { }

    void render(crsf_payload_parameter_entry_t &payload) {
        payload.type = CRSF_PARAM_TYPE_INFO;
        payload.label = _name;
        payload.value.value_info = {.name = _info};
    }
    void setValue(crsf_payload_parameter_value_t value) { }

protected:
    const char *_name;
    const char *_info;
};

class CrsfDeviceFolderRoot : public CrsfDeviceFolder {
public:
    CrsfDeviceFolderRoot() : CrsfDeviceFolder(NULL),
        _param_version(this, "version", esp_app_get_description()->version) {
    }

    const char *getName() const { return "__root__"; }

private:
    CrsfDeviceParamInfo _param_version;
};

#endif
