#ifndef MIM_SKYMAP_H
#define MIM_SKYMAP_H

#include <freertos/FreeRTOS.h>

typedef struct mim_skymap_ctx_s *mim_skymap_handle_t;

esp_err_t mim_skymap_init(BaseType_t priority, mim_skymap_handle_t *handle);

esp_err_t mim_skymap_deinit(mim_skymap_handle_t handle);

bool mim_skymap_is_ready(const mim_skymap_handle_t handle);

bool mim_skymap_is_engaging(const mim_skymap_handle_t handle);

void mim_skymap_set_engaging(mim_skymap_handle_t handle, bool enable);

#endif
