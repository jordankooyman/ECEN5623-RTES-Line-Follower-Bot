#pragma once
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

float get_line_angle_from_frame(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif
