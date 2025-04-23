#include "esp_camera.h"
#include <Arduino.h>
#include "line_algorithm.h"
#include "fb_gfx.h"


float get_line_angle_from_frame(camera_fb_t *fb) {
    if (!fb || fb->format != PIXFORMAT_GRAYSCALE || !fb->buf || fb->len == 0) {
        Serial.println("Invalid frame.");
        return NAN;
    }

    const int w = fb->width;
    const int h = fb->height;
    const int scan_start = h - 20;
    const int scan_end   = h - 10;
    const float ANGLE_THRESHOLD = 20.0f;

    uint32_t weighted_sum = 0;
    uint32_t count        = 0;

    // Highlight dark pixels
    for (int y = scan_start; y <= scan_end; ++y) {
        for (int x = 0; x < w; ++x) {
            int idx = y * w + x;
            if (idx >= fb->len) break;
            if (fb->buf[idx] < 100) {
                weighted_sum += x;
                count++;
                fb->buf[idx] = 200;
            }
        }
    }

    // === Always draw vertical lines at angle thresholds ===
    auto draw_threshold_line = [&](float angle) {
        float error = angle / 45.0f;
        int x = int((error * (w / 2.0f)) + (w / 2.0f));
        if (x < 0 || x >= w) return;
        for (int y = 0; y < h; ++y) {
            int idx = y * w + x;
            if (idx >= 0 && idx < fb->len) {
                fb->buf[idx] = 255; // white threshold lines
            }
        }
    };

    draw_threshold_line(-ANGLE_THRESHOLD);  // left threshold
    draw_threshold_line( ANGLE_THRESHOLD);  // right threshold

    // If no dark line found, exit after drawing threshold lines
    if (count == 0) {
        //Serial.println("No line detected.");
        return NAN;
    }

    // Compute angle from center
    float center_x = float(weighted_sum) / float(count);
    float error    = (center_x - (w / 2.0f)) / (w / 2.0f);
    float angle    = error * 45.0f;
    //Serial.printf("Line detected at X = %.2f, angle = %.2f (pixels = %u)\n", center_x, angle, count);

    // Draw 3-pixel-wide center line
    int center = int(center_x + 0.5f);
    int y_min  = (scan_start - 3 < 0)    ? 0     : scan_start - 3;
    int y_max  = (scan_end   + 3 >= h)   ? h - 1 : scan_end   + 3;
    for (int dx = -1; dx <= 1; ++dx) {
        int cx = center + dx;
        if (cx < 0 || cx >= w) continue;
        for (int y = y_min; y <= y_max; ++y) {
            int idx = y * w + cx;
            if (idx >= 0 && idx < fb->len) {
                fb->buf[idx] = 255;   // white center line
            }
        }
    }

    return angle;
}
