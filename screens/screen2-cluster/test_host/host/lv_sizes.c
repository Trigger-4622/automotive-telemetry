/*
 * Compiled twice by run.py to assembly - natively and with -m32 - and never
 * linked: each constant below appears as a number in the output. The 32-bit
 * sizes match the ESP32's (ILP32: 4-byte pointers and size_t).
 */
#include "lvgl.h"

/* Private to lv_event.c; mirrored field for field. */
typedef struct {
    lv_event_cb_t cb;
    void *user_data;
    lv_event_code_t filter : 8;
} lv_event_dsc_t;

#define SZ(t) const unsigned SZ_##t = sizeof(t);
SZ(lv_obj_t)
SZ(lv_label_t)
SZ(lv_arc_t)
SZ(lv_bar_t)
SZ(lv_img_t)
SZ(lv_meter_t)
SZ(lv_chart_t)
SZ(_lv_obj_style_t)
SZ(lv_style_t)
SZ(lv_style_value_t)
SZ(lv_style_prop_t)
SZ(_lv_obj_spec_attr_t)
SZ(lv_event_dsc_t)
SZ(lv_meter_scale_t)
SZ(lv_meter_indicator_t)
SZ(lv_chart_series_t)
SZ(lv_anim_t)
SZ(lv_timer_t)
SZ(lv_coord_t)
const unsigned SZ_ptr = sizeof(void *);
const unsigned SZ_size_t = sizeof(size_t);
