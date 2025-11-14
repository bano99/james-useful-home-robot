/*
 * LVGL Configuration for James Remote Control
 * Minimal configuration for LilyGO T-Display S3 AMOLED
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

// Color settings
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

// Memory settings
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (48U * 1024U)

// Display settings (Portrait mode for remote control)
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 536
#define LV_DPI_DEF 130

// Rendering settings
#define LV_DISP_DEF_REFR_PERIOD 30
#define LV_INDEV_DEF_READ_PERIOD 30

// Feature settings
#define LV_USE_ANIMATION 1
#define LV_USE_SHADOW 0
#define LV_USE_BLEND_MODES 0
#define LV_USE_OPA_SCALE 0
#define LV_USE_IMG_TRANSFORM 0

// Font settings
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

// Widget settings
#define LV_USE_ARC 0
#define LV_USE_BAR 1
#define LV_USE_BTN 1
#define LV_USE_BTNMATRIX 0
#define LV_USE_CANVAS 0
#define LV_USE_CHECKBOX 0
#define LV_USE_DROPDOWN 0
#define LV_USE_IMG 0
#define LV_USE_LABEL 1
#define LV_USE_LINE 0
#define LV_USE_ROLLER 0
#define LV_USE_SLIDER 0
#define LV_USE_SWITCH 0
#define LV_USE_TEXTAREA 0
#define LV_USE_TABLE 0

// Theme settings
#define LV_USE_THEME_DEFAULT 1
#define LV_USE_THEME_BASIC 1

// Logging
#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 1

// Performance monitoring
#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0

// Asserts
#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_STYLE 0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ 0

// Tick settings
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

#endif // LV_CONF_H
