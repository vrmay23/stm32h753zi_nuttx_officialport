/****************************************************************************
 * apps/hmi_manager/uiux/widgets.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or
 * more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information regarding
 * copyright ownership. The ASF licenses this file to you under the
 * Apache License, Version 2.0 (the "License"); you may not use
 * this file except in compliance with the License.  You may obtain
 * a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Dashboard Widget API - Retro T3 v3
 * LVGL 9.2 | ST7796 480x320 RGB565
 *
 * Coordinate source: cluster_final.svg (Inkscape)
 *
 * Hub centres (needle pivots) and arc centres are extracted
 * separately from the SVG since they do NOT coincide:
 *   Left  hub cx=114 cy=141, arc centre cx=108 cy=142
 *   Right hub cx=364 cy=142, arc centre cx=367 cy=143
 *
 ****************************************************************************/

#ifndef __APPS_HMI_MANAGER_UIUX_WIDGETS_H
#define __APPS_HMI_MANAGER_UIUX_WIDGETS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <lvgl/lvgl.h>
#include <stdint.h>

/****************************************************************************
 * Direction / Mode Constants
 ****************************************************************************/

#define WIDGET_DIR_NEUTRAL    0
#define WIDGET_DIR_FORWARD    1
#define WIDGET_DIR_REVERSE    2

#define WIDGET_MODE_STANDARD  0
#define WIDGET_MODE_SPORT     1
#define WIDGET_MODE_ECO       2

/****************************************************************************
 * Colour Palette
 ****************************************************************************/

#define CLR_CYAN        lv_color_hex(0x00E5FF)
#define CLR_MAGENTA     lv_color_hex(0xC800FF)
#define CLR_RED_ARC     lv_color_hex(0xFF0040)
#define CLR_WHITE       lv_color_hex(0xFFFFFF)
#define CLR_AMBER       lv_color_hex(0xFFA500)
#define CLR_AMBER_HOT   lv_color_hex(0xFF6600)
#define CLR_RED         lv_color_hex(0xFF2000)
#define CLR_GREEN       lv_color_hex(0x00CC44)
#define CLR_GREEN_MODE  lv_color_hex(0x00DD00)
#define CLR_GREEN_TURN  lv_color_hex(0x00FF66)
#define CLR_DARK_BG     lv_color_hex(0x0A0A0F)

/****************************************************************************
 * Arc fill opacity (0..255)
 ****************************************************************************/

#define ARC_FILL_OPA    140

/****************************************************************************
 * Left Gauge (Speed outer + RPM inner)
 *
 * SVG hub ellipse: cx=114.43 cy=140.62 rx=40.7 ry=38.9
 * SVG outer arc path centre: cx=108 cy=142 r=78
 * SVG inner arc path centre: cx=108 cy=142 r=55
 * Arc sweep: 220 deg (gap at bottom)
 *
 * Needles pivot from the HUB centre (visual anchor).
 * Arcs are positioned at the ARC centre (path geometry).
 ****************************************************************************/

/* Hub = needle pivot */

#define SPD_HUB_CX       114
#define SPD_HUB_CY       141

/* Arc overlay position */

#define SPD_ARC_CX        108
#define SPD_ARC_CY        142
#define SPD_R_OUTER        78
#define SPD_R_INNER        55

/* lv_arc angles: 0=right(3h), CW */

#define SPD_ARC_START     110
#define SPD_ARC_END       250

#define SPD_MAX_KMH       160
#define RPM_MAX         10000

/* Needle lengths */

#define SPD_NEEDLE_LEN     68
#define RPM_NEEDLE_LEN     45

/* Needle angles: 0=12h, CW */

#define NEEDLE_START_DEG  210
#define NEEDLE_SWEEP_DEG  220

/****************************************************************************
 * Right Gauge (Voltage outer + Ampere inner)
 *
 * SVG hub ellipse: cx=363.6 cy=142.2 rx=45.3 ry=38.9
 * SVG outer arc (path200+matrix): cx=367 cy=143 rx=87
 * Inner arc: same centre, r=55
 ****************************************************************************/

/* Hub = needle pivot */

#define REG_HUB_CX       364
#define REG_HUB_CY       142

/* Arc overlay position */

#define REG_ARC_CX        367
#define REG_ARC_CY        143
#define REG_R_OUTER        87
#define REG_R_INNER        55

#define REG_ARC_START     110
#define REG_ARC_END       250

#define REG_MAX_AMPS      500
#define REG_MAX_VOLTS     600
#define REG_NEEDLE_LEN     68

/****************************************************************************
 * Turn Signals
 * SVG left: rect(6,7) 52x28 | right: rect(423,7) 52x28
 ****************************************************************************/

#define TURN_L_X            6
#define TURN_L_Y            7
#define TURN_L_W           52
#define TURN_L_H           28

#define TURN_R_X          423
#define TURN_R_Y            7
#define TURN_R_W           52
#define TURN_R_H           28

#define TURN_BLINK_MS     500

/****************************************************************************
 * Gear Hexagon
 * SVG polygon + matrix(1.4715,0,0,1.506,-109.45,-236.91)
 * Computed centre: (242, 82), bbox 59x57
 ****************************************************************************/

#define GEAR_CX           242
#define GEAR_CY            82
#define GEAR_W             50
#define GEAR_H             50

/****************************************************************************
 * Drive Mode - SVG rect(204, 238) 80x25
 ****************************************************************************/

#define MODE_X            204
#define MODE_Y            238
#define MODE_W             80
#define MODE_H             25

/****************************************************************************
 * Clock Frame
 * SVG notch top: x=195..293 y=279, bottom: y=319
 * Centre: (244, 299), usable width ~98
 ****************************************************************************/

#define CLK_X             195
#define CLK_Y             279
#define CLK_W              98
#define CLK_H              40

/****************************************************************************
 * Battery Bar - SVG rect(41, 266) 122x22
 ****************************************************************************/

#define BAT_X              41
#define BAT_Y             266
#define BAT_W             122
#define BAT_H              22

/****************************************************************************
 * Bottom-Left Labels
 * Remaining time: SVG rect(40, 294) 46x18
 * Battery temp:   SVG rect(117, 295) 46x18
 ****************************************************************************/

#define REM_TIME_X         40
#define REM_TIME_Y        294
#define REM_TIME_W         46
#define REM_TIME_H         18

#define BATT_TEMP_X       117
#define BATT_TEMP_Y       295
#define BATT_TEMP_W        46
#define BATT_TEMP_H        18

/****************************************************************************
 * Bottom-Right Labels
 * Odometer: SVG rect(326, 291) 37x21
 * SoC:      SVG rect(379, 291) 37x21
 * Ext Temp: SVG rect(432, 291) 37x21
 ****************************************************************************/

#define ODO_X             326
#define ODO_Y             291
#define ODO_W              37
#define ODO_H              21

#define SOC_X             379
#define SOC_Y             291
#define SOC_W              37
#define SOC_H              21

#define EXT_TEMP_X        432
#define EXT_TEMP_Y        291
#define EXT_TEMP_W         37
#define EXT_TEMP_H         21

/****************************************************************************
 * Battery voltage scale
 ****************************************************************************/

#define BAT_VOLTAGE_MAX_DV  960
#define BAT_VOLTAGE_MIN_DV  600
#define BAT_WARN_PCT         26
#define BAT_WARN_LOW_PCT     11
#define BAT_CRIT_PCT          1

/****************************************************************************
 * Public API
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

int  widgets_init(void);

void widgets_set_speed(int kmh);
void widgets_set_rpm(int rpm);
void widgets_set_current(int amps);
void widgets_set_voltage_arc(int volts);

void widgets_turn_left(bool on);
void widgets_turn_right(bool on);
void widgets_turn_hazard(bool on);
void widgets_turn_off(void);

void widgets_set_direction(int dir);
void widgets_set_mode(int mode);
void widgets_set_battery_pct(int pct);
void widgets_set_voltage(int dv);
void widgets_set_odometer(uint32_t km);
void widgets_set_ext_temp(int temp_c);
void widgets_set_soc(int pct);
void widgets_set_remaining_time(int minutes);
void widgets_set_batt_temp(int temp_c);
void widgets_set_clock(int hour, int minute);
void widgets_set_warning(const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_HMI_MANAGER_UIUX_WIDGETS_H */
