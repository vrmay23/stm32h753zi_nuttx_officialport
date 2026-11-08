/****************************************************************************
 * apps/hmi_manager/uiux/widgets.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Dashboard Widget API - Retro T3 Theme
 * LVGL 9.2  |  ST7796 480x320 RGB565
 *
 * ── Coordinate source ────────────────────────────────────────────────────
 *
 *  ALL coordinates derived from:
 *    lvgl_placeholder_retro_t3.svg  (Inkscape, 126.76mm × 84.57mm)
 *  Converted to LCD pixels at scale 3.811 px/mm (x) × 3.820 px/mm (y).
 *  Source of truth: the SVG file.  Do NOT hand-edit these numbers —
 *  re-run the conversion script if the SVG changes.
 *
 * ── Widget layout (480×320 LCD pixels) ───────────────────────────────────
 *
 *  ┌────────────────────────────────────────────────────────────────┐
 *  │  ╔══════════════╗  [GEAR 226,81 28×30]     ╔══════════════╗    │
 *  │  ║  Speedo      ║  [MODE 202,135 81×14]    ║  Regen       ║    │
 *  │  ║  cx=110      ║                          ║  cx=375      ║    │
 *  │  ║  cy=116      ║                          ║  cy=116      ║    │
 *  │  ║  r_out=79    ║                          ║  r_out=79    ║    │
 *  │  ║  r_in =43    ║                          ║  r_in =43    ║    │
 *  │  ╚══════════════╝                          ╚══════════════╝    │
 *  │                                                                │
 *  │  [RED  39,233 100×14]          [PINK  343,234 102×11]          │
 *  │  [YEL  71,268  90×13]          [PURP  381,259  63× 9]          │
 *  │                                [ORNG  382,278  63×10]          │
 *  │              [CREAM 195,286 91×27]                             │
 *  └────────────────────────────────────────────────────────────────┘
 *
 * ── Needle geometry ──────────────────────────────────────────────────────
 *
 *  Both gauges share the same angular sweep:
 *    Start : 135°  (7 o'clock  = minimum value)
 *    End   : 405°  (5 o'clock  = maximum value)
 *    Sweep : 270°
 *
 *  Speedometer has TWO co-axial needles (same pivot cx=110,cy=116):
 *    Speed needle : white,  length = 70 px  (≈ r_outer - 9)
 *    RPM   needle : amber,  length = 55 px  (≈ r_inner + 12)
 *
 *  Regen gauge has ONE needle (pivot cx=375,cy=116):
 *    Current needle: amber, length = 70 px
 *
 *  Needle tip formula (LVGL angle: 0°=12-o'clock, CW positive):
 *    tip_x = cx + length × sin(angle_rad)
 *    tip_y = cy − length × cos(angle_rad)
 *
 * ── Font requirements (lv_conf.h / Kconfig) ──────────────────────────────
 *
 *  CONFIG_LV_FONT_MONTSERRAT_28=y   (gear letter N/D/R/P)
 *  CONFIG_LV_FONT_MONTSERRAT_14=y   (mode, odometer, range, chg, temp)
 *  CONFIG_LV_FONT_MONTSERRAT_12=y   (aux/warning — fallback to 14 if absent)
 *
 ****************************************************************************/

#ifndef __APPS_HMI_MANAGER_UIUX_WIDGETS_H
#define __APPS_HMI_MANAGER_UIUX_WIDGETS_H

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

#define CLR_WHITE       lv_color_hex(0xFFFFFF)
#define CLR_AMBER       lv_color_hex(0xFFA500)
#define CLR_AMBER_HOT   lv_color_hex(0xFF6600)
#define CLR_AMBER_DIM   lv_color_hex(0x3A2000)
#define CLR_RED         lv_color_hex(0xFF2000)
#define CLR_GREEN_MODE  lv_color_hex(0x00DD00)
#define CLR_PINK        lv_color_hex(0xFF9EAA)
#define CLR_PURPLE      lv_color_hex(0xBB00CC)
#define CLR_ORANGE      lv_color_hex(0xFF6E00)
#define CLR_DARK_BG     lv_color_hex(0x0A0A0A)

/****************************************************************************
 * Layout — Gauge Circles
 * Source: SVG circles path21083 / path21083-7 / path21083-1 / path21083-1-2
 ****************************************************************************/

#define SPD_CX            110   /* speedometer pivot x                      */
#define SPD_CY            116   /* speedometer pivot y                      */
#define SPD_R_OUTER        79   /* outer ring radius (px)                   */
#define SPD_R_INNER        43   /* inner ring radius (px)                   */
#define SPD_NEEDLE_LEN     70   /* speed  needle length (white, long)       */
#define RPM_NEEDLE_LEN     28   /* rpm    needle length (amber, half speed) */
#define SPD_ANGLE_START   210   /* deg at 0 km/h  (7 o'clock)              */
#define SPD_ANGLE_SWEEP   300   /* total sweep 7h->5h clockwise             */
#define SPD_MAX_KMH       160   /* full-scale km/h                          */
#define RPM_MAX         10000   /* full-scale RPM                           */

#define REG_CX            375   /* regen gauge pivot x                      */
#define REG_CY            116   /* regen gauge pivot y                      */
#define REG_R_OUTER        79   /* outer ring radius (px)                   */
#define REG_R_INNER        43   /* inner ring radius (px)                   */
#define REG_NEEDLE_LEN     70   /* current needle length (amber)            */
#define REG_ANGLE_START   210   /* deg at 0 A                               */
#define REG_ANGLE_SWEEP   300   /* total sweep 7h->5h clockwise             */
#define REG_MAX_AMPS      500   /* full-scale (matches gauge face label)    */

/****************************************************************************
 * Layout — Placeholder Rectangles
 * Source: SVG rect elements, converted at 3.811 px/mm
 *
 * Naming: <widget>_X/Y = top-left corner, _W/_H = width/height (px)
 ****************************************************************************/

/* GEAR indicator — cyan rect inside hex frame */
#define GEAR_X            226
#define GEAR_Y             81
#define GEAR_W             28
#define GEAR_H             30
#define GEAR_CX           240   /* = GEAR_X + GEAR_W/2 */
#define GEAR_CY            96   /* = GEAR_Y + GEAR_H/2 */

/* MODE display — green rect below hex */
#define MODE_X            202
#define MODE_Y            135
#define MODE_W             81
#define MODE_H             14

/* BATTERY % bar — red rect, bottom-left */
#define BAT_X              39
#define BAT_Y             233
#define BAT_W             100
#define BAT_H              14

/* ODOMETER — yellow rect, bottom-left */
#define ODO_X              71
#define ODO_Y             268
#define ODO_W              90
#define ODO_H              13

/* EXTERNAL TEMPERATURE — pink rect, bottom-right top */
#define EXT_TEMP_X        343
#define EXT_TEMP_Y        234
#define EXT_TEMP_W        102
#define EXT_TEMP_H         11

/* RANGE PREDICTION — purple rect, bottom-right mid */
#define RANGE_X           381
#define RANGE_Y           259
#define RANGE_W            63
#define RANGE_H             9

/* CHARGE TIME — orange rect, bottom-right bottom */
#define CHG_X             382
#define CHG_Y             278
#define CHG_W              63
#define CHG_H              10

/* AUX / WARNING display — cream rect, bottom-centre */
#define AUX_X             195
#define AUX_Y             286
#define AUX_W              91
#define AUX_H              27

/****************************************************************************
 * Battery voltage scale (adjust for your pack chemistry)
 ****************************************************************************/

#define BAT_VOLTAGE_MAX_DV   960   /* 96.0 V = 100 % */
#define BAT_VOLTAGE_MIN_DV   600   /* 60.0 V =   0 % */
#define BAT_WARN_PCT          26   /* bar → amber (100..26%)  */
#define BAT_WARN_LOW_PCT      11   /* bar → orange (25..11%)  */
#define BAT_CRIT_PCT           1   /* bar → red   (10..1%)    */
                                   /* 0% → bar empty/hidden   */

/****************************************************************************
 * Public API
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

int  widgets_init(void);

/* Gauge needles */
void widgets_set_speed(int kmh);      /* white needle,  speedo             */
void widgets_set_rpm(int rpm);        /* amber needle,  speedo (co-axial)  */
void widgets_set_current(int amps);   /* amber needle,  regen gauge        */
void widgets_set_kw(int watts);        /* white needle,  regen gauge (kW)   */

/* Text / bar widgets */
void widgets_set_direction(int dir);         /* GEAR: N/D/R/P              */
void widgets_set_mode(int mode);             /* MODE: STANDARD/SPORT/ECO   */
void widgets_set_battery_pct(int pct);       /* BAT bar 0-100 %            */
void widgets_set_voltage(int dv);            /* dV → % → battery bar       */
void widgets_set_odometer(uint32_t km);      /* "NNNNNN km"                */
void widgets_set_ext_temp(int temp_c);       /* "NN°C"                     */
void widgets_set_range(int km);              /* "NNN km"                   */
void widgets_set_chg_time(int minutes);      /* "NhNNm"                    */
void widgets_set_warning(const char *msg);   /* aux/warning text           */

#ifdef __cplusplus
}
#endif

#endif /* __APPS_HMI_MANAGER_UIUX_WIDGETS_H */
