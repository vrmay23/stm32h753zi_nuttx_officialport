/****************************************************************************
 * apps/hmi_manager/uiux/widgets.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Dashboard Widget Implementation - Retro T3 Theme
 * LVGL 9.2  |  ST7796 480x320 RGB565
 *
 * Coordinates source: lvgl_placeholder_retro_t3.svg (Inkscape)
 *
 * ── Widget list ──────────────────────────────────────────────────────────
 *
 *  Needle gauges (lv_line over static background image):
 *    1. Speed   needle — white, len=70, pivot cx=110 cy=116
 *    2. RPM     needle — amber, len=55, pivot cx=110 cy=116 (co-axial)
 *    3. Current needle — amber, len=70, pivot cx=375 cy=116
 *    4. Pivot dots     — filled circles at both gauge centres
 *
 *  Text / bar widgets:
 *    5.  GEAR    lv_label  226, 81,  28×30   white large letter N/D/R/P
 *    6.  MODE    lv_label  202,135,  81×14   coloured text STANDARD/SPORT/ECO
 *    7.  BAT     lv_bar     39,233, 100×14   colour-coded fill 0-100 %
 *    8.  ODO     lv_label   71,268,  90×13   amber  "NNNNNN km"
 *    9.  TEMP    lv_label  343,234, 102×11   pink   "NN°C"
 *   10.  RANGE   lv_label  381,259,  63× 9   purple "NNN km"
 *   11.  CHG     lv_label  382,278,  63×10   orange "NhNNm"
 *   12.  AUX     lv_label  195,286,  91×27   white  warning / info text
 *
 ****************************************************************************/

#include "widgets.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>   /* sinf, cosf — link with -lm */

/****************************************************************************
 * Font aliases  (swap here for custom 7-segment font if desired)
 ****************************************************************************/

#define FONT_GEAR    (&lv_font_montserrat_28)   /* N D R P                 */
#define FONT_STD     (&lv_font_montserrat_14)   /* all small text labels   */

/****************************************************************************
 * Needle geometry
 ****************************************************************************/

#define LV_PI_F      3.14159265f
#define NEEDLE_W     2     /* line width in pixels  */
#define PIVOT_R      4     /* pivot dot radius      */

/****************************************************************************
 * Private widget handles
 ****************************************************************************/

/* Needle lines */
static lv_obj_t *g_spd_line   = NULL;
static lv_obj_t *g_rpm_line   = NULL;
static lv_obj_t *g_reg_line   = NULL;

/* Pivot dots */
static lv_obj_t *g_spd_pivot  = NULL;
static lv_obj_t *g_reg_pivot  = NULL;

/* Needle point arrays (pivot + tip, reused on every update) */
static lv_point_precise_t g_spd_pts[2];
static lv_point_precise_t g_rpm_pts[2];
static lv_point_precise_t g_reg_pts[2];

/* KW gauge needle (white, regen circle — separate from current) */
static lv_obj_t           *g_kw_line  = NULL;
static lv_point_precise_t  g_kw_pts[2];

/* Text / bar widgets */
static lv_obj_t *g_gear_lbl   = NULL;
static lv_obj_t *g_mode_lbl   = NULL;
static lv_obj_t *g_bat_bar    = NULL;
static lv_obj_t *g_odo_lbl    = NULL;
static lv_obj_t *g_temp_lbl   = NULL;
static lv_obj_t *g_range_lbl  = NULL;
static lv_obj_t *g_chg_lbl    = NULL;
static lv_obj_t *g_aux_lbl    = NULL;

/****************************************************************************
 * Private helpers
 ****************************************************************************/

static void transp(lv_obj_t *o)
{
  lv_obj_set_style_bg_opa(o,        LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(o,  0,             LV_PART_MAIN);
  lv_obj_set_style_outline_width(o, 0,             LV_PART_MAIN);
  lv_obj_set_style_pad_all(o,       0,             LV_PART_MAIN);
}

/* Create a label filling a placeholder rectangle */
static lv_obj_t *mklabel(lv_coord_t x, lv_coord_t y,
                           lv_coord_t w, lv_coord_t h,
                           const lv_font_t *font,
                           lv_color_t color,
                           const char *text)
{
  lv_obj_t *o = lv_label_create(lv_scr_act());
  if (!o) return NULL;
  transp(o);
  lv_obj_set_style_text_font(o,  font,                   LV_PART_MAIN);
  lv_obj_set_style_text_color(o, color,                  LV_PART_MAIN);
  lv_obj_set_style_text_align(o, LV_TEXT_ALIGN_CENTER,   LV_PART_MAIN);
  lv_obj_set_size(o, w, h);
  lv_obj_set_pos(o, x, y);
  lv_label_set_text(o, text);
  return o;
}

/****************************************************************************
 * Name: needle_update
 *
 * Description:
 *   Recompute the tip of a needle given a value, then refresh the lv_line.
 *
 *   angle_deg = start_deg + (value/max) * sweep_deg
 *   tip_x     = cx + len * sin(angle_rad)
 *   tip_y     = cy - len * cos(angle_rad)   (screen Y is inverted)
 *
 ****************************************************************************/

static void needle_update(lv_obj_t           *line,
                           lv_point_precise_t *pts,
                           lv_coord_t cx, lv_coord_t cy,
                           int value,  int max_val,
                           int len,    int start_deg, int sweep_deg)
{
  float ratio = (float)value / (float)max_val;
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;

  float deg = (float)start_deg + ratio * (float)sweep_deg;
  float rad = deg * LV_PI_F / 180.0f;

  pts[0].x = (lv_value_precise_t)cx;
  pts[0].y = (lv_value_precise_t)cy;
  pts[1].x = (lv_value_precise_t)(cx + (int)(len * sinf(rad)));
  pts[1].y = (lv_value_precise_t)(cy - (int)(len * cosf(rad)));

  lv_line_set_points(line, pts, 2);
}

/* Create a needle lv_line at value=0 */
static lv_obj_t *mkneedle(lv_point_precise_t *pts,
                            lv_coord_t cx, lv_coord_t cy,
                            int len, int start_deg, int sweep_deg,
                            int max_val,
                            lv_color_t color)
{
  lv_obj_t *line = lv_line_create(lv_scr_act());
  if (!line) return NULL;
  transp(line);
  lv_obj_set_style_line_color(line,    color,  LV_PART_MAIN);
  lv_obj_set_style_line_width(line,    NEEDLE_W, LV_PART_MAIN);
  lv_obj_set_style_line_rounded(line,  true,   LV_PART_MAIN);
  needle_update(line, pts, cx, cy, 0, max_val, len, start_deg, sweep_deg);
  return line;
}

/* Small filled circle at needle pivot */
static lv_obj_t *mkpivot(lv_coord_t cx, lv_coord_t cy, lv_color_t color)
{
  lv_obj_t *o = lv_obj_create(lv_scr_act());
  if (!o) return NULL;
  lv_obj_set_size(o, PIVOT_R * 2, PIVOT_R * 2);
  lv_obj_set_pos(o, cx - PIVOT_R, cy - PIVOT_R);
  lv_obj_set_style_radius(o,        LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(o,      color,            LV_PART_MAIN);
  lv_obj_set_style_bg_opa(o,        LV_OPA_COVER,     LV_PART_MAIN);
  lv_obj_set_style_border_width(o,  0,                LV_PART_MAIN);
  lv_obj_set_style_outline_width(o, 0,                LV_PART_MAIN);
  lv_obj_set_style_pad_all(o,       0,                LV_PART_MAIN);
  return o;
}

/****************************************************************************
 * widgets_init
 ****************************************************************************/

int widgets_init(void)
{
  /* ── 1. Speed needle (white, long) — speedometer ───────────────────── */

  g_spd_line = mkneedle(g_spd_pts,
                         SPD_CX, SPD_CY,
                         SPD_NEEDLE_LEN,
                         SPD_ANGLE_START, SPD_ANGLE_SWEEP,
                         SPD_MAX_KMH, CLR_WHITE);
  if (!g_spd_line) return -1;

  /* ── 2. RPM needle (amber, shorter) — same pivot as speed ─────────── */

  g_rpm_line = mkneedle(g_rpm_pts,
                         SPD_CX, SPD_CY,
                         RPM_NEEDLE_LEN,
                         SPD_ANGLE_START, SPD_ANGLE_SWEEP,
                         RPM_MAX, CLR_AMBER);
  if (!g_rpm_line) return -1;

  /* ── 3. Speedometer pivot dot ──────────────────────────────────────── */

  g_spd_pivot = mkpivot(SPD_CX, SPD_CY, CLR_WHITE);
  if (!g_spd_pivot) return -1;

  /* ── 4. Current needle (amber) — regen gauge ───────────────────────── */

  g_reg_line = mkneedle(g_reg_pts,
                         REG_CX, REG_CY,
                         REG_NEEDLE_LEN,
                         REG_ANGLE_START, REG_ANGLE_SWEEP,
                         REG_MAX_AMPS, CLR_AMBER);
  if (!g_reg_line) return -1;

  /* ── 5. Regen gauge pivot dot ──────────────────────────────────────── */

  g_reg_pivot = mkpivot(REG_CX, REG_CY, CLR_AMBER);
  if (!g_reg_pivot) return -1;

  /* ── 5b. KW needle (white, shorter) — regen gauge ────────────────── *
   *  Separate from current needle. Shows electrical power 0..max kW.   *
   *  White colour distinguishes it from the amber current needle.       *
   *  Length = 50px (shorter than current 70px — inner ring reference). */

  g_kw_line = mkneedle(g_kw_pts,
                        REG_CX, REG_CY,
                        50,
                        REG_ANGLE_START, REG_ANGLE_SWEEP,
                        1000,          /* max 100.0 kW (stored as 10x) */
                        CLR_WHITE);
  if (!g_kw_line) return -1;

  /* ── 6. Gear label — cyan rect 226,81 28×30 ────────────────────────── *
   *  Large single uppercase letter.  FONT_GEAR (28px) cap-height ≈ 22px, *
   *  fits inside 30px rect height with minimal padding.                  */

  g_gear_lbl = mklabel(GEAR_X, GEAR_Y, GEAR_W, GEAR_H,
                        FONT_GEAR, CLR_WHITE, "N");
  if (!g_gear_lbl) return -1;

  /* ── 7. Mode label — green rect 202,135 81×14 ──────────────────────── */

  g_mode_lbl = mklabel(MODE_X, MODE_Y, MODE_W, MODE_H,
                        FONT_STD, CLR_GREEN_MODE, "STANDARD");
  if (!g_mode_lbl) return -1;

  /* ── 8. Battery bar — red rect 39,233 100×14 ───────────────────────── *
   *  lv_bar: dark track + colour-coded fill.                             *
   *  The red border rectangle is already drawn in the background image.  */

  g_bat_bar = lv_bar_create(lv_scr_act());
  if (!g_bat_bar) return -1;

  lv_obj_set_size(g_bat_bar, BAT_W, BAT_H);
  lv_obj_set_pos(g_bat_bar,  BAT_X, BAT_Y);
  lv_bar_set_range(g_bat_bar, 0, 100);
  lv_bar_set_value(g_bat_bar, 0, LV_ANIM_OFF);

  lv_obj_set_style_bg_color(g_bat_bar,     CLR_DARK_BG,  LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_bat_bar,       LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_bat_bar, 0,            LV_PART_MAIN);
  lv_obj_set_style_radius(g_bat_bar,       2,            LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_bat_bar,      1,            LV_PART_MAIN);

  lv_obj_set_style_bg_color(g_bat_bar, CLR_AMBER,    LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(g_bat_bar,   LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_radius(g_bat_bar,   1,            LV_PART_INDICATOR);

  /* ── 9. Odometer — yellow rect 71,268 90×13 ────────────────────────── */

  g_odo_lbl = mklabel(ODO_X, ODO_Y, ODO_W, ODO_H,
                       FONT_STD, CLR_AMBER, "000000 km");
  if (!g_odo_lbl) return -1;

  /* ── 10. External temperature — pink rect 343,234 102×11 ───────────── */

  g_temp_lbl = mklabel(EXT_TEMP_X, EXT_TEMP_Y, EXT_TEMP_W, EXT_TEMP_H,
                        FONT_STD, CLR_PINK, "--\xc2\xb0""C");
  if (!g_temp_lbl) return -1;

  /* ── 11. Range prediction — purple rect 381,259 63×9 ───────────────── */

  g_range_lbl = mklabel(RANGE_X, RANGE_Y, RANGE_W, RANGE_H,
                         FONT_STD, CLR_PURPLE, "--- km");
  if (!g_range_lbl) return -1;

  /* ── 12. Charge time — orange rect 382,278 63×10 ───────────────────── */

  g_chg_lbl = mklabel(CHG_X, CHG_Y, CHG_W, CHG_H,
                       FONT_STD, CLR_ORANGE, "--h--m");
  if (!g_chg_lbl) return -1;

  /* ── 13. Aux / Warning — cream rect 195,286 91×27 ──────────────────── *
   *  General-purpose text area: warnings, status messages.               *
   *  Pass "" to clear.                                                   */

  g_aux_lbl = mklabel(AUX_X, AUX_Y, AUX_W, AUX_H,
                       FONT_STD, CLR_WHITE, "");
  if (!g_aux_lbl) return -1;

  return 0;
}

/****************************************************************************
 * Needle update functions
 ****************************************************************************/

void widgets_set_speed(int kmh)
{
  if (!g_spd_line) return;
  if (kmh < 0)           kmh = 0;
  if (kmh > SPD_MAX_KMH) kmh = SPD_MAX_KMH;
  needle_update(g_spd_line, g_spd_pts,
                SPD_CX, SPD_CY, kmh, SPD_MAX_KMH,
                SPD_NEEDLE_LEN, SPD_ANGLE_START, SPD_ANGLE_SWEEP);
}

void widgets_set_rpm(int rpm)
{
  if (!g_rpm_line) return;
  if (rpm < 0)       rpm = 0;
  if (rpm > RPM_MAX) rpm = RPM_MAX;
  needle_update(g_rpm_line, g_rpm_pts,
                SPD_CX, SPD_CY, rpm, RPM_MAX,
                RPM_NEEDLE_LEN, SPD_ANGLE_START, SPD_ANGLE_SWEEP);
}

/* Cached voltage for kW = V * I computation */
static int g_cached_voltage_dv = 0;

void widgets_set_current(int amps)
{
  int watts;

  if (!g_reg_line) return;

  /* Current needle — shows raw amperes */
  int display_amps = amps < 0 ? -amps : amps;
  if (display_amps > REG_MAX_AMPS) display_amps = REG_MAX_AMPS;
  needle_update(g_reg_line, g_reg_pts,
                REG_CX, REG_CY, display_amps, REG_MAX_AMPS,
                REG_NEEDLE_LEN, REG_ANGLE_START, REG_ANGLE_SWEEP);

  /* KW needle — computed from V * I
   * g_cached_voltage_dv is in decivolts (e.g. 840 = 84.0V)
   * amps is in raw amperes
   * watts = (dv/10) * amps = dv * amps / 10                    */
  watts = (g_cached_voltage_dv * amps) / 10;
  widgets_set_kw(watts);
}

/****************************************************************************
 * Text widget update functions
 ****************************************************************************/

void widgets_set_direction(int dir)
{
  if (!g_gear_lbl) return;
  switch (dir)
    {
      case WIDGET_DIR_FORWARD:
        lv_label_set_text(g_gear_lbl, "D");
        lv_obj_set_style_text_color(g_gear_lbl, CLR_WHITE,    LV_PART_MAIN);
        break;
      case WIDGET_DIR_REVERSE:
        lv_label_set_text(g_gear_lbl, "R");
        lv_obj_set_style_text_color(g_gear_lbl, CLR_AMBER_HOT, LV_PART_MAIN);
        break;
      case WIDGET_DIR_NEUTRAL:
      default:
        lv_label_set_text(g_gear_lbl, "N");
        lv_obj_set_style_text_color(g_gear_lbl, CLR_WHITE,    LV_PART_MAIN);
        break;
    }
}

void widgets_set_mode(int mode)
{
  if (!g_mode_lbl) return;
  switch (mode)
    {
      case WIDGET_MODE_SPORT:
        lv_label_set_text(g_mode_lbl, "SPORT");
        lv_obj_set_style_text_color(g_mode_lbl, CLR_AMBER_HOT,        LV_PART_MAIN);
        break;
      case WIDGET_MODE_ECO:
        lv_label_set_text(g_mode_lbl, "ECO");
        lv_obj_set_style_text_color(g_mode_lbl, lv_color_hex(0x00FFAA), LV_PART_MAIN);
        break;
      case WIDGET_MODE_STANDARD:
      default:
        lv_label_set_text(g_mode_lbl, "STANDARD");
        lv_obj_set_style_text_color(g_mode_lbl, CLR_GREEN_MODE,        LV_PART_MAIN);
        break;
    }
}

void widgets_set_battery_pct(int pct)
{
  lv_color_t c;
  if (!g_bat_bar) return;
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;

  lv_bar_set_value(g_bat_bar, pct, LV_ANIM_OFF);

  if (pct == 0)
    {
      /* 0% — hide indicator completely */
      lv_obj_set_style_bg_opa(g_bat_bar, LV_OPA_TRANSP, LV_PART_INDICATOR);
      return;
    }

  /* Restore opacity in case it was hidden */
  lv_obj_set_style_bg_opa(g_bat_bar, LV_OPA_COVER, LV_PART_INDICATOR);

  if      (pct <= BAT_CRIT_PCT)    c = CLR_RED;           /* 1..10%  red    */
  else if (pct <= BAT_WARN_LOW_PCT) c = CLR_AMBER_HOT;    /* 11..25% orange */
  else if (pct <= BAT_WARN_PCT)    c = CLR_AMBER_HOT;     /* 26% boundary   */
  else                              c = lv_color_hex(0x00CC44); /* 27..100% green */

  lv_obj_set_style_bg_color(g_bat_bar, c, LV_PART_INDICATOR);
}

void widgets_set_voltage(int dv)
{
  int range = BAT_VOLTAGE_MAX_DV - BAT_VOLTAGE_MIN_DV;
  int pct;

  /* Cache for kW = V * I computation in widgets_set_current() */
  g_cached_voltage_dv = dv;

  if (dv < BAT_VOLTAGE_MIN_DV) dv = BAT_VOLTAGE_MIN_DV;
  if (dv > BAT_VOLTAGE_MAX_DV) dv = BAT_VOLTAGE_MAX_DV;
  pct = ((dv - BAT_VOLTAGE_MIN_DV) * 100) / range;
  widgets_set_battery_pct(pct);
}

void widgets_set_odometer(uint32_t km)
{
  char buf[16];
  if (!g_odo_lbl) return;
  if (km > 999999u) km = 999999u;
  snprintf(buf, sizeof(buf), "%06lu km", (unsigned long)km);
  lv_label_set_text(g_odo_lbl, buf);
}

void widgets_set_ext_temp(int temp_c)
{
  char buf[12];
  if (!g_temp_lbl) return;
  snprintf(buf, sizeof(buf), "%d\xc2\xb0""C", temp_c);
  lv_label_set_text(g_temp_lbl, buf);
}

void widgets_set_range(int km)
{
  char buf[12];
  if (!g_range_lbl) return;
  if (km < 0) km = 0;
  snprintf(buf, sizeof(buf), "%d km", km);
  lv_label_set_text(g_range_lbl, buf);
}

void widgets_set_chg_time(int minutes)
{
  char buf[12];
  if (!g_chg_lbl) return;
  if      (minutes <= 0)  lv_label_set_text(g_chg_lbl, "Full");
  else if (minutes >= 60)
    {
      snprintf(buf, sizeof(buf), "%dh%02dm", minutes/60, minutes%60);
      lv_label_set_text(g_chg_lbl, buf);
    }
  else
    {
      snprintf(buf, sizeof(buf), "%d min", minutes);
      lv_label_set_text(g_chg_lbl, buf);
    }
}

void widgets_set_warning(const char *msg)
{
  if (!g_aux_lbl || !msg) return;
  lv_label_set_text(g_aux_lbl, msg);
}

/****************************************************************************
 * Name: widgets_set_kw
 *
 * Description:
 *   Update the KW needle on the regen gauge.
 *   @watts: instantaneous power in watts (0..100000 = 0..100 kW)
 *           Negative (regen) mapped to positive for display — needle
 *           shows magnitude only; colour indicates direction.
 *           Positive = white (motoring), Negative = amber (regenerating).
 *
 ****************************************************************************/

void widgets_set_kw(int watts)
{
  int abs_w = watts < 0 ? -watts : watts;

  if (!g_kw_line) return;

  /* Clamp to 100 kW max */
  if (abs_w > 100000) abs_w = 100000;

  /* Store as 10x so max=1000 matches mkneedle(max_val=1000) */
  int val10 = abs_w / 100;   /* 0..1000 */

  needle_update(g_kw_line, g_kw_pts,
                REG_CX, REG_CY, val10, 1000,
                50, REG_ANGLE_START, REG_ANGLE_SWEEP);

  /* Colour: white = motoring, amber = regenerating */
  lv_obj_set_style_line_color(g_kw_line,
                               watts >= 0 ? CLR_WHITE : CLR_AMBER,
                               LV_PART_MAIN);
}
