/****************************************************************************
 * apps/hmi_manager/uiux/widgets.c
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
 * Dashboard Widget Implementation - Retro T3 v3
 * LVGL 9.2 | ST7796 480x320 RGB565
 *
 * Coordinates: cluster_final.svg
 *
 * Key design decisions:
 *
 * 1) Hub vs Arc centres
 *    The SVG hub ellipses and arc paths have DIFFERENT centres.
 *    Needles pivot from the HUB centre (visual anchor point).
 *    lv_arc widgets are positioned at the ARC path centre.
 *
 * 2) Label centering (container + LV_ALIGN_CENTER)
 *    LVGL 9.x labels only centre text horizontally with
 *    LV_TEXT_ALIGN_CENTER.  Vertical alignment is NOT automatic.
 *    To centre both H and V, every label is a child of an
 *    invisible container, aligned with LV_ALIGN_CENTER.
 *    After each lv_label_set_text(), lv_obj_align() must be
 *    called again because text changes invalidate layout.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "widgets.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FONT_GEAR   (&lv_font_montserrat_28)
#define FONT_HUB    (&lv_font_montserrat_22)
#define FONT_HUB_SM (&lv_font_montserrat_14)
#define FONT_STD    (&lv_font_montserrat_14)
#define FONT_SMALL  (&lv_font_montserrat_12)
#define FONT_CLK    (&lv_font_montserrat_22)

#define ARC_W_OUTER 10
#define ARC_W_INNER  7

/* Hub numeric label bounding boxes */

#define HUB_NUM_W    60
#define HUB_NUM_H    26
#define HUB_SUB_W    60
#define HUB_SUB_H    18

/****************************************************************************
 * Private Data
 ****************************************************************************/

static lv_obj_t *g_spd_arc  = NULL;
static lv_obj_t *g_rpm_arc  = NULL;
static lv_obj_t *g_volt_arc = NULL;
static lv_obj_t *g_amp_arc  = NULL;

/* Hub numeric labels (replace needles + pivots) */

static lv_obj_t *g_spd_num_lbl  = NULL;
static lv_obj_t *g_rpm_num_lbl  = NULL;
static lv_obj_t *g_volt_num_lbl = NULL;
static lv_obj_t *g_amp_num_lbl  = NULL;

static lv_obj_t   *g_turn_l      = NULL;
static lv_obj_t   *g_turn_r      = NULL;
static lv_timer_t *g_blink_tm    = NULL;
static bool        g_turn_l_on   = false;
static bool        g_turn_r_on   = false;
static bool        g_blink_state = false;

static lv_obj_t *g_gear_lbl = NULL;
static lv_obj_t *g_mode_lbl = NULL;
static lv_obj_t *g_rem_lbl  = NULL;
static lv_obj_t *g_btmp_lbl = NULL;
static lv_obj_t *g_odo_lbl  = NULL;
static lv_obj_t *g_soc_lbl  = NULL;
static lv_obj_t *g_temp_lbl = NULL;
static lv_obj_t *g_clk_lbl  = NULL;
static lv_obj_t *g_aux_lbl  = NULL;
static lv_obj_t *g_bat_bar  = NULL;

static int g_cached_voltage_dv = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mklabel
 *
 * Description:
 *   Create a label centred H+V inside a bounding box at (x,y,w,h).
 *   Returns the lv_label (child), not the container (parent).
 *
 ****************************************************************************/

static lv_obj_t *mklabel(lv_coord_t x,
                          lv_coord_t y,
                          lv_coord_t w,
                          lv_coord_t h,
                          const lv_font_t *font,
                          lv_color_t color,
                          const char *text)
{
  lv_obj_t *box;
  lv_obj_t *lbl;

  box = lv_obj_create(lv_scr_act());
  if (!box) return NULL;

  lv_obj_set_size(box, w, h);
  lv_obj_set_pos(box, x, y);
  lv_obj_set_style_bg_opa(box,
    LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(box,
    0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(box,
    0, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(box,
    LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(box,
    LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(box);
  if (!lbl) return NULL;

  lv_obj_set_style_text_font(lbl,
    font, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl,
    color, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl,
    LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_label_set_long_mode(lbl,
    LV_LABEL_LONG_CLIP);
  lv_obj_set_width(lbl, w);
  lv_label_set_text(lbl, text);
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

  return lbl;
}

/****************************************************************************
 * Name: mkarc
 ****************************************************************************/

static lv_obj_t *mkarc(lv_coord_t cx,
                        lv_coord_t cy,
                        lv_coord_t radius,
                        int arc_w,
                        int sa, int ea,
                        lv_color_t color,
                        lv_opa_t opa)
{
  int sz = radius * 2;
  lv_obj_t *a = lv_arc_create(lv_scr_act());
  if (!a) return NULL;

  lv_obj_set_size(a, sz, sz);
  lv_obj_set_pos(a,
    cx - radius, cy - radius);

  lv_obj_set_style_pad_all(a,
    0, LV_PART_KNOB);
  lv_obj_set_style_bg_opa(a,
    LV_OPA_TRANSP, LV_PART_KNOB);

  lv_obj_set_style_arc_width(a,
    arc_w, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(a,
    LV_OPA_TRANSP, LV_PART_MAIN);

  lv_obj_set_style_arc_width(a,
    arc_w, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(a,
    color, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(a,
    opa, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(a,
    true, LV_PART_INDICATOR);

  lv_arc_set_bg_angles(a, sa, ea);
  lv_arc_set_rotation(a, 0);
  lv_arc_set_mode(a, LV_ARC_MODE_NORMAL);
  lv_arc_set_range(a, 0, 1000);
  lv_arc_set_value(a, 0);

  lv_obj_remove_flag(a,
    LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_bg_opa(a,
    LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(a,
    0, LV_PART_MAIN);

  return a;
}

/* needle_update / mkneedle / mkpivot removed — replaced by
 * hub numeric labels (g_spd_num_lbl, g_rpm_num_lbl, etc.)
 */

/****************************************************************************
 * Turn Signal helpers
 ****************************************************************************/

static lv_obj_t *mk_turn(lv_coord_t x,
                          lv_coord_t y,
                          lv_coord_t w,
                          lv_coord_t h,
                          bool left)
{
  lv_obj_t *box;
  lv_obj_t *lbl;

  box = lv_obj_create(lv_scr_act());
  if (!box) return NULL;

  lv_obj_set_size(box, w, h);
  lv_obj_set_pos(box, x, y);
  lv_obj_set_style_bg_opa(box,
    LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(box,
    0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(box,
    0, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(box,
    LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(box,
    LV_OBJ_FLAG_SCROLLABLE);

  lbl = lv_label_create(box);
  if (!lbl) return NULL;

  lv_obj_set_style_text_font(lbl,
    &lv_font_montserrat_22,
    LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl,
    CLR_GREEN_TURN, LV_PART_MAIN);
  lv_label_set_text(lbl,
    left ? LV_SYMBOL_LEFT
         : LV_SYMBOL_RIGHT);
  lv_obj_align(lbl,
    LV_ALIGN_CENTER, 0, 0);

  lv_obj_set_style_opa(box,
    LV_OPA_TRANSP, LV_PART_MAIN);

  return box;
}

static void blink_cb(lv_timer_t *t)
{
  (void)t;
  g_blink_state = !g_blink_state;

  if (g_turn_l && g_turn_l_on)
    {
      lv_obj_set_style_opa(g_turn_l,
        g_blink_state ? LV_OPA_COVER
                      : LV_OPA_TRANSP,
        LV_PART_MAIN);
    }

  if (g_turn_r && g_turn_r_on)
    {
      lv_obj_set_style_opa(g_turn_r,
        g_blink_state ? LV_OPA_COVER
                      : LV_OPA_TRANSP,
        LV_PART_MAIN);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int widgets_init(void)
{
  /* Left gauge arcs (at ARC centre) */

  g_spd_arc = mkarc(SPD_ARC_CX, SPD_ARC_CY,
    SPD_R_OUTER, ARC_W_OUTER,
    SPD_ARC_START, SPD_ARC_END,
    CLR_CYAN, ARC_FILL_OPA);
  if (!g_spd_arc) return -1;

  g_rpm_arc = mkarc(SPD_ARC_CX, SPD_ARC_CY,
    SPD_R_INNER, ARC_W_INNER,
    SPD_ARC_START, SPD_ARC_END,
    CLR_MAGENTA, ARC_FILL_OPA);
  if (!g_rpm_arc) return -1;

  /* Left hub numeric labels (speed + RPM) */

  g_spd_num_lbl = mklabel(
    SPD_HUB_CX - HUB_NUM_W / 2,
    SPD_HUB_CY - HUB_NUM_H - 1,
    HUB_NUM_W, HUB_NUM_H,
    FONT_HUB, CLR_CYAN, "0");
  if (!g_spd_num_lbl) return -1;

  g_rpm_num_lbl = mklabel(
    SPD_HUB_CX - HUB_SUB_W / 2,
    SPD_HUB_CY + 1,
    HUB_SUB_W, HUB_SUB_H,
    FONT_HUB_SM, CLR_MAGENTA, "0");
  if (!g_rpm_num_lbl) return -1;

  /* Right gauge arcs (at ARC centre) */

  g_volt_arc = mkarc(REG_ARC_CX, REG_ARC_CY,
    REG_R_OUTER, ARC_W_OUTER,
    REG_ARC_START, REG_ARC_END,
    CLR_CYAN, ARC_FILL_OPA);
  if (!g_volt_arc) return -1;

  g_amp_arc = mkarc(REG_ARC_CX, REG_ARC_CY,
    REG_R_INNER, ARC_W_INNER,
    REG_ARC_START, REG_ARC_END,
    CLR_RED_ARC, ARC_FILL_OPA);
  if (!g_amp_arc) return -1;

  /* Right hub numeric labels (voltage + current) */

  g_volt_num_lbl = mklabel(
    REG_HUB_CX - HUB_NUM_W / 2,
    REG_HUB_CY - HUB_NUM_H - 1,
    HUB_NUM_W, HUB_NUM_H,
    FONT_HUB, CLR_CYAN, "0V");
  if (!g_volt_num_lbl) return -1;

  g_amp_num_lbl = mklabel(
    REG_HUB_CX - HUB_SUB_W / 2,
    REG_HUB_CY + 1,
    HUB_SUB_W, HUB_SUB_H,
    FONT_HUB_SM, CLR_AMBER, "0A");
  if (!g_amp_num_lbl) return -1;

  /* Turn signals */

  g_turn_l = mk_turn(
    TURN_L_X, TURN_L_Y,
    TURN_L_W, TURN_L_H, true);
  g_turn_r = mk_turn(
    TURN_R_X, TURN_R_Y,
    TURN_R_W, TURN_R_H, false);
  g_blink_tm = lv_timer_create(
    blink_cb, TURN_BLINK_MS, NULL);

  /* Gear (centred in hexagon bbox) */

  g_gear_lbl = mklabel(
    GEAR_CX - GEAR_W / 2,
    GEAR_CY - GEAR_H / 2,
    GEAR_W, GEAR_H,
    FONT_GEAR, CLR_WHITE, "N");
  if (!g_gear_lbl) return -1;

  /* Mode */

  g_mode_lbl = mklabel(
    MODE_X, MODE_Y,
    MODE_W, MODE_H,
    FONT_STD, CLR_GREEN_MODE,
    "STANDARD");
  if (!g_mode_lbl) return -1;

  /* Battery bar */

  g_bat_bar = lv_bar_create(lv_scr_act());
  if (!g_bat_bar) return -1;

  lv_obj_set_size(g_bat_bar, BAT_W, BAT_H);
  lv_obj_set_pos(g_bat_bar, BAT_X, BAT_Y);
  lv_bar_set_range(g_bat_bar, 0, 100);
  lv_bar_set_value(g_bat_bar, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(g_bat_bar,
    CLR_DARK_BG, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_bat_bar,
    LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_bat_bar,
    0, LV_PART_MAIN);
  lv_obj_set_style_radius(g_bat_bar,
    2, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_bat_bar,
    1, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_bat_bar,
    CLR_GREEN, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(g_bat_bar,
    LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_radius(g_bat_bar,
    1, LV_PART_INDICATOR);

  /* Remaining time */

  g_rem_lbl = mklabel(
    REM_TIME_X, REM_TIME_Y,
    REM_TIME_W, REM_TIME_H,
    FONT_SMALL, CLR_CYAN, "--:--");
  if (!g_rem_lbl) return -1;

  /* Battery temp */

  g_btmp_lbl = mklabel(
    BATT_TEMP_X, BATT_TEMP_Y,
    BATT_TEMP_W, BATT_TEMP_H,
    FONT_SMALL, CLR_CYAN,
    "--\xc2\xb0""C");
  if (!g_btmp_lbl) return -1;

  /* Odometer */

  g_odo_lbl = mklabel(
    ODO_X, ODO_Y, ODO_W, ODO_H,
    FONT_SMALL, CLR_CYAN, "0");
  if (!g_odo_lbl) return -1;

  /* SoC */

  g_soc_lbl = mklabel(
    SOC_X, SOC_Y, SOC_W, SOC_H,
    FONT_SMALL, CLR_CYAN, "--%");
  if (!g_soc_lbl) return -1;

  /* Ext temp */

  g_temp_lbl = mklabel(
    EXT_TEMP_X, EXT_TEMP_Y,
    EXT_TEMP_W, EXT_TEMP_H,
    FONT_SMALL, CLR_CYAN,
    "--\xc2\xb0""C");
  if (!g_temp_lbl) return -1;

  /* Clock (static) */

  g_clk_lbl = mklabel(
    CLK_X, CLK_Y, CLK_W, CLK_H,
    FONT_CLK, CLR_CYAN, "12:00");
  if (!g_clk_lbl) return -1;

  /* Aux / warning */

  g_aux_lbl = mklabel(
    MODE_X, MODE_Y + MODE_H + 2,
    MODE_W, 14,
    FONT_SMALL, CLR_WHITE, "");
  if (!g_aux_lbl) return -1;

  return 0;
}

/****************************************************************************
 * Gauge updates
 ****************************************************************************/

void widgets_set_speed(int kmh)
{
  char buf[8];

  if (kmh < 0)           kmh = 0;
  if (kmh > SPD_MAX_KMH) kmh = SPD_MAX_KMH;

  if (g_spd_arc)
    lv_arc_set_value(g_spd_arc,
      (kmh * 1000) / SPD_MAX_KMH);

  if (g_spd_num_lbl)
    {
      snprintf(buf, sizeof(buf), "%d", kmh);
      lv_label_set_text(g_spd_num_lbl, buf);
      lv_obj_align(g_spd_num_lbl,
        LV_ALIGN_CENTER, 0, 0);
    }
}

void widgets_set_rpm(int rpm)
{
  char buf[8];

  if (rpm < 0)       rpm = 0;
  if (rpm > RPM_MAX) rpm = RPM_MAX;

  if (g_rpm_arc)
    lv_arc_set_value(g_rpm_arc,
      (rpm * 1000) / RPM_MAX);

  if (g_rpm_num_lbl)
    {
      snprintf(buf, sizeof(buf), "%d", rpm);
      lv_label_set_text(g_rpm_num_lbl, buf);
      lv_obj_align(g_rpm_num_lbl,
        LV_ALIGN_CENTER, 0, 0);
    }
}

void widgets_set_voltage_arc(int volts)
{
  if (volts < 0)             volts = 0;
  if (volts > REG_MAX_VOLTS) volts = REG_MAX_VOLTS;

  if (g_volt_arc)
    lv_arc_set_value(g_volt_arc,
      (volts * 1000) / REG_MAX_VOLTS);
}

void widgets_set_current(int amps)
{
  char buf[8];
  int da;
  int av;

  da = amps < 0 ? -amps : amps;
  if (da > REG_MAX_AMPS) da = REG_MAX_AMPS;

  av = (da * 1000) / REG_MAX_AMPS;

  if (g_amp_arc)
    {
      lv_arc_set_value(g_amp_arc, av);
      lv_obj_set_style_arc_color(g_amp_arc,
        amps < 0 ? CLR_RED_ARC
                 : CLR_MAGENTA,
        LV_PART_INDICATOR);
    }

  if (g_amp_num_lbl)
    {
      snprintf(buf, sizeof(buf), "%dA", da);
      lv_label_set_text(g_amp_num_lbl, buf);
      lv_obj_align(g_amp_num_lbl,
        LV_ALIGN_CENTER, 0, 0);
    }
}

/****************************************************************************
 * Turn signals
 ****************************************************************************/

void widgets_turn_left(bool on)
{
  g_turn_l_on = on;
  if (!on && g_turn_l)
    lv_obj_set_style_opa(g_turn_l,
      LV_OPA_TRANSP, LV_PART_MAIN);
}

void widgets_turn_right(bool on)
{
  g_turn_r_on = on;
  if (!on && g_turn_r)
    lv_obj_set_style_opa(g_turn_r,
      LV_OPA_TRANSP, LV_PART_MAIN);
}

void widgets_turn_hazard(bool on)
{
  widgets_turn_left(on);
  widgets_turn_right(on);
}

void widgets_turn_off(void)
{
  widgets_turn_left(false);
  widgets_turn_right(false);
}

/****************************************************************************
 * Text / bar updates
 *
 * Every set function re-calls lv_obj_align(LV_ALIGN_CENTER)
 * after changing text to keep the label centred.
 ****************************************************************************/

void widgets_set_direction(int dir)
{
  if (!g_gear_lbl) return;

  switch (dir)
    {
      case WIDGET_DIR_FORWARD:
        lv_label_set_text(g_gear_lbl, "D");
        lv_obj_set_style_text_color(
          g_gear_lbl,
          CLR_WHITE, LV_PART_MAIN);
        break;
      case WIDGET_DIR_REVERSE:
        lv_label_set_text(g_gear_lbl, "R");
        lv_obj_set_style_text_color(
          g_gear_lbl,
          CLR_AMBER_HOT, LV_PART_MAIN);
        break;
      default:
        lv_label_set_text(g_gear_lbl, "N");
        lv_obj_set_style_text_color(
          g_gear_lbl,
          CLR_WHITE, LV_PART_MAIN);
        break;
    }

  lv_obj_align(g_gear_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_mode(int mode)
{
  if (!g_mode_lbl) return;

  switch (mode)
    {
      case WIDGET_MODE_SPORT:
        lv_label_set_text(
          g_mode_lbl, "SPORT");
        lv_obj_set_style_text_color(
          g_mode_lbl,
          CLR_AMBER_HOT, LV_PART_MAIN);
        break;
      case WIDGET_MODE_ECO:
        lv_label_set_text(
          g_mode_lbl, "ECO");
        lv_obj_set_style_text_color(
          g_mode_lbl,
          lv_color_hex(0x00FFAA),
          LV_PART_MAIN);
        break;
      default:
        lv_label_set_text(
          g_mode_lbl, "STANDARD");
        lv_obj_set_style_text_color(
          g_mode_lbl,
          CLR_GREEN_MODE, LV_PART_MAIN);
        break;
    }

  lv_obj_align(g_mode_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_battery_pct(int pct)
{
  lv_color_t c;
  if (!g_bat_bar) return;
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;

  lv_bar_set_value(g_bat_bar,
    pct, LV_ANIM_ON);

  if (pct == 0)
    {
      lv_obj_set_style_bg_opa(g_bat_bar,
        LV_OPA_TRANSP,
        LV_PART_INDICATOR);
      return;
    }

  lv_obj_set_style_bg_opa(g_bat_bar,
    LV_OPA_COVER, LV_PART_INDICATOR);

  if      (pct <= BAT_CRIT_PCT)     c = CLR_RED;
  else if (pct <= BAT_WARN_LOW_PCT) c = CLR_AMBER_HOT;
  else if (pct <= BAT_WARN_PCT)     c = CLR_AMBER;
  else                               c = CLR_GREEN;

  lv_obj_set_style_bg_color(g_bat_bar,
    c, LV_PART_INDICATOR);
}

void widgets_set_voltage(int dv)
{
  char buf[8];
  int range;
  int pct;

  /* Skip zero — 0 dV is never a valid vehicle battery reading.
   * Avoids arc/bar/label jumping to min (60V) on stale
   * snapshots triggered by unrelated g_ui_dirty events.
   */

  if (dv == 0)
    return;

  g_cached_voltage_dv = dv;

  if (dv < BAT_VOLTAGE_MIN_DV)
    dv = BAT_VOLTAGE_MIN_DV;
  if (dv > BAT_VOLTAGE_MAX_DV)
    dv = BAT_VOLTAGE_MAX_DV;

  range = BAT_VOLTAGE_MAX_DV
        - BAT_VOLTAGE_MIN_DV;
  pct   = ((dv - BAT_VOLTAGE_MIN_DV) * 100)
        / range;

  widgets_set_battery_pct(pct);
  widgets_set_voltage_arc(dv / 10);

  if (g_volt_num_lbl)
    {
      snprintf(buf, sizeof(buf), "%d.%dV",
        dv / 10, dv % 10);
      lv_label_set_text(g_volt_num_lbl, buf);
      lv_obj_align(g_volt_num_lbl,
        LV_ALIGN_CENTER, 0, 0);
    }
}

void widgets_set_odometer(uint32_t km)
{
  char buf[12];
  if (!g_odo_lbl) return;
  if (km > 999999u) km = 999999u;
  snprintf(buf, sizeof(buf), "%lu",
    (unsigned long)km);
  lv_label_set_text(g_odo_lbl, buf);
  lv_obj_align(g_odo_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_ext_temp(int temp_c)
{
  char buf[8];
  if (!g_temp_lbl) return;
  snprintf(buf, sizeof(buf),
    "%d\xc2\xb0""C", temp_c);
  lv_label_set_text(g_temp_lbl, buf);
  lv_obj_align(g_temp_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_soc(int pct)
{
  char buf[8];
  if (!g_soc_lbl) return;
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  snprintf(buf, sizeof(buf), "%d%%", pct);
  lv_label_set_text(g_soc_lbl, buf);
  lv_obj_align(g_soc_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_remaining_time(int minutes)
{
  char buf[8];
  if (!g_rem_lbl) return;

  if (minutes <= 0)
    {
      lv_label_set_text(
        g_rem_lbl, "Full");
    }
  else if (minutes >= 60)
    {
      snprintf(buf, sizeof(buf),
        "%dh%02d", minutes / 60,
        minutes % 60);
      lv_label_set_text(g_rem_lbl, buf);
    }
  else
    {
      snprintf(buf, sizeof(buf),
        "%dmin", minutes);
      lv_label_set_text(g_rem_lbl, buf);
    }

  lv_obj_align(g_rem_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_batt_temp(int temp_c)
{
  char buf[8];
  if (!g_btmp_lbl) return;
  snprintf(buf, sizeof(buf),
    "%d\xc2\xb0""C", temp_c);
  lv_label_set_text(g_btmp_lbl, buf);
  lv_obj_align(g_btmp_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_clock(int hour, int minute)
{
  char buf[8];
  if (!g_clk_lbl) return;
  snprintf(buf, sizeof(buf),
    "%02d:%02d", hour % 24,
    minute % 60);
  lv_label_set_text(g_clk_lbl, buf);
  lv_obj_align(g_clk_lbl,
    LV_ALIGN_CENTER, 0, 0);
}

void widgets_set_warning(const char *msg)
{
  if (!g_aux_lbl || !msg) return;
  lv_label_set_text(g_aux_lbl, msg);
  lv_obj_align(g_aux_lbl,
    LV_ALIGN_CENTER, 0, 0);
}
