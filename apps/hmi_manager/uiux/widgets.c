/****************************************************************************
 * apps/hmi_manager/uiux/widgets.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <lvgl/lvgl.h>

#include "widgets.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static lv_obj_t *g_speed_label = NULL;
static lv_obj_t *g_voltage_label = NULL;
static lv_obj_t *g_current_label = NULL;
static lv_obj_t *g_direction_label = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: widgets_init
 *
 * Description:
 *   Create LVGL UI widgets.
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int widgets_init(void)
{
  lv_obj_t *screen;
  lv_obj_t *container;

  if (g_speed_label != NULL)
    {
      return -EBUSY;
    }

  /* Get active screen */

  screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);

  /* === Speed (top center) === */

  container = lv_obj_create(screen);
  lv_obj_set_size(container, 200, 100);
  lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 20);
  lv_obj_set_style_bg_color(container, lv_color_hex(0x1a1a1a), 0);
  lv_obj_set_style_border_color(container, lv_color_hex(0x00ff00), 0);
  lv_obj_set_style_border_width(container, 2, 0);

  g_speed_label = lv_label_create(container);
  lv_label_set_text(g_speed_label, "0 km/h");
  lv_obj_set_style_text_color(g_speed_label, lv_color_hex(0x00ff00), 0);
  lv_obj_set_style_text_font(g_speed_label, &lv_font_montserrat_32, 0);
  lv_obj_center(g_speed_label);

  /* === Voltage (left) === */

  container = lv_obj_create(screen);
  lv_obj_set_size(container, 140, 60);
  lv_obj_align(container, LV_ALIGN_LEFT_MID, 20, -30);
  lv_obj_set_style_bg_color(container, lv_color_hex(0x1a1a1a), 0);
  lv_obj_set_style_border_color(container, lv_color_hex(0xffaa00), 0);
  lv_obj_set_style_border_width(container, 2, 0);

  g_voltage_label = lv_label_create(container);
  lv_label_set_text(g_voltage_label, "0.0 V");
  lv_obj_set_style_text_color(g_voltage_label,
                               lv_color_hex(0xffaa00), 0);
  lv_obj_set_style_text_font(g_voltage_label,
                              &lv_font_montserrat_20, 0);
  lv_obj_center(g_voltage_label);

  /* === Current (right) === */

  container = lv_obj_create(screen);
  lv_obj_set_size(container, 140, 60);
  lv_obj_align(container, LV_ALIGN_RIGHT_MID, -20, -30);
  lv_obj_set_style_bg_color(container, lv_color_hex(0x1a1a1a), 0);
  lv_obj_set_style_border_color(container, lv_color_hex(0x00aaff), 0);
  lv_obj_set_style_border_width(container, 2, 0);

  g_current_label = lv_label_create(container);
  lv_label_set_text(g_current_label, "0.0 A");
  lv_obj_set_style_text_color(g_current_label,
                               lv_color_hex(0x00aaff), 0);
  lv_obj_set_style_text_font(g_current_label,
                              &lv_font_montserrat_20, 0);
  lv_obj_center(g_current_label);

  /* === Direction (bottom center) === */

  container = lv_obj_create(screen);
  lv_obj_set_size(container, 200, 60);
  lv_obj_align(container, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_set_style_bg_color(container, lv_color_hex(0x1a1a1a), 0);
  lv_obj_set_style_border_color(container, lv_color_hex(0xffffff), 0);
  lv_obj_set_style_border_width(container, 2, 0);

  g_direction_label = lv_label_create(container);
  lv_label_set_text(g_direction_label, "NEUTRAL");
  lv_obj_set_style_text_color(g_direction_label,
                               lv_color_hex(0xffffff), 0);
  lv_obj_set_style_text_font(g_direction_label,
                              &lv_font_montserrat_24, 0);
  lv_obj_center(g_direction_label);

  printf("Widgets: UI created\n");
  return 0;
}

/****************************************************************************
 * Name: widgets_set_speed
 *
 * Description:
 *   Update speed display.
 *
 * Input Parameters:
 *   speed_kmh - Speed in km/h
 ****************************************************************************/

void widgets_set_speed(int speed_kmh)
{
  char buf[32];

  if (g_speed_label == NULL)
    {
      return;
    }

  snprintf(buf, sizeof(buf), "%d km/h", speed_kmh);
  lv_label_set_text(g_speed_label, buf);
}

/****************************************************************************
 * Name: widgets_set_voltage
 *
 * Description:
 *   Update voltage display.
 *
 * Input Parameters:
 *   voltage_dv - Voltage in 0.1V units (e.g., 500 = 50.0V)
 ****************************************************************************/

void widgets_set_voltage(int voltage_dv)
{
  char buf[32];

  if (g_voltage_label == NULL)
    {
      return;
    }

  snprintf(buf, sizeof(buf), "%d.%d V", voltage_dv / 10,
           voltage_dv % 10);
  lv_label_set_text(g_voltage_label, buf);
}

/****************************************************************************
 * Name: widgets_set_current
 *
 * Description:
 *   Update current display.
 *
 * Input Parameters:
 *   current_da - Current in 0.1A units (e.g., 200 = 20.0A)
 ****************************************************************************/

void widgets_set_current(int current_da)
{
  char buf[32];

  if (g_current_label == NULL)
    {
      return;
    }

  snprintf(buf, sizeof(buf), "%d.%d A", current_da / 10,
           current_da % 10);
  lv_label_set_text(g_current_label, buf);
}

/****************************************************************************
 * Name: widgets_set_direction
 *
 * Description:
 *   Update direction display.
 *
 * Input Parameters:
 *   direction - WIDGET_DIR_FORWARD, WIDGET_DIR_REVERSE, or
 *               WIDGET_DIR_NEUTRAL
 ****************************************************************************/

void widgets_set_direction(int direction)
{
  if (g_direction_label == NULL)
    {
      return;
    }

  switch (direction)
    {
      case WIDGET_DIR_FORWARD:
        lv_label_set_text(g_direction_label, "FORWARD");
        lv_obj_set_style_text_color(g_direction_label,
                                     lv_color_hex(0x00FF00), 0);
        break;

      case WIDGET_DIR_REVERSE:
        lv_label_set_text(g_direction_label, "REVERSE");
        lv_obj_set_style_text_color(g_direction_label,
                                     lv_color_hex(0xFF0000), 0);
        break;

      case WIDGET_DIR_NEUTRAL:
      default:
        lv_label_set_text(g_direction_label, "NEUTRAL");
        lv_obj_set_style_text_color(g_direction_label,
                                     lv_color_hex(0xFFFFFF), 0);
        break;
    }
}
