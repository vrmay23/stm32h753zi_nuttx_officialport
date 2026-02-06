/****************************************************************************
 * apps/hmi_manager/app_hmi_manager.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/select.h>
#include <nuttx/can.h>
#include <lvgl/lvgl.h>

#include "can/can_handler.h"
#include "can/car_can.h"
#include "can/can_trace.h"
#include "io_handler/button_irq.h"
#include "uiux/fb_handler.h"
#include "uiux/widgets.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LVGL_TICK_MS  10

/* Button to CAN mapping */

#define BTN0_ACTION_ID        CAR_CAN_DRIVER_COMMANDS_FRAME_ID
#define BTN0_ACTION_DIR       WIDGET_DIR_NEUTRAL

#define BTN1_ACTION_ID        CAR_CAN_DRIVER_COMMANDS_FRAME_ID
#define BTN1_ACTION_DIR       WIDGET_DIR_FORWARD

#define BTN2_ACTION_ID        CAR_CAN_DRIVER_COMMANDS_FRAME_ID
#define BTN2_ACTION_DIR       WIDGET_DIR_REVERSE

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_speed = 0;
static int g_voltage = 0;
static int g_current = 0;
static int g_direction = WIDGET_DIR_NEUTRAL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: process_button_press
 *
 * Description:
 *   Process button press and send CAN message.
 ****************************************************************************/

static void process_button_press(int button)
{
  struct car_can_driver_commands_t cmd;
  uint8_t data[CAR_CAN_DRIVER_COMMANDS_LENGTH];
  struct can_frame frame;
  int ret;

  car_can_driver_commands_init(&cmd);

  switch (button)
    {
      case 0:
        cmd.demanded_drive_direction = BTN0_ACTION_DIR;
        cmd.demanded_drive_state = 0;
        break;

      case 1:
        cmd.demanded_drive_direction = BTN1_ACTION_DIR;
        cmd.demanded_drive_state = 1;
        break;

      case 2:
        cmd.demanded_drive_direction = BTN2_ACTION_DIR;
        cmd.demanded_drive_state = 1;
        break;

      default:
        return;
    }

  ret = car_can_driver_commands_pack(data, &cmd, sizeof(data));
  if (ret < 0)
    {
      printf("ERROR: pack failed\n");
      return;
    }

  ret = can_handler_send(CAR_CAN_DRIVER_COMMANDS_FRAME_ID,
                         CAR_CAN_DRIVER_COMMANDS_IS_EXTENDED,
                         data, ret);
  if (ret < 0)
    {
      printf("ERROR: send failed\n");
      return;
    }

  /* Log TX frame for trace */

  memset(&frame, 0, sizeof(frame));
  frame.can_id = CAR_CAN_DRIVER_COMMANDS_FRAME_ID;
  if (CAR_CAN_DRIVER_COMMANDS_IS_EXTENDED)
    {
      frame.can_id |= CAN_EFF_FLAG;
    }

  frame.can_dlc = ret;
  memcpy(frame.data, data, ret);
  can_trace_tx(&frame);

  printf("TX: BTN%d -> DIR=%d\n", button, cmd.demanded_drive_direction);
}

/****************************************************************************
 * Name: process_can_frame
 *
 * Description:
 *   Process received CAN frame and update widgets.
 ****************************************************************************/

static void process_can_frame(struct can_frame *frame)
{
  uint32_t id;
  int ret;

  id = frame->can_id & CAN_EFF_MASK;

  /* Speed */

  if (id == CAR_CAN_INVERTER_DEMANDERS_FRAME_ID)
    {
      struct car_can_inverter_demanders_t msg;

      ret = car_can_inverter_demanders_unpack(&msg, frame->data,
                                               frame->can_dlc);
      if (ret == 0)
        {
          g_speed = (int)car_can_inverter_demanders_pedal_request_decode(
                            msg.pedal_request);
          widgets_set_speed(g_speed);
        }
    }

  /* Battery voltage/current */

  else if (id == CAR_CAN_SYSTEM_VOLTAGES_FRAME_ID)
    {
      struct car_can_system_voltages_t msg;

      ret = car_can_system_voltages_unpack(&msg, frame->data,
                                            frame->can_dlc);
      if (ret == 0)
        {
          g_voltage = (int)(car_can_system_voltages_meas_batt_voltage_decode(
                              msg.meas_batt_voltage) * 10.0);
          widgets_set_voltage(g_voltage);
        }
    }

  /* Direction */

  else if (id == CAR_CAN_DRIVER_COMMANDS_FRAME_ID)
    {
      struct car_can_driver_commands_t msg;

      ret = car_can_driver_commands_unpack(&msg, frame->data,
                                            frame->can_dlc);
      if (ret == 0)
        {
          g_direction = msg.demanded_drive_direction;
          widgets_set_direction(g_direction);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   HMI Manager application entry point.
 ****************************************************************************/

int main(int argc, char *argv[])
{
  struct can_frame frame;
  struct timeval tv;
  fd_set readfds;
  int can_fd;
  int button;
  int ret;

  printf("\n");
  printf("================================\n");
  printf("  HMI Manager\n");
  printf("================================\n");
  printf("\n");

  /* Initialize framebuffer + LVGL */

  printf("Initializing display...\n");
  ret = fb_handler_init("/dev/fb0");
  if (ret < 0)
    {
      printf("ERROR: fb_handler_init failed: %d\n", ret);
      return 1;
    }

  /* Initialize CAN */

  printf("Initializing CAN...\n");
  ret = can_handler_init("can0");
  if (ret < 0)
    {
      printf("ERROR: can_handler_init failed: %d\n", ret);
      fb_handler_close();
      return 1;
    }

  can_fd = can_handler_get_fd();

  /* Initialize CAN trace */

  printf("Initializing CAN trace...\n");
  can_trace_init();
  can_trace_enable(true);  /* enabled by default */

  /* Initialize buttons */

  printf("Initializing buttons...\n");
  ret = button_irq_init("/dev/buttons");
  if (ret < 0)
    {
      printf("WARNING: button_irq_init failed: %d\n", ret);
    }

  /* Create widgets */

  printf("Creating UI...\n");
  ret = widgets_init();
  if (ret < 0)
    {
      printf("ERROR: widgets_init failed: %d\n", ret);
      can_handler_close();
      fb_handler_close();
      return 1;
    }

  /* Initial display update */

  widgets_set_speed(g_speed);
  widgets_set_voltage(g_voltage);
  widgets_set_current(g_current);
  widgets_set_direction(g_direction);

  printf("\n");
  printf("Ready!\n");
  printf("\n");
  printf("CAN RX IDs:\n");
  printf("  InverterDemanders: 0x%08lX\n",
         (unsigned long)CAR_CAN_INVERTER_DEMANDERS_FRAME_ID);
  printf("  SystemVoltages:    0x%08lX\n",
         (unsigned long)CAR_CAN_SYSTEM_VOLTAGES_FRAME_ID);
  printf("  DriverCommands:    0x%08lX\n",
         (unsigned long)CAR_CAN_DRIVER_COMMANDS_FRAME_ID);
  printf("\n");

  /* Main loop */

  while (1)
    {
      /* Check button press */

      button = button_irq_get_pressed();
      if (button != BUTTON_INVALID)
        {
          process_button_press(button);
          button_irq_clear();
        }

      /* Wait for CAN message with timeout */

      FD_ZERO(&readfds);
      FD_SET(can_fd, &readfds);
      tv.tv_sec = 0;
      tv.tv_usec = LVGL_TICK_MS * 1000;

      ret = select(can_fd + 1, &readfds, NULL, NULL, &tv);
      if (ret > 0 && FD_ISSET(can_fd, &readfds))
        {
          ret = can_handler_read(&frame);
          if (ret == 0)
            {
              can_trace_rx(&frame);
              process_can_frame(&frame);
            }
        }

      /* Update LVGL */

      lv_tick_inc(LVGL_TICK_MS);
      lv_timer_handler();
    }

  /* Cleanup (never reached) */

  button_irq_close();
  can_handler_close();
  fb_handler_close();

  return 0;
}
