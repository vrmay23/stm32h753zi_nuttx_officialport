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
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <nuttx/can.h>
#include <lvgl/lvgl.h>
#include <nuttx/wqueue.h>

#include "can/can_handler.h"
#include "can/car_can.h"
#include "can/can_trace.h"
#include "io_handler/button_irq.h"
#include "io_handler/led_control.h"
#include "uiux/fb_handler.h"
#include "uiux/widgets.h"
#include "cmd/commands.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* APP Configuration */

#define LVGL_TICK_MS          10
#define CMD_BUFFER_SIZE       256
#define HMI_SOCKET_PATH       "/tmp/hmi_cmd"

/* Button to CAN mapping */

#define BTN0_ACTION_ID        CAR_CAN_DRIVER_COMMANDS_FRAME_ID
#define BTN0_ACTION_DIR       WIDGET_DIR_NEUTRAL
#define BTN1_ACTION_ID        CAR_CAN_DRIVER_COMMANDS_FRAME_ID
#define BTN1_ACTION_DIR       WIDGET_DIR_FORWARD
#define BTN2_ACTION_ID        CAR_CAN_DRIVER_COMMANDS_FRAME_ID
#define BTN2_ACTION_DIR       WIDGET_DIR_REVERSE

/* CAN 2.0B MAX Payload */

#define CAN_MAX_PAYLOAD       8

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Truck interaction (HMI) */

static int g_speed   = 0;
static int g_voltage = 0;
static int g_current = 0;
static int g_direction = WIDGET_DIR_NEUTRAL;

/* Work queue structures for periodic CAN transmission */

static struct work_s can_tx_100ms_work;
static struct work_s can_tx_1000ms_work;

/* Global CAN TX messages (persistent state) */

static struct car_can_driver_commands_t driver_commands_msg;
static struct car_can_hmi_info_t hmi_info_msg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: process_button_press
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
 ****************************************************************************/

static void process_can_frame(struct can_frame *frame)
{
  uint32_t id;
  int ret;

  id = frame->can_id & CAN_EFF_MASK;

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
 * Name: process_command
 ****************************************************************************/

static void process_command(const char *cmd_str)
{
  char *argv[16];
  char cmd_copy[CMD_BUFFER_SIZE];
  int argc;
  char *token;
  int i;

  printf("DEBUG: Received command: '%s'\n", cmd_str);

  strncpy(cmd_copy, cmd_str, sizeof(cmd_copy) - 1);
  cmd_copy[sizeof(cmd_copy) - 1] = '\0';

  argc = 0;
  token = strtok(cmd_copy, " ");
  while (token != NULL && argc < 16)
    {
      argv[argc++] = token;
      token = strtok(NULL, " ");
    }

  if (argc > 0)
    {
      for (i = argc; i > 0; i--)
        {
          argv[i] = argv[i - 1];
        }

      argv[0] = "hmi_ctl";
      argc++;

      commands_parse(argc, argv);
    }
}

/****************************************************************************
 * Name: init_command_socket
 ****************************************************************************/

static int init_command_socket(void)
{
  struct sockaddr_un addr;
  int sock;
  int ret;
  int flags;

  unlink(HMI_SOCKET_PATH);

  /* Create STREAM socket */

  sock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock < 0)
    {
      printf("ERROR: socket() failed: %d\n", errno);
      return -1;
    }

  /* Set non-blocking */

  flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  /* Bind */

  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, HMI_SOCKET_PATH, sizeof(addr.sun_path) - 1);

  ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0)
    {
      printf("ERROR: bind() failed: %d\n", errno);
      close(sock);
      return -1;
    }

  /* Listen for connections */

  ret = listen(sock, 5);
  if (ret < 0)
    {
      printf("ERROR: listen() failed: %d\n", errno);
      close(sock);
      return -1;
    }

  printf("Command socket: %s (fd=%d)\n", HMI_SOCKET_PATH, sock);
  return sock;
}

/****************************************************************************
 * Name: can_tx_100ms_worker
 *
 * Description:
 *   Work queue callback - sends DRIVER_COMMANDS every 100ms.
 *   Runs in LPWORK thread context (safe for CAN operations).
 *
 ****************************************************************************/

static void can_tx_100ms_worker(void *arg)
{
  uint8_t data[CAN_MAX_PAYLOAD];

  /* Pack message */

  car_can_driver_commands_pack(data, &driver_commands_msg,
                                CAR_CAN_DRIVER_COMMANDS_LENGTH);

  /* Send CAN - SAFE (worker thread context) */

  can_handler_send(CAR_CAN_DRIVER_COMMANDS_FRAME_ID,
                   CAR_CAN_DRIVER_COMMANDS_IS_EXTENDED,
                   data, CAR_CAN_DRIVER_COMMANDS_LENGTH);

  /* Re-schedule for next cycle (100ms later) */

  work_queue(LPWORK, &can_tx_100ms_work, can_tx_100ms_worker,
             NULL, MSEC2TICK(100));
}

/****************************************************************************
 * Name: can_tx_1000ms_worker
 *
 * Description:
 *   Work queue callback - sends HMI_INFO every 1000ms.
 *   Runs in LPWORK thread context.
 *
 ****************************************************************************/

static void can_tx_1000ms_worker(void *arg)
{
  uint8_t data[CAN_MAX_PAYLOAD];

  /* Update timestamp */

  hmi_info_msg.hmi1ms_ticks = TICK2MSEC(clock_systime_ticks());

  /* Pack message */

  car_can_hmi_info_pack(data, &hmi_info_msg,
                        CAR_CAN_HMI_INFO_LENGTH);

  /* Send CAN */

  can_handler_send(CAR_CAN_HMI_INFO_FRAME_ID,
                   CAR_CAN_HMI_INFO_IS_EXTENDED,
                   data, CAR_CAN_HMI_INFO_LENGTH);

  /* Re-schedule for next cycle (1000ms later) */

  work_queue(LPWORK, &can_tx_1000ms_work, can_tx_1000ms_worker,
             NULL, MSEC2TICK(1000));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  struct can_frame frame;
  struct timeval tv;
  fd_set readfds;
  char cmd_buffer[CMD_BUFFER_SIZE];
  int can_fd;
  int cmd_fd;
  int button;
  int maxfd;
  int ret;

  printf("\n============================\n");
  printf("  HMI Manager Daemon          \n");
  printf("============================\n\n");

  printf("Initializing display...\n");
  ret = fb_handler_init("/dev/fb0");
  if (ret < 0)
    {
      printf("ERROR: fb_handler_init failed: %d\n", ret);
      return 1;
    }

  printf("Initializing LEDs...\n");
  ret = led_control_init("/dev/userleds");
  if (ret < 0)
    {
      printf("WARNING: led_control_init failed: %d\n", ret);
    }

  printf("Initializing CAN...\n");
  ret = can_handler_init("can0");
  if (ret < 0)
    {
      printf("ERROR: can_handler_init failed: %d\n", ret);
      fb_handler_close();
      return 1;
    }

  can_fd = can_handler_get_fd();

  printf("Initializing CAN trace...\n");
  can_trace_init();
  can_trace_enable(false);

  /* Initialize CAN TX message defaults */

  driver_commands_msg.dc_link_active_demand =
      CAR_CAN_DRIVER_COMMANDS_DC_LINK_ACTIVE_DEMAND_NO_DEMAND_CHOICE;

  driver_commands_msg.demanded_drive_direction =
      CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_DIRECTION_FORWARD_CHOICE;

  driver_commands_msg.demanded_drive_state =
      CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_STATE_DISABLE_CHOICE;

  driver_commands_msg.reset_inverter_errrors = 0;
  driver_commands_msg.pedal_setting = 0;

  hmi_info_msg.hmi1ms_ticks = 0;

  /* Start periodic CAN transmission work */

  work_queue(LPWORK, &can_tx_100ms_work, can_tx_100ms_worker,
             NULL, MSEC2TICK(100));
  work_queue(LPWORK, &can_tx_1000ms_work, can_tx_1000ms_worker,
             NULL, MSEC2TICK(1000));

  printf("CAN TX work queues started\n");

  printf("Initializing buttons...\n");
  ret = button_irq_init("/dev/buttons");
  if (ret < 0)
    {
      printf("WARNING: button_irq_init failed: %d\n", ret);
    }

  printf("Initializing command interface...\n");
  cmd_fd = init_command_socket();
  if (cmd_fd < 0)
    {
      printf("ERROR: init_command_socket failed\n");
      can_handler_close();
      fb_handler_close();
      return 1;
    }

  printf("Creating UI...\n");
  ret = widgets_init();
  if (ret < 0)
    {
      printf("ERROR: widgets_init failed: %d\n", ret);
      close(cmd_fd);
      can_handler_close();
      fb_handler_close();
      return 1;
    }

  widgets_set_speed(g_speed);
  widgets_set_voltage(g_voltage);
  widgets_set_current(g_current);
  widgets_set_direction(g_direction);

  printf("\nReady!\n");
  printf("Use 'hmi_ctl <command>' to control the daemon\n\n");

  maxfd = (can_fd > cmd_fd ? can_fd : cmd_fd) + 1;

  while (1)
    {
      button = button_irq_get_pressed();
      if (button != BUTTON_INVALID)
        {
          process_button_press(button);
          button_irq_clear();
        }

      FD_ZERO(&readfds);
      FD_SET(can_fd, &readfds);
      FD_SET(cmd_fd, &readfds);
      tv.tv_sec = 0;
      tv.tv_usec = LVGL_TICK_MS * 1000;

      ret = select(maxfd, &readfds, NULL, NULL, &tv);

      if (ret > 0 && FD_ISSET(can_fd, &readfds))
        {
          ret = can_handler_read(&frame);
          if (ret == 0)
            {
              can_trace_rx(&frame);
              process_can_frame(&frame);
            }
        }

      if (ret > 0 && FD_ISSET(cmd_fd, &readfds))
        {
          int client_fd;
          struct sockaddr_un client_addr;
          socklen_t client_len = sizeof(client_addr);

          printf("DEBUG daemon: accepting connection...\n");
          client_fd = accept(cmd_fd, (struct sockaddr *)&client_addr,
                             &client_len);
          if (client_fd >= 0)
            {
              printf("DEBUG daemon: client connected fd=%d\n", client_fd);
              ret = recv(client_fd, cmd_buffer, sizeof(cmd_buffer) - 1, 0);
              printf("DEBUG daemon: recv returned %d\n", ret);
              if (ret > 0)
                {
                  cmd_buffer[ret] = '\0';
                  printf("DEBUG daemon: received '%s'\n", cmd_buffer);
                  process_command(cmd_buffer);
                }
              else if (ret < 0)
                {
                  printf("DEBUG daemon: recv error %d\n", errno);
                }

              close(client_fd);
            }
          else
            {
              printf("DEBUG daemon: accept error %d\n", errno);
            }
        }

      lv_tick_inc(LVGL_TICK_MS);
      lv_timer_handler();
    }

  close(cmd_fd);
  unlink(HMI_SOCKET_PATH);
  button_irq_close();
  led_control_close();
  can_handler_close();
  fb_handler_close();

  return 0;
}