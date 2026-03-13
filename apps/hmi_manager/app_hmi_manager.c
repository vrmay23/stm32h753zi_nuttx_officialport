/****************************************************************************
 * apps/hmi_manager/app_hmi_manager.c
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
 * Architecture Overview
 *
 * This application uses a multi-threaded architecture to avoid
 * starvation between CAN I/O and LVGL rendering.
 *
 * Thread model:
 *
 *   main thread   - command socket (hmi_ctl) + button polling
 *   can_rx thread - blocking read() on CAN socket, high prio
 *   can_tx_100ms  - cyclic DRIVER_COMMANDS, normal prio
 *   can_tx_1000ms - cyclic HMI_INFO heartbeat, normal prio
 *   lvgl thread   - lv_tick_inc + lv_timer_handler, low prio
 *
 * The CAN RX thread uses blocking read() so it never competes
 * with LVGL for CPU time. Each subsystem runs independently.
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
#include <pthread.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <nuttx/can.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
#include <sys/ioctl.h>
#include <nuttx/video/fb.h>
#include <lvgl/lvgl.h>
#endif

#include "can/can_handler.h"
#include "can/car_can.h"
#include "can/can_trace.h"
#include "io_handler/button_irq.h"
#include "io_handler/led_control.h"
#include "cmd/commands.h"

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
#include "uiux/fb_handler.h"
#include "uiux/widgets.h"
#include "uiux/screen/screen_manager.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* SCHED PRIORITY */

#define SCHED_PRIORITY_VERRY_HIGH (SCHED_PRIORITY_DEFAULT + 20)
#define SCHED_PRIORITY_HIGH       (SCHED_PRIORITY_DEFAULT + 10)
#define SCHED_PRIORITY_STD        (SCHED_PRIORITY_DEFAULT +  0)
#define SCHED_PRIORITY_LOW        (SCHED_PRIORITY_DEFAULT - 10) 
#define SCHED_PRIORITY_VERRY_LOW  (SCHED_PRIORITY_DEFAULT - 20)

/* LVGL tick period in milliseconds */

#define LVGL_TICK_MS          10

/* Command buffer size for hmi_ctl IPC */

#define CMD_BUFFER_SIZE       256

/* Unix domain socket path for hmi_ctl commands */

#define HMI_SOCKET_PATH       "/tmp/hmi_cmd"

/* CAN 2.0B maximum payload size */

#define CAN_MAX_PAYLOAD       8

/* Thread stack sizes */

#define CAN_RX_STACK_SIZE     2048
#define CAN_TX_STACK_SIZE     2048
#define LVGL_STACK_SIZE       4096

/* Thread priorities (higher = more important)
 *
 * CAN RX must be highest to never lose frames.
 * CAN TX at default to guarantee cyclic transmission.
 * LVGL lowest - rendering is not safety-critical.
 * Main at default for command processing.
 */

#define CAN_RX_PRIORITY       SCHED_PRIORITY_VERRY_HIGH
#define CAN_TX_PRIORITY       SCHED_PRIORITY_HIGH 
#define LVGL_PRIORITY         SCHED_PRIORITY_LOW

/* Command socket poll timeout (milliseconds) */

#define CMD_POLL_TIMEOUT_MS   50

/* Speed conversion parameters (from Kconfig) */

#define GEAR_RATIO      CONFIG_HMI_MANAGER_GEAR_RATIO
#define WHEEL_DIAM_M \
    (CONFIG_HMI_MANAGER_WHEEL_DIAMETER_MM / 1000.0)

/* Splash screen hold duration (from Kconfig choice).
 * HMI_SPLASH_TIME_0 or HMI_SPLASH_NONE -> 0 ms
 * HMI_SPLASH_TIME_1 -> 1000 ms
 * HMI_SPLASH_TIME_2 -> 2000 ms
 */

#if defined(CONFIG_HMI_SPLASH_TIME_2)
#  define SPLASH_TIME_MS  2000
#elif defined(CONFIG_HMI_SPLASH_TIME_1)
#  define SPLASH_TIME_MS  1000
#else
#  define SPLASH_TIME_MS  0
#endif

/* Button 6 carousel: cycles through these screens in order.
 * SCREEN_DASHBOARD -> SCREEN_THEME -> repeat
 */

#define BTN6_CAROUSEL_COUNT  2

/* Button 7 screen-off toggle state */

#define SCREEN_OFF_DISABLED  0   /* display is ON  (normal)  */
#define SCREEN_OFF_ENABLED   1   /* display is OFF (blanked) */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Vehicle state (updated from CAN RX, read by LVGL) */

static int g_speed      = 0;   /* km/h converted from RPM           */
static int g_rpm        = 0;   /* raw motor RPM                     */
static int g_voltage    = 0;   /* battery voltage decivolts         */
static int g_current    = 0;   /* motor current amps                */
static int g_direction  = WIDGET_DIR_NEUTRAL;
static int g_drive_mode = WIDGET_MODE_STANDARD;

/* DC Link state (updated from INVERTER_INFO CAN message) */

static uint8_t g_dc_link_state =
    CAR_CAN_INVERTER_INFO_DC_LINK_STATE_DC_LINK_OFF_CHOICE;

/* CAN TX messages (persistent state, written by buttons,
 * read by TX threads)
 */

static struct car_can_driver_commands_t g_driver_cmd;
static struct car_can_hmi_info_t g_hmi_info;

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
/* Button 6 carousel sequence */

static const screen_id_t g_btn6_carousel[BTN6_CAROUSEL_COUNT] =
{
  SCREEN_DASHBOARD,
  SCREEN_THEME
};

static int g_btn6_index = 0;

/* Screen-off toggle: SCREEN_OFF_DISABLED (0) = display ON,
 *                    SCREEN_OFF_ENABLED  (1) = display OFF.
 * Written by process_button_press() (main thread).
 * Screen switch deferred to lvgl_thread via g_screen_request.
 */

static int g_screen_off = SCREEN_OFF_DISABLED;

/* UI dirty flag + snapshot
 *
 * can_rx_thread writes to g_speed/g_rpm/g_voltage/g_direction
 * and then sets g_ui_dirty = true.
 * lvgl_thread reads g_ui_dirty, copies the snapshot, resets the
 * flag, and calls widgets_set_*() — the ONLY place LVGL is touched.
 *
 * On Cortex-M7, int/bool reads/writes are single-instruction and
 * therefore atomic for this producer-consumer pattern. No mutex
 * needed: the worst case is lvgl_thread reads a value that is one
 * LVGL tick (10 ms) stale, which is acceptable for a vehicle HMI.
 */

static volatile bool g_ui_dirty = false;

struct ui_snapshot_t
{
  int speed;      /* km/h     */
  int rpm;        /* raw RPM  */
  int voltage;    /* dV       */
  int current;    /* amps     */
  int direction;  /* WIDGET_DIR_* */
  int drive_mode; /* WIDGET_MODE_* */
};

static struct ui_snapshot_t g_ui_snap;

/* Screen switch request from main/button thread to lvgl_thread.
 * SCREEN_COUNT means "no pending request".
 * Written by process_button_press(), read by lvgl_thread.
 */

static volatile screen_id_t g_screen_request = SCREEN_COUNT;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpm_to_kmh
 *
 * Description:
 *   Converts motor RPM to vehicle speed in km/h.
 *
 *   Formula: km/h = (RPM x pi x D x 60) / (ratio x 1000)
 *
 *   Derivation:
 *     1. Wheel RPM = Motor RPM / GEAR_RATIO
 *     2. Wheel circumference = pi x D (meters)
 *     3. Distance per minute = Wheel RPM x circumference
 *     4. Distance per hour = distance per minute x 60
 *     5. km/h = distance per hour / 1000
 *
 ****************************************************************************/

static int rpm_to_kmh(int rpm)
{
  double wheel_rpm;
  double circumference;
  double kmh;

  wheel_rpm = (double)rpm / GEAR_RATIO;
  circumference = 3.14159 * WHEEL_DIAM_M;
  kmh = (wheel_rpm * circumference * 60.0) / 1000.0;

  return (int)kmh;
}

/****************************************************************************
 * Name: toggle_dc_link_demand
 *
 * Description:
 *   Toggles DC Link demand state (ON <-> OFF).
 *   Updates g_driver_cmd.dc_link_active_demand which is
 *   sent cyclically via CAN TX thread.
 *
 ****************************************************************************/

static void toggle_dc_link_demand(void)
{
  if (g_driver_cmd.dc_link_active_demand ==
      CAR_CAN_DRIVER_COMMANDS_DC_LINK_ACTIVE_DEMAND_NO_DEMAND_CHOICE)
    {
      g_driver_cmd.dc_link_active_demand =
          CAR_CAN_DRIVER_COMMANDS_DC_LINK_ACTIVE_DEMAND_ACTIVE_DEMAND_CHOICE;
      printf("DC Link: Demand ON\n");
    }
  else
    {
      g_driver_cmd.dc_link_active_demand =
          CAR_CAN_DRIVER_COMMANDS_DC_LINK_ACTIVE_DEMAND_NO_DEMAND_CHOICE;
      printf("DC Link: Demand OFF\n");
    }
}

/****************************************************************************
 * Name: process_button_press
 *
 * Description:
 *   Handles button press events and updates CAN message state.
 *   Does NOT send CAN directly - state is transmitted by cyclic
 *   100ms TX thread.
 *
 * Button Mapping:
 *   BTN0: Toggle DC Link demand (ON/OFF)
 *   BTN1: Enable drive Forward (requires DC Link ON)
 *   BTN2: Enable drive Reverse (requires DC Link ON)
 *   BTN3: Disable drive (Neutral)
 *   BTN4: Reset inverter faults
 *   BTN5: Toggle pedal mode (ECO/NORMAL/SPORT)
 *   BTN6: Carousel: dashboard -> theme -> splash
 *   BTN7: ScreenOff
 *   BTN8-10: Reserved
 *
 ****************************************************************************/

static void process_button_press(int button)
{
  switch (button)
    {
      case BUTTON_DC_LINK_TOGGLE:
        toggle_dc_link_demand();
        break;

      case BUTTON_FORWARD:

        /* Safety interlock: DC Link must be ON */

        if (g_dc_link_state !=
            CAR_CAN_INVERTER_INFO_DC_LINK_STATE_DC_LINK_ON_CHOICE)
          {
            printf("ERROR: Cannot enable drive - "
                   "DC Link not ready (state=%d)\n",
                   g_dc_link_state);
            return;
          }

        g_driver_cmd.demanded_drive_direction =
            WIDGET_DIR_FORWARD;
        g_driver_cmd.demanded_drive_state =
            CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_STATE_ENABLE_CHOICE;
        g_direction = WIDGET_DIR_FORWARD;
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
        g_ui_dirty = true;
#endif
        printf("Drive: FORWARD enabled\n");
        break;

      case BUTTON_REVERSE:

        /* Safety interlock: DC Link must be ON */

        if (g_dc_link_state !=
            CAR_CAN_INVERTER_INFO_DC_LINK_STATE_DC_LINK_ON_CHOICE)
          {
            printf("ERROR: Cannot enable drive - "
                   "DC Link not ready (state=%d)\n",
                   g_dc_link_state);
            return;
          }

        g_driver_cmd.demanded_drive_direction =
            WIDGET_DIR_REVERSE;
        g_driver_cmd.demanded_drive_state =
            CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_STATE_ENABLE_CHOICE;
        g_direction = WIDGET_DIR_REVERSE;
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
        g_ui_dirty = true;
#endif
        printf("Drive: REVERSE enabled\n");
        break;

      case BUTTON_NEUTRAL:
        g_driver_cmd.demanded_drive_direction =
            WIDGET_DIR_NEUTRAL;
        g_driver_cmd.demanded_drive_state =
            CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_STATE_DISABLE_CHOICE;
        g_direction = WIDGET_DIR_NEUTRAL;
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
        g_ui_dirty = true;
#endif
        printf("Drive: NEUTRAL (disabled)\n");
        break;

      case BUTTON_RESET_FAULTS:
        g_driver_cmd.reset_inverter_errrors = 1;
        printf("Inverter faults: RESET requested\n");
        break;

      case BUTTON_PEDAL_MODE:
        g_driver_cmd.pedal_setting =
            (g_driver_cmd.pedal_setting + 1) % 3;
        g_drive_mode = g_driver_cmd.pedal_setting;
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
        g_ui_dirty = true;
#endif
        printf("Pedal mode: %d ",
               g_driver_cmd.pedal_setting);
        switch (g_driver_cmd.pedal_setting)
          {
            case 0:
              printf("(ECO)\n");
              break;
            case 1:
              printf("(NORMAL)\n");
              break;
            case 2:
              printf("(SPORT)\n");
              break;
          }
        break;

      case 6:
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
        {
          /* Re-enable backlight if leaving black screen.
           * The actual screen switch is deferred to lvgl_thread
           * via g_screen_request to avoid calling lv_scr_load()
           * from the main thread (LVGL is not thread-safe).
           */

          if (screen_manager_current() == SCREEN_BLACK)
            {
              int fb_fd = open("/dev/fb0", O_RDWR);
              if (fb_fd >= 0)
                {
                  ioctl(fb_fd, FBIOSET_POWER, 1);
                  close(fb_fd);
                }
            }

          g_btn6_index = (g_btn6_index + 1)
                         % BTN6_CAROUSEL_COUNT;
          g_screen_request = g_btn6_carousel[g_btn6_index];
        }
#endif
        break;

      case 7:
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
        {
          int fb_fd;

          if (g_screen_off == SCREEN_OFF_DISABLED)
            {
              /* Screen is ON -> turn it OFF */

              g_screen_off = SCREEN_OFF_ENABLED;
              g_screen_request = SCREEN_BLACK;
              fb_fd = open("/dev/fb0", O_RDWR);
              if (fb_fd >= 0)
                {
                  ioctl(fb_fd, FBIOSET_POWER, 0);
                  close(fb_fd);
                }
              printf("Screen: OFF\n");
            }
          else
            {
              /* Screen is OFF -> turn it back ON */

              g_screen_off = SCREEN_OFF_DISABLED;
              fb_fd = open("/dev/fb0", O_RDWR);
              if (fb_fd >= 0)
                {
                  ioctl(fb_fd, FBIOSET_POWER, 1);
                  close(fb_fd);
                }

              /* Restore last carousel screen */

              g_screen_request = g_btn6_carousel[g_btn6_index];
              printf("Screen: ON (restored)\n");
            }
        }
#endif
        break;

      case 8:
      case 9:
      case 10:

        /* Reserved for future features */

        printf("Button %d: Reserved (no action)\n", button);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: process_can_frame
 *
 * Description:
 *   Decode received CAN frame and update vehicle state.
 *   Called from can_rx_thread context.
 *
 ****************************************************************************/

static void process_can_frame(struct can_frame *frame)
{
  uint32_t id;
  int ret;

  id = frame->can_id & CAN_EFF_MASK;

  if (id == CAR_CAN_INVERTER_SPEED_INFO_FRAME_ID)
    {
      struct car_can_inverter_speed_info_t msg;

      ret = car_can_inverter_speed_info_unpack(
              &msg, frame->data, frame->can_dlc);
      if (ret == 0)
        {
          int rpm =
              (int)car_can_inverter_speed_info_em_speed_rpm_decode(
                  msg.em_speed_rpm);
          g_rpm   = (rpm < 0) ? -rpm : rpm;
          g_speed = rpm_to_kmh(g_rpm);

          /* Signal lvgl_thread to refresh widgets — do NOT call
           * any LVGL API from this thread (LVGL is not thread-safe).
           */

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
          g_ui_dirty = true;
#endif
        }
    }
  else if (id == CAR_CAN_SYSTEM_VOLTAGES_FRAME_ID)
    {
      struct car_can_system_voltages_t msg;

      ret = car_can_system_voltages_unpack(
              &msg, frame->data, frame->can_dlc);
      if (ret == 0)
        {
          g_voltage =
              (int)(car_can_system_voltages_meas_batt_voltage_decode(
                  msg.meas_batt_voltage) * 10.0);
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
          g_ui_dirty = true;
#endif
        }
    }
  else if (id == CAR_CAN_DRIVER_COMMANDS_FRAME_ID)
    {
      struct car_can_driver_commands_t msg;

      ret = car_can_driver_commands_unpack(
              &msg, frame->data, frame->can_dlc);
      if (ret == 0)
        {
          /* N = drive disabled regardless of direction */
          /* D = forward + enabled                      */
          /* R = backward + enabled                     */

          if (msg.demanded_drive_state ==
              CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_STATE_DISABLE_CHOICE)
            {
              g_direction = WIDGET_DIR_NEUTRAL;
            }
          else if (msg.demanded_drive_direction ==
                   CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_DIRECTION_BACKWARD_CHOICE)
            {
              g_direction = WIDGET_DIR_REVERSE;
            }
          else
            {
              g_direction = WIDGET_DIR_FORWARD;
            }

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
          g_ui_dirty = true;
#endif
        }
    }
  else if (id == CAR_CAN_INVERTER_INFO_FRAME_ID)
    {
      struct car_can_inverter_info_t msg;

      ret = car_can_inverter_info_unpack(
              &msg, frame->data, frame->can_dlc);
      if (ret == 0)
        {
          g_dc_link_state = msg.dc_link_state;

          if (msg.dc_link_state ==
              CAR_CAN_INVERTER_INFO_DC_LINK_STATE_DC_LINK_ON_CHOICE)
            {
              led_control_on(LED_DC_LINK_ON);
              led_control_off(LED_DC_LINK_PRECHARGE);
            }
          else if (msg.dc_link_state ==
              CAR_CAN_INVERTER_INFO_DC_LINK_STATE_PRE_CHARGING_CHOICE)
            {
              led_control_off(LED_DC_LINK_ON);
              led_control_on(LED_DC_LINK_PRECHARGE);
            }
          else
            {
              led_control_off(LED_DC_LINK_ON);
              led_control_off(LED_DC_LINK_PRECHARGE);
            }
        }
    }
}

/****************************************************************************
 * Name: process_command
 *
 * Description:
 *   Parse command string received from hmi_ctl via Unix socket.
 *
 ****************************************************************************/

static void process_command(const char *cmd_str)
{
  char *argv[16];
  char cmd_copy[CMD_BUFFER_SIZE];
  int argc;
  char *token;
  int i;

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
      /* Shift argv to insert program name at [0] */

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
 *
 * Description:
 *   Create and bind Unix domain socket for hmi_ctl IPC.
 *   Socket is set non-blocking for use with select().
 *
 * Returned Value:
 *   Socket fd on success, -1 on failure.
 *
 ****************************************************************************/

static int init_command_socket(void)
{
  struct sockaddr_un addr;
  int sock;
  int ret;
  int flags;

  unlink(HMI_SOCKET_PATH);

  sock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock < 0)
    {
      printf("ERROR: socket() failed: %d\n", errno);
      return -1;
    }

  flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, HMI_SOCKET_PATH,
          sizeof(addr.sun_path) - 1);

  ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0)
    {
      printf("ERROR: bind() failed: %d\n", errno);
      close(sock);
      return -1;
    }

  ret = listen(sock, 5);
  if (ret < 0)
    {
      printf("ERROR: listen() failed: %d\n", errno);
      close(sock);
      return -1;
    }

  printf("Command socket: %s (fd=%d)\n",
         HMI_SOCKET_PATH, sock);
  return sock;
}

/****************************************************************************
 * Thread Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_rx_thread
 *
 * Description:
 *   Dedicated CAN RX thread. Uses blocking read() on CAN socket.
 *   This ensures CAN frames are always consumed regardless of
 *   LVGL rendering load or command processing.
 *
 *   Runs at elevated priority to prevent RX FIFO overrun.
 *
 ****************************************************************************/

static void *can_rx_thread(void *arg)
{
  struct can_frame frame;
  int ret;

  printf("[CAN RX] Thread started\n");

  while (true)
    {
      ret = can_handler_read(&frame);
      if (ret == 0)
        {
          can_trace_rx(&frame);
          process_can_frame(&frame);
        }
      else if (ret != -EAGAIN)
        {
          /* Real error - brief sleep to avoid busy loop */

          nxsched_msleep(10);
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: can_tx_loop_100ms
 *
 * Description:
 *   Cyclic CAN TX thread - sends DRIVER_COMMANDS every 100ms.
 *   Reads current state from g_driver_cmd (updated by buttons).
 *
 ****************************************************************************/

static void *can_tx_loop_100ms(void *arg)
{
  uint8_t data[CAN_MAX_PAYLOAD];

  printf("[CAN TX 100ms] Thread started\n");

  while (true)
    {
      car_can_driver_commands_pack(
          data, &g_driver_cmd,
          CAR_CAN_DRIVER_COMMANDS_LENGTH);

      can_handler_send(CAR_CAN_DRIVER_COMMANDS_FRAME_ID,
                       CAR_CAN_DRIVER_COMMANDS_IS_EXTENDED,
                       data,
                       CAR_CAN_DRIVER_COMMANDS_LENGTH);

      nxsched_msleep(100);
    }

  return NULL;
}

/****************************************************************************
 * Name: can_tx_loop_1000ms
 *
 * Description:
 *   Cyclic CAN TX thread - sends HMI_INFO every 1000ms.
 *   Acts as heartbeat so Linux side knows NuttX is alive.
 *
 ****************************************************************************/

static void *can_tx_loop_1000ms(void *arg)
{
  uint8_t data[CAN_MAX_PAYLOAD];

  printf("[CAN TX 1000ms] Thread started\n");

  while (true)
    {
      g_hmi_info.hmi1ms_ticks =
          TICK2MSEC(clock_systime_ticks());

      car_can_hmi_info_pack(
          data, &g_hmi_info,
          CAR_CAN_HMI_INFO_LENGTH);

      can_handler_send(CAR_CAN_HMI_INFO_FRAME_ID,
                       CAR_CAN_HMI_INFO_IS_EXTENDED,
                       data,
                       CAR_CAN_HMI_INFO_LENGTH);

      nxsched_msleep(1000);
    }

  return NULL;
}

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
/****************************************************************************
 * Name: lvgl_thread
 *
 * Description:
 *   Dedicated LVGL rendering thread. Calls lv_tick_inc and
 *   lv_timer_handler at LVGL_TICK_MS intervals.
 *
 *   Boot sequence:
 *     1. Attach assets (phase 2)
 *     2. Render splash (one LVGL cycle to flush framebuffer)
 *     3. Hold splash for configured duration
 *     4. Switch to dashboard
 *     5. Enter render loop
 *
 *   Runs at lower priority than CAN threads so rendering
 *   never starves CAN communication.
 *
 ****************************************************************************/

static void *lvgl_thread(void *arg)
{
  printf("[LVGL] Thread started\n");

  /* Phase 2: attach assets and build widget tree.
   * Must run inside lvgl_thread - LVGL is not thread-safe.
   */

  screen_manager_setup_assets();

  /* Render one LVGL cycle so the splash image is actually
   * flushed to the framebuffer before we hold.
   */

  lv_tick_inc(LVGL_TICK_MS);
  lv_timer_handler();

  /* Hold splash screen for the configured duration.
   * SPLASH_TIME_MS is 0 when HMI_SPLASH_NONE is selected
   * or when HMI_SPLASH_TIME_0 is selected.
   */

#if SPLASH_TIME_MS > 0
  printf("[LVGL] Splash hold %d ms\n", SPLASH_TIME_MS);
  nxsched_msleep(SPLASH_TIME_MS);
#endif

  /* Transition to dashboard */

  screen_manager_load(SCREEN_DASHBOARD);

  while (true)
    {
      /* --- Flush pending UI data (g_ui_dirty pattern) ---
       *
       * can_rx_thread / button handler only write to plain C
       * globals and set g_ui_dirty = true.  This thread is the
       * ONLY caller of any LVGL API, so there is no race.
       */

      if (g_ui_dirty)
        {
          /* Snapshot globals before clearing the flag so we
           * don't miss an update that arrives during the copy.
           */

          g_ui_snap.speed      = g_speed;
          g_ui_snap.rpm        = g_rpm;
          g_ui_snap.voltage    = g_voltage;
          g_ui_snap.current    = g_current;
          g_ui_snap.direction  = g_direction;
          g_ui_snap.drive_mode = g_drive_mode;
          g_ui_dirty = false;

          widgets_set_speed(g_ui_snap.speed);
          widgets_set_rpm(g_ui_snap.rpm);
          widgets_set_voltage(g_ui_snap.voltage);
          widgets_set_current(g_ui_snap.current);
          widgets_set_direction(g_ui_snap.direction);
          widgets_set_mode(g_ui_snap.drive_mode);
        }

      /* --- Handle deferred screen switch request --- */

      if (g_screen_request != SCREEN_COUNT)
        {
          screen_id_t req = g_screen_request;
          g_screen_request = SCREEN_COUNT;
          screen_manager_load(req);
        }

      lv_tick_inc(LVGL_TICK_MS);
      lv_timer_handler();
      nxsched_msleep(LVGL_TICK_MS);
    }

  return NULL;
}
#endif

/****************************************************************************
 * Helper Functions
 ****************************************************************************/

/****************************************************************************
 * Name: create_thread
 *
 * Description:
 *   Helper to create a pthread with specified stack and priority.
 *
 * Input Parameters:
 *   tid      - Pointer to thread ID (output)
 *   func     - Thread entry function
 *   stack    - Stack size in bytes
 *   priority - Scheduling priority
 *   name     - Thread name (for debug)
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int create_thread(pthread_t *tid,
                         void *(*func)(void *),
                         int stack, int priority,
                         const char *name)
{
  pthread_attr_t attr;
  struct sched_param param;
  int ret;

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, stack);
  param.sched_priority = priority;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(tid, &attr, func, NULL);
  pthread_attr_destroy(&attr);

  if (ret != 0)
    {
      printf("ERROR: pthread_create(%s) failed: %d\n",
             name, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   HMI Manager entry point. Initializes all subsystems,
 *   spawns worker threads, then runs command processing loop.
 *
 *   Init order:
 *     1. Display (framebuffer + LVGL)
 *     2. LEDs
 *     3. CAN (socket + bind)
 *     4. CAN trace
 *     5. CAN TX message defaults
 *     6. Buttons
 *     7. Command socket
 *     8. Screen manager (phase 1 - skeletons only)
 *     9. Spawn threads (CAN RX, CAN TX x2, LVGL)
 *    10. Main loop (command socket + buttons only)
 *
 ****************************************************************************/

int main(int argc, char *argv[])
{
  char cmd_buffer[CMD_BUFFER_SIZE];
  pthread_t rx_tid;
  pthread_t tx_100ms_tid;
  pthread_t tx_1000ms_tid;
  struct timeval tv;
  fd_set readfds;
  int cmd_fd;
  int button;
  int ret;
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
  pthread_t lvgl_tid;
#endif

  printf("\n============================\n");
  printf("  HMI Manager Daemon\n");
  printf("============================\n\n");

  /* --- 1. Display init --- */

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
  printf("Initializing display...\n");
  ret = fb_handler_init("/dev/fb0");
  if (ret < 0)
    {
      printf("ERROR: fb_handler_init failed: %d\n", ret);
      return 1;
    }
#endif

  /* --- 2. LEDs init --- */

  printf("Initializing LEDs...\n");
  ret = led_control_init("/dev/userleds");
  if (ret < 0)
    {
      printf("WARNING: led_control_init failed: %d\n", ret);
    }

  /* --- 3. CAN init --- */

  printf("Initializing CAN...\n");
  ret = can_handler_init("can0");
  if (ret < 0)
    {
      printf("ERROR: can_handler_init failed: %d\n", ret);
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
      fb_handler_close();
#endif
      return 1;
    }

  /* --- 4. CAN trace init --- */

  printf("Initializing CAN trace...\n");
  can_trace_init();
  can_trace_enable(false);

  /* --- 5. CAN TX message defaults --- */

  g_driver_cmd.dc_link_active_demand =
      CAR_CAN_DRIVER_COMMANDS_DC_LINK_ACTIVE_DEMAND_NO_DEMAND_CHOICE;
  g_driver_cmd.demanded_drive_direction =
      WIDGET_DIR_FORWARD;
  g_driver_cmd.demanded_drive_state =
      CAR_CAN_DRIVER_COMMANDS_DEMANDED_DRIVE_STATE_DISABLE_CHOICE;
  g_driver_cmd.reset_inverter_errrors = 0;
  g_driver_cmd.pedal_setting = 0;
  g_hmi_info.hmi1ms_ticks = 0;

  /* --- 6. Buttons init --- */

  printf("Initializing buttons...\n");
  ret = button_irq_init("/dev/buttons");
  if (ret < 0)
    {
      printf("WARNING: button_irq_init failed: %d\n", ret);
    }

  /* --- 7. Command socket init --- */

  printf("Initializing command interface...\n");
  cmd_fd = init_command_socket();
  if (cmd_fd < 0)
    {
      printf("ERROR: init_command_socket failed\n");
      can_handler_close();
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
      fb_handler_close();
#endif
      return 1;
    }

  /* --- 8. UI screen manager init (phase 1 - no assets) --- */

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
  printf("Creating UI...\n");
  ret = screen_manager_init();
  if (ret < 0)
    {
      printf("ERROR: screen_manager_init failed: %d\n",
             ret);
      close(cmd_fd);
      can_handler_close();
      fb_handler_close();
      return 1;
    }
#endif

  /* --- 9. Spawn worker threads --- */

  printf("Starting threads...\n");

  ret = create_thread(&rx_tid, can_rx_thread,
                      CAN_RX_STACK_SIZE, CAN_RX_PRIORITY,
                      "can_rx");
  if (ret != 0)
    {
      return 1;
    }

  ret = create_thread(&tx_100ms_tid, can_tx_loop_100ms,
                      CAN_TX_STACK_SIZE, CAN_TX_PRIORITY,
                      "can_tx_100ms");
  if (ret != 0)
    {
      return 1;
    }

  ret = create_thread(&tx_1000ms_tid, can_tx_loop_1000ms,
                      CAN_TX_STACK_SIZE, CAN_TX_PRIORITY,
                      "can_tx_1000ms");
  if (ret != 0)
    {
      return 1;
    }

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
  ret = create_thread(&lvgl_tid, lvgl_thread,
                      LVGL_STACK_SIZE, LVGL_PRIORITY,
                      "lvgl");
  if (ret != 0)
    {
      return 1;
    }
#endif

  printf("\nReady!\n");
  printf("Use 'hmi_ctl <command>' to control\n\n");

  /* --- 10. Main loop: commands + buttons only --- */

  while (true)
    {
      /* Poll buttons */

      button = button_irq_get_pressed();
      if (button != BUTTON_INVALID)
        {
          process_button_press(button);
          button_irq_clear();
        }

      /* Wait for command socket activity */

      FD_ZERO(&readfds);
      FD_SET(cmd_fd, &readfds);
      tv.tv_sec = 0;
      tv.tv_usec = CMD_POLL_TIMEOUT_MS * 1000;

      ret = select(cmd_fd + 1, &readfds, NULL, NULL, &tv);

      if (ret > 0 && FD_ISSET(cmd_fd, &readfds))
        {
          int client_fd;
          struct sockaddr_un client_addr;
          socklen_t client_len = sizeof(client_addr);

          client_fd = accept(cmd_fd,
                             (struct sockaddr *)&client_addr,
                             &client_len);
          if (client_fd >= 0)
            {
              ret = recv(client_fd, cmd_buffer,
                         sizeof(cmd_buffer) - 1, 0);
              if (ret > 0)
                {
                  cmd_buffer[ret] = '\0';
                  process_command(cmd_buffer);
                }

              close(client_fd);
            }
        }
    }

  /* Cleanup (unreachable in normal operation) */

  close(cmd_fd);
  unlink(HMI_SOCKET_PATH);
  button_irq_close();
  led_control_close();
  can_handler_close();

#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
  fb_handler_close();
#endif

  return 0;
}
