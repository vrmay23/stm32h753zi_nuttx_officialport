/****************************************************************************
 * apps/hmi_manager/io_handler/button_irq.c
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
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/input/buttons.h>

#include "button_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUTTON_SIGNAL  SIGUSR1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_btn_fd = -1;
static volatile int g_button_pressed = BUTTON_INVALID;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_signal_handler
 *
 * Description:
 *   Signal handler for button press interrupt.
 ****************************************************************************/

static void button_signal_handler(int signo, siginfo_t *info,
                                   void *context)
{
  btn_buttonset_t buttons;
  int i;

  if (g_btn_fd < 0)
    {
      return;
    }

  /* Read button state */

  if (read(g_btn_fd, &buttons, sizeof(buttons)) != sizeof(buttons))
    {
      return;
    }

  /* Find first pressed button (bit set) */

  for (i = 0; i < 8; i++)
    {
      if (buttons & (1 << i))
        {
          g_button_pressed = i;
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_irq_init
 *
 * Description:
 *   Initialize button interrupt handler.
 *
 * Input Parameters:
 *   devpath - Button device path (e.g., "/dev/buttons")
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int button_irq_init(const char *devpath)
{
  struct sigaction sa;
  struct btn_notify_s notify;
  btn_buttonset_t supported;
  int ret;

  if (g_btn_fd >= 0)
    {
      return -EBUSY;
    }

  /* Open button device */

  g_btn_fd = open(devpath, O_RDONLY);
  if (g_btn_fd < 0)
    {
      printf("ERROR: open(%s) failed: %d\n", devpath, errno);
      return -errno;
    }

  /* Get supported buttons */

  ret = ioctl(g_btn_fd, BTNIOC_SUPPORTED, &supported);
  if (ret < 0)
    {
      printf("ERROR: BTNIOC_SUPPORTED failed: %d\n", errno);
      close(g_btn_fd);
      g_btn_fd = -1;
      return -errno;
    }

  printf("Buttons: Supported mask = 0x%02X\n", (unsigned)supported);

  /* Setup signal handler */

  memset(&sa, 0, sizeof(sa));
  sa.sa_sigaction = button_signal_handler;
  sa.sa_flags = SA_SIGINFO;
  sigemptyset(&sa.sa_mask);

  ret = sigaction(BUTTON_SIGNAL, &sa, NULL);
  if (ret < 0)
    {
      printf("ERROR: sigaction() failed: %d\n", errno);
      close(g_btn_fd);
      g_btn_fd = -1;
      return -errno;
    }

  /* Register for button press notification */

  memset(&notify, 0, sizeof(notify));
  notify.bn_press = supported;
  notify.bn_release = 0;
  notify.bn_event.sigev_notify = SIGEV_SIGNAL;
  notify.bn_event.sigev_signo = BUTTON_SIGNAL;

  ret = ioctl(g_btn_fd, BTNIOC_REGISTER, &notify);
  if (ret < 0)
    {
      printf("ERROR: BTNIOC_REGISTER failed: %d\n", errno);
      close(g_btn_fd);
      g_btn_fd = -1;
      return -errno;
    }

  printf("Buttons: IRQ configured on %s\n", devpath);
  return 0;
}

/****************************************************************************
 * Name: button_irq_get_pressed
 *
 * Description:
 *   Get last pressed button index.
 *
 * Returned Value:
 *   Button index (0-7), or BUTTON_INVALID if no button pressed.
 ****************************************************************************/

int button_irq_get_pressed(void)
{
  return g_button_pressed;
}

/****************************************************************************
 * Name: button_irq_clear
 *
 * Description:
 *   Clear pressed button flag.
 ****************************************************************************/

void button_irq_clear(void)
{
  g_button_pressed = BUTTON_INVALID;
}

/****************************************************************************
 * Name: button_irq_close
 *
 * Description:
 *   Close button device.
 ****************************************************************************/

void button_irq_close(void)
{
  if (g_btn_fd >= 0)
    {
      close(g_btn_fd);
      g_btn_fd = -1;
    }
}
