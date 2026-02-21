/****************************************************************************
 * apps/hmi_manager/io_handler/led_control.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/leds/userled.h>
#include "led_control.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_led_fd = -1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_control_init
 ****************************************************************************/

int led_control_init(const char *devpath)
{
  g_led_fd = open(devpath, O_RDWR);
  if (g_led_fd < 0)
    {
      printf("ERROR: Failed to open %s: %d\n", devpath, errno);
      return -errno;
    }

  /* Turn off all LEDs initially */

  ioctl(g_led_fd, ULEDIOC_SETALL, 0);

  return 0;
}

/****************************************************************************
 * Name: led_control_on
 ****************************************************************************/

void led_control_on(int led)
{
  userled_set_t leds;

  if (g_led_fd < 0)
    {
      return;
    }

  ioctl(g_led_fd, ULEDIOC_GETALL, &leds);
  leds |= (1 << led);
  ioctl(g_led_fd, ULEDIOC_SETALL, leds);
}

/****************************************************************************
 * Name: led_control_off
 ****************************************************************************/

void led_control_off(int led)
{
  userled_set_t leds;

  if (g_led_fd < 0)
    {
      return;
    }

  ioctl(g_led_fd, ULEDIOC_GETALL, &leds);
  leds &= ~(1 << led);
  ioctl(g_led_fd, ULEDIOC_SETALL, leds);
}

/****************************************************************************
 * Name: led_control_set_all
 ****************************************************************************/

void led_control_set_all(uint32_t ledset)
{
  if (g_led_fd < 0)
    {
      return;
    }

  ioctl(g_led_fd, ULEDIOC_SETALL, ledset);
}

/****************************************************************************
 * Name: led_control_close
 ****************************************************************************/

void led_control_close(void)
{
  if (g_led_fd >= 0)
    {
      close(g_led_fd);
      g_led_fd = -1;
    }
}
