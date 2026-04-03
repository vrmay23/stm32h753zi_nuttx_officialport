/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/drivers/driver_modules/
 *   stm32_ft6336.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version
 * 2.0 (the "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 ****************************************************************************/

/* Board-level glue between the NuttX FT5x06 driver and the STM32H753ZI
 * hardware.  The FT6336U is register-compatible with the FT5x06 family
 * and reuses drivers/input/ft5x06.c without modification.
 *
 * Responsibilities of this file:
 *   - Parse the INT GPIO pin string from Kconfig
 *   - Configure the EXTI interrupt via stm32_gpiosetevent()
 *   - Populate struct ft5x06_config_s (attach / enable / clear / address)
 *   - Call ft5x06_register() to bind everything together
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/ft5x06.h>

#include "stm32_gpio.h"
#include "../../nucleo-h753zi.h"

#ifdef CONFIG_NUCLEO_H753ZI_FT6336_ENABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO configuration for the INT pin: floating input, EXTI capable.
 * The actual port/pin bits are filled at runtime from the parsed pin
 * string; only the mode/pull flags are fixed here.
 */

#define FT6336_INT_FLAGS  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI)

/* GPIO configuration for the RST pin: push-pull output, active-low.
 * Assert low to reset the chip, deassert high for normal operation.
 */

#define FT6336_RST_FLAGS  (GPIO_OUTPUT | GPIO_PUSHPULL | \
                           GPIO_SPEED_2MHz | GPIO_OUTPUT_SET)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container that extends ft5x06_config_s with the resolved GPIO cfg word */

struct ft6336_board_s
{
  struct ft5x06_config_s config; /* Driver config for ft5x06_register */
  uint32_t               intcfg; /* Resolved STM32 GPIO config word */
  uint32_t               rstcfg; /* Resolved RST GPIO config word   */
  xcpt_t                 isr;    /* ISR saved by attach for enable */
  FAR void              *arg;    /* Argument saved by attach       */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  ft6336_attach(FAR const struct ft5x06_config_s *config,
                          xcpt_t isr, FAR void *arg);
static void ft6336_enable(FAR const struct ft5x06_config_s *config,
                          bool enable);
static void ft6336_clear(FAR const struct ft5x06_config_s *config);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ft6336_board_s g_ft6336_board =
{
  .config =
    {
      .address   = CONFIG_NUCLEO_H753ZI_FT6336_I2C_ADDR,
      .frequency = CONFIG_NUCLEO_H753ZI_FT6336_I2C_FREQUENCY,
      .attach    = ft6336_attach,
      .enable    = ft6336_enable,
      .clear     = ft6336_clear,
    },
  .intcfg = 0,   /* Filled by stm32_ft6336_initialize() */
  .rstcfg = 0,   /* Filled by stm32_ft6336_initialize() */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft6336_attach
 *
 * Description:
 *   Attach the FT6336U interrupt handler.  Called once by ft5x06_register.
 *
 * Input Parameters:
 *   config - Board configuration (unused beyond container access)
 *   isr    - Interrupt service routine provided by the upper driver
 *   arg    - Argument passed back to isr on interrupt
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int ft6336_attach(FAR const struct ft5x06_config_s *config,
                         xcpt_t isr, FAR void *arg)
{
  /* ft5x06_register() calls attach with the original config pointer
   * (&g_ft6336_board.config).  For consistency with enable/clear and
   * to avoid fragile pointer arithmetic, use the global directly.
   */

  UNUSED(config);

  /* Save the ISR and argument so ft6336_enable() can reuse them. */

  g_ft6336_board.isr = isr;
  g_ft6336_board.arg = arg;

  /* GPIO already configured by stm32_configgpio() in
   * stm32_ft6336_initialize() before ft5x06_register() is called.
   * EXTI will be armed by ft6336_enable(true) — nothing to do here.
   */

  return OK;
}

/****************************************************************************
 * Name: ft6336_enable
 *
 * Description:
 *   Enable or disable the FT6336U EXTI interrupt.
 *
 * Input Parameters:
 *   config - Board configuration
 *   enable - true = enable interrupt, false = disable
 *
 ****************************************************************************/

static void ft6336_enable(FAR const struct ft5x06_config_s *config,
                          bool enable)
{
  /* ft5x06_register() calls enable/clear via priv->config, which is a
   * copy of the original ft5x06_config_s made by memcpy.  Casting that
   * pointer back to ft6336_board_s* would be OOB.  Use the global
   * directly — there is only one instance of this driver.
   */

  UNUSED(config);

  if (enable)
    {
      /* Arm EXTI falling-edge with the ISR saved during attach. */

      stm32_gpiosetevent(g_ft6336_board.intcfg,
                         false, true, false,
                         g_ft6336_board.isr,
                         g_ft6336_board.arg);
    }
  else
    {
      /* Disarm EXTI — clear all edge detection and handler. */

      stm32_gpiosetevent(g_ft6336_board.intcfg,
                         false, false, false,
                         NULL, NULL);
    }
}

/****************************************************************************
 * Name: ft6336_clear
 *
 * Description:
 *   Clear any pending FT6336U interrupt.  On STM32H7 the EXTI pending bit
 *   is cleared automatically by the hardware after the ISR returns, so
 *   this is a no-op.
 *
 * Input Parameters:
 *   config - Board configuration (unused)
 *
 ****************************************************************************/

static void ft6336_clear(FAR const struct ft5x06_config_s *config)
{
  /* STM32H7 EXTI clears the pending bit in hardware. */

  UNUSED(config);
}

/****************************************************************************
 * Name: parse_gpio_pin
 *
 * Description:
 *   Convert a Kconfig pin string (e.g., "PG8") into an STM32 GPIO
 *   configuration word suitable for stm32_configgpio() /
 *   stm32_gpiosetevent().
 *
 *   Accepted format: P<port><pin>  where
 *     port  = A-H  (case-insensitive)
 *     pin   = 0-15
 *
 * Input Parameters:
 *   pinstr - NUL-terminated pin string from Kconfig
 *   flags  - Additional GPIO flags to OR in (e.g. GPIO_INPUT | GPIO_FLOAT)
 *   outcfg - Receives the resolved configuration word
 *
 * Returned Value:
 *   OK on success; -EINVAL if the string cannot be parsed.
 *
 ****************************************************************************/

static int parse_gpio_pin(FAR const char *pinstr,
                          uint32_t flags,
                          FAR uint32_t *outcfg)
{
  char   port_ch;
  int    pin_num;
  uint32_t port_bits;
  uint32_t pin_bits;

  DEBUGASSERT(pinstr != NULL && outcfg != NULL);

  /* Expect leading 'P' or 'p' */

  if (pinstr[0] != 'P' && pinstr[0] != 'p')
    {
      return -EINVAL;
    }

  port_ch = pinstr[1];
  if (port_ch >= 'a' && port_ch <= 'h')
    {
      port_ch -= ('a' - 'A');
    }

  if (port_ch < 'A' || port_ch > 'H')
    {
      return -EINVAL;
    }

  pin_num = 0;

  {
    const char *p = &pinstr[2];
    if (*p < '0' || *p > '9')
      {
        return -EINVAL;
      }

    while (*p >= '0' && *p <= '9')
      {
        pin_num = pin_num * 10 + (*p - '0');
        p++;
      }
  }

  if (pin_num > 15)
    {
      return -EINVAL;
    }

  /* Build the STM32 GPIO port bits.
   * GPIO_PORT_SHIFT encodes the port as a number (0=A..7=H).
   */

  port_bits = (uint32_t)(port_ch - 'A') << GPIO_PORT_SHIFT;

  /* Build the STM32 GPIO pin bits.
   * GPIO_PIN_SHIFT encodes the pin NUMBER (0-15), not a bitmask.
   * GPIO_PIN0 = (0 << GPIO_PIN_SHIFT), GPIO_PIN15 = (15 << GPIO_PIN_SHIFT)
   */

  pin_bits = (uint32_t)pin_num << GPIO_PIN_SHIFT;

  *outcfg = flags | port_bits | pin_bits;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft6336_reset
 *
 * Description:
 *   Perform a hardware reset of the FT6336U.
 *   RST is active-low: pull low for 10ms, then high for 300ms to allow
 *   the chip to complete its internal initialization before I2C access.
 *
 ****************************************************************************/

static void ft6336_reset(void)
{
  /* Assert reset — pull RST low */

  stm32_gpiowrite(g_ft6336_board.rstcfg, false);
  usleep(10 * 1000);                    /* Hold reset for 10 ms */

  /* Deassert reset — pull RST high */

  stm32_gpiowrite(g_ft6336_board.rstcfg, true);
  usleep(300 * 1000);                   /* Wait 300 ms for chip boot */
}

/****************************************************************************
 * Name: stm32_ft6336_initialize
 *
 * Description:
 *   Initialize and register the FT6336U capacitive touch controller.
 *
 *   This function:
 *     1. Resolves the INT GPIO pin string from Kconfig into a hardware cfg
 *     2. Configures the GPIO as a floating EXTI input
 *     3. Registers an I2C device entry for tracking/debugging
 *     4. Calls ft5x06_register() to bind the driver to the hardware
 *
 * Input Parameters:
 *   None — all configuration comes from Kconfig symbols.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_ft6336_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  /* Resolve INT pin string -> STM32 GPIO config word */

  ret = parse_gpio_pin(CONFIG_NUCLEO_H753ZI_FT6336_INT_PIN,
                       FT6336_INT_FLAGS,
                       &g_ft6336_board.intcfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: [FT6336] Invalid INT pin \"%s\": %d\n",
             CONFIG_NUCLEO_H753ZI_FT6336_INT_PIN, ret);
      return ret;
    }

  /* Configure the physical INT GPIO pin */

  stm32_configgpio(g_ft6336_board.intcfg);

  syslog(LOG_INFO,
         "[FT6336] INT pin %s configured\n",
         CONFIG_NUCLEO_H753ZI_FT6336_INT_PIN);

  /* Resolve RST pin string -> STM32 GPIO config word */

  ret = parse_gpio_pin(CONFIG_NUCLEO_H753ZI_FT6336_RST_PIN,
                       FT6336_RST_FLAGS,
                       &g_ft6336_board.rstcfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: [FT6336] Invalid RST pin \"%s\": %d\n",
             CONFIG_NUCLEO_H753ZI_FT6336_RST_PIN, ret);
      return ret;
    }

  /* Configure the physical RST GPIO pin and perform hardware reset */

  stm32_configgpio(g_ft6336_board.rstcfg);
  ft6336_reset();

  syslog(LOG_INFO,
         "[FT6336] RST pin %s configured, chip reset done\n",
         CONFIG_NUCLEO_H753ZI_FT6336_RST_PIN);

  /* Get the I2C master for the configured bus */

  i2c = stm32_i2c_get_master(CONFIG_NUCLEO_H753ZI_FT6336_I2C_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: [FT6336] I2C%d not available\n",
             CONFIG_NUCLEO_H753ZI_FT6336_I2C_BUS);
      return -ENODEV;
    }

  /* Register device for tracking/debugging */

  stm32_i2c_register_device(CONFIG_NUCLEO_H753ZI_FT6336_I2C_BUS,
                             CONFIG_NUCLEO_H753ZI_FT6336_I2C_ADDR,
                             CONFIG_NUCLEO_H753ZI_FT6336_I2C_FREQUENCY,
                             "ft6336u");

  /* Register the FT5x06-compatible driver */

  ret = ft5x06_register(i2c,
                        &g_ft6336_board.config,
                        CONFIG_NUCLEO_H753ZI_FT6336_DEVNO);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: [FT6336] ft5x06_register() failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO,
         "[FT6336] Touch controller registered as /dev/input%d\n",
         CONFIG_NUCLEO_H753ZI_FT6336_DEVNO);

  return OK;
}

#endif /* CONFIG_NUCLEO_H753ZI_FT6336_ENABLE */
