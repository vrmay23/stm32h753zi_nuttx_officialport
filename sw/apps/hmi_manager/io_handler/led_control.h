#ifndef __APPS_HMI_MANAGER_IO_HANDLER_LED_CONTROL_H
#define __APPS_HMI_MANAGER_IO_HANDLER_LED_CONTROL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED indices (mapped to board LEDs via /dev/userleds)
 * According to stm32_userleds.c:
 *   LED 0 = Green (LD1)
 *   LED 1 = Orange (LD2)
 *   LED 2 = Red (LD3)
 *
 * TODO: Make configurable via Kconfig
 */

#define LED_GREEN               0
#define LED_ORANGE              1
#define LED_RED                 2

#define LED_DC_LINK_ON          LED_GREEN
#define LED_DC_LINK_PRECHARGE   LED_ORANGE

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize LED device */

int led_control_init(const char *devpath);

/* Turn LED on */

void led_control_on(int led);

/* Turn LED off */

void led_control_off(int led);

/* Set all LEDs at once (bitmask) */

void led_control_set_all(uint32_t ledset);

/* Close LED device */

void led_control_close(void);

#endif /* __APPS_HMI_MANAGER_IO_HANDLER_LED_CONTROL_H */
