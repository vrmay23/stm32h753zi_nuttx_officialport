/****************************************************************************
 * apps/hmi_manager/io_handler/button_irq.h
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

#ifndef __APPS_HMI_MANAGER_IO_HANDLER_BUTTON_IRQ_H
#define __APPS_HMI_MANAGER_IO_HANDLER_BUTTON_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUTTON_INVALID  -1

/* Button indices (hardware mapping via /dev/buttons)
 * TODO: Make configurable via Kconfig
 */

#define BUTTON_DC_LINK_TOGGLE   0
#define BUTTON_FORWARD          1
#define BUTTON_REVERSE          2
#define BUTTON_NEUTRAL          3
#define BUTTON_RESET_FAULTS     4
#define BUTTON_PEDAL_MODE       5

/* BUTTON 6-10: Reserved for future use */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize button interrupt handler */

int button_irq_init(const char *devpath);

/* Get last pressed button index (non-blocking) */

int button_irq_get_pressed(void);

/* Clear pressed button flag */

void button_irq_clear(void);

/* Close button device */

void button_irq_close(void);

#endif /* __APPS_HMI_MANAGER_IO_HANDLER_BUTTON_IRQ_H */
