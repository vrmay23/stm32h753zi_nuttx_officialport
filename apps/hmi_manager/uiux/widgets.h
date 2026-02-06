/****************************************************************************
 * apps/hmi_manager/uiux/widgets.h
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

#ifndef __APPS_HMI_MANAGER_UIUX_WIDGETS_H
#define __APPS_HMI_MANAGER_UIUX_WIDGETS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Direction states */

#define WIDGET_DIR_FORWARD   0
#define WIDGET_DIR_REVERSE   1
#define WIDGET_DIR_NEUTRAL   2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Create UI widgets */

int widgets_init(void);

/* Update speed display */

void widgets_set_speed(int speed_kmh);

/* Update voltage display (value in 0.1V units) */

void widgets_set_voltage(int voltage_dv);

/* Update current display (value in 0.1A units) */

void widgets_set_current(int current_da);

/* Update direction display */

void widgets_set_direction(int direction);

#endif /* __APPS_HMI_MANAGER_UIUX_WIDGETS_H */
