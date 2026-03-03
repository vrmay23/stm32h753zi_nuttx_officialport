/****************************************************************************
 * apps/hmi_manager/uiux/screen/screen_manager.h
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

#ifndef __APPS_HMI_MANAGER_UIUX_SCREEN_SCREEN_MANAGER_H
#define __APPS_HMI_MANAGER_UIUX_SCREEN_SCREEN_MANAGER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Animation duration in milliseconds for screen transitions */

#define SCREEN_ANIM_MS  300

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Screen identifiers - order defines logical sequence */

typedef enum
{
  SCREEN_SPLASH     = 0,  /* Static PNG image (flash asset)   */
  SCREEN_DASHBOARD  = 1,  /* Existing LVGL widgets dashboard  */
  SCREEN_BLACK      = 2,  /* Blank black screen               */
  SCREEN_COUNT            /* Sentinel - do not use directly   */
} screen_id_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Phase 1: Create screen skeletons (no LVGL objects with assets).
 * Called from main() after fb_handler_init(), before threads start.
 * Returns 0 on success, negative errno on failure.
 */

int screen_manager_init(void);

/* Phase 2: Attach image assets and build full widget tree.
 * MUST be called from lvgl_thread context ONLY.
 * Called once after lvgl_thread starts.
 */

void screen_manager_setup_assets(void);

/* Load a specific screen unconditionally.
 * Safe to call from main thread or button handler.
 * screen: target screen identifier (SCREEN_SPLASH..SCREEN_BLACK)
 */

void screen_manager_load(screen_id_t screen);

/* Return current active screen identifier */

screen_id_t screen_manager_current(void);

#endif /* __APPS_HMI_MANAGER_UIUX_SCREEN_SCREEN_MANAGER_H */
