/****************************************************************************
 * apps/hmi_manager/uiux/notifications.h
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

#ifndef __APPS_HMI_MANAGER_UIUX_NOTIFICATIONS_H
#define __APPS_HMI_MANAGER_UIUX_NOTIFICATIONS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NOTIF_TYPE_INFO     0
#define NOTIF_TYPE_WARNING  1
#define NOTIF_TYPE_ERROR    2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize notification system */

int notifications_init(void);

/* Show notification popup */

int notifications_show(int type, const char *message, int timeout_ms);

/* Hide current notification */

void notifications_hide(void);

#endif /* __APPS_HMI_MANAGER_UIUX_NOTIFICATIONS_H */
