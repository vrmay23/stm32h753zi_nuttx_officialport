/****************************************************************************
 * apps/hmi_manager/modules/rfid.h
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

#ifndef __APPS_HMI_MANAGER_MODULES_RFID_H
#define __APPS_HMI_MANAGER_MODULES_RFID_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RFID_MODE_BYPASS        0
#define RFID_MODE_REQUEST_CARD  1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize RFID module */

int rfid_init(const char *devpath);

/* Set RFID mode */

int rfid_set_mode(int mode);

/* Read card ID (non-blocking) */

int rfid_read_card(uint8_t *uid, size_t *len);

/* Close RFID module */

void rfid_close(void);

#endif /* __APPS_HMI_MANAGER_MODULES_RFID_H */
