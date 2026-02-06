/****************************************************************************
 * apps/hmi_manager/can/can_handler.h
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

#ifndef __APPS_HMI_MANAGER_CAN_CAN_HANDLER_H
#define __APPS_HMI_MANAGER_CAN_CAN_HANDLER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/can.h>
#include <sys/socket.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize CAN interface - ifup + socket + bind */

int can_handler_init(const char *ifname);

/* Get CAN socket file descriptor */

int can_handler_get_fd(void);

/* Send CAN frame */

int can_handler_send(uint32_t id, bool extended, const uint8_t *data,
                     uint8_t dlc);

/* Read CAN frame (non-blocking) - returns 0 if no data */

int can_handler_read(struct can_frame *frame);

/* Close CAN socket */

void can_handler_close(void);

#endif /* __APPS_HMI_MANAGER_CAN_CAN_HANDLER_H */
