/****************************************************************************
 * apps/hmi_manager/modules/accel.h
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

#ifndef __APPS_HMI_MANAGER_MODULES_ACCEL_H
#define __APPS_HMI_MANAGER_MODULES_ACCEL_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct accel_data_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize accelerometer */

int accel_init(const char *devpath);

/* Read accelerometer data */

int accel_read(struct accel_data_s *data);

/* Close accelerometer */

void accel_close(void);

#endif /* __APPS_HMI_MANAGER_MODULES_ACCEL_H */
