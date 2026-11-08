/****************************************************************************
 * apps/hmi_manager/logs/logger.h
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

#ifndef __APPS_HMI_MANAGER_LOGS_LOGGER_H
#define __APPS_HMI_MANAGER_LOGS_LOGGER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Log levels */

#define LOG_LEVEL_ERROR   0
#define LOG_LEVEL_WARN    1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_DEBUG   3

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize logger */

void logger_init(int level);

/* Set log level */

void logger_set_level(int level);

/* Enable/disable logging */

void logger_enable(bool enable);

/* Log functions */

void log_error(const char *fmt, ...);
void log_warn(const char *fmt, ...);
void log_info(const char *fmt, ...);
void log_debug(const char *fmt, ...);

#endif /* __APPS_HMI_MANAGER_LOGS_LOGGER_H */
