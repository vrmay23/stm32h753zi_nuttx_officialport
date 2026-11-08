/****************************************************************************
 * apps/hmi_manager/logs/logger.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "logger.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_log_level = LOG_LEVEL_INFO;
static bool g_log_enabled = true;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: logger_init
 *
 * Description:
 *   Initialize logger with specified level.
 ****************************************************************************/

void logger_init(int level)
{
  g_log_level = level;
  g_log_enabled = true;
}

/****************************************************************************
 * Name: logger_set_level
 *
 * Description:
 *   Set log level.
 ****************************************************************************/

void logger_set_level(int level)
{
  g_log_level = level;
}

/****************************************************************************
 * Name: logger_enable
 *
 * Description:
 *   Enable or disable logging.
 ****************************************************************************/

void logger_enable(bool enable)
{
  g_log_enabled = enable;
}

/****************************************************************************
 * Name: log_error
 *
 * Description:
 *   Log error message.
 ****************************************************************************/

void log_error(const char *fmt, ...)
{
  va_list ap;

  if (!g_log_enabled || g_log_level < LOG_LEVEL_ERROR)
    {
      return;
    }

  printf("[ERROR] ");
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n");
}

/****************************************************************************
 * Name: log_warn
 *
 * Description:
 *   Log warning message.
 ****************************************************************************/

void log_warn(const char *fmt, ...)
{
  va_list ap;

  if (!g_log_enabled || g_log_level < LOG_LEVEL_WARN)
    {
      return;
    }

  printf("[WARN] ");
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n");
}

/****************************************************************************
 * Name: log_info
 *
 * Description:
 *   Log info message.
 ****************************************************************************/

void log_info(const char *fmt, ...)
{
  va_list ap;

  if (!g_log_enabled || g_log_level < LOG_LEVEL_INFO)
    {
      return;
    }

  printf("[INFO] ");
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n");
}

/****************************************************************************
 * Name: log_debug
 *
 * Description:
 *   Log debug message.
 ****************************************************************************/

void log_debug(const char *fmt, ...)
{
  va_list ap;

  if (!g_log_enabled || g_log_level < LOG_LEVEL_DEBUG)
    {
      return;
    }

  printf("[DEBUG] ");
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n");
}
