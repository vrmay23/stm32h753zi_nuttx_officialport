/****************************************************************************
 * apps/hmi_manager/uiux/aux_disp.c
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
#include <errno.h>

#include "aux_disp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int aux_disp_init(const char *devpath)
{
  /* TODO: Implement SSD1306 initialization */

  return -ENOSYS;
}

int aux_disp_print(const char *line1, const char *line2)
{
  /* TODO: Implement text display */

  return -ENOSYS;
}

int aux_disp_clear(void)
{
  /* TODO: Implement screen clear */

  return -ENOSYS;
}

void aux_disp_close(void)
{
  /* TODO: Implement cleanup */
}
