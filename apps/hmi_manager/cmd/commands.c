/****************************************************************************
 * apps/hmi_manager/cmd/commands.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "commands.h"
#include "can/can_trace.h"
#include "logs/logger.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_log
 *
 * Description:
 *   Handle log commands.
 ****************************************************************************/

static int cmd_log(int argc, char *argv[])
{
  if (argc < 3)
    {
      printf("Usage: hmi_manager log --show|--hide\n");
      return -EINVAL;
    }

  if (strcmp(argv[2], "--show") == 0)
    {
      logger_enable(true);
      printf("Log enabled\n");
    }
  else if (strcmp(argv[2], "--hide") == 0)
    {
      logger_enable(false);
      printf("Log disabled\n");
    }
  else
    {
      printf("Unknown log command: %s\n", argv[2]);
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: cmd_can
 *
 * Description:
 *   Handle CAN commands.
 ****************************************************************************/

static int cmd_can(int argc, char *argv[])
{
  if (argc < 3)
    {
      printf("Usage:\n");
      printf("  hmi_manager can --trace show|hide\n");
      printf("  hmi_manager can --trace filter <id>\n");
      return -EINVAL;
    }

  /* can --trace ... */

  if (strcmp(argv[2], "--trace") == 0)
    {
      if (argc < 4)
        {
          printf("Usage: hmi_manager can --trace show|hide|filter\n");
          return -EINVAL;
        }

      if (strcmp(argv[3], "show") == 0)
        {
          can_trace_enable(true);
          printf("CAN trace enabled\n");
        }
      else if (strcmp(argv[3], "hide") == 0)
        {
          can_trace_enable(false);
          printf("CAN trace disabled\n");
        }
      else if (strcmp(argv[3], "filter") == 0)
        {
          if (argc < 5)
            {
              printf("Usage: hmi_manager can --trace filter <id>\n");
              printf("  <id> can be hex (0x123) or decimal (291)\n");
              printf("  Use 0 to disable filter (show all)\n");
              return -EINVAL;
            }

          uint32_t id;
          char *endptr;

          /* Parse ID (supports hex 0x... or decimal) */

          id = strtoul(argv[4], &endptr, 0);
          if (*endptr != '\0')
            {
              printf("Invalid ID format: %s\n", argv[4]);
              return -EINVAL;
            }

          can_trace_set_filter(id);
          if (id == 0)
            {
              printf("CAN trace filter disabled (all IDs)\n");
            }
          else
            {
              printf("CAN trace filter set to 0x%08lX\n",
                     (unsigned long)id);
            }
        }
      else
        {
          printf("Unknown trace command: %s\n", argv[3]);
          return -EINVAL;
        }
    }
  else
    {
      printf("Unknown CAN command: %s\n", argv[2]);
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: cmd_rfid
 *
 * Description:
 *   Handle RFID commands (placeholder).
 ****************************************************************************/

static int cmd_rfid(int argc, char *argv[])
{
  printf("RFID commands not implemented yet\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: cmd_app
 *
 * Description:
 *   Handle app commands (placeholder).
 ****************************************************************************/

static int cmd_app(int argc, char *argv[])
{
  printf("App commands not implemented yet\n");
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: commands_print_usage
 *
 * Description:
 *   Print command usage help.
 ****************************************************************************/

void commands_print_usage(void)
{
  printf("\nHMI Manager Commands:\n\n");
  printf("Log commands:\n");
  printf("  hmi_manager log --show\n");
  printf("  hmi_manager log --hide\n");
  printf("\n");
  printf("CAN commands:\n");
  printf("  hmi_manager can --trace show\n");
  printf("  hmi_manager can --trace hide\n");
  printf("  hmi_manager can --trace filter <id>\n");
  printf("    Examples:\n");
  printf("      hmi_manager can --trace filter 0x18FEF100\n");
  printf("      hmi_manager can --trace filter 0\n");
  printf("\n");
  printf("RFID commands (TODO):\n");
  printf("  hmi_manager rfid --mode bypass\n");
  printf("  hmi_manager rfid --mode request_card\n");
  printf("\n");
  printf("App commands (TODO):\n");
  printf("  hmi_manager app --mode dev\n");
  printf("  hmi_manager app --mode eol\n");
  printf("  hmi_manager app --reload last_status\n");
  printf("  hmi_manager app --reload default\n");
  printf("\n");
}

/****************************************************************************
 * Name: commands_parse
 *
 * Description:
 *   Parse and execute command.
 ****************************************************************************/

int commands_parse(int argc, char *argv[])
{
  if (argc < 2)
    {
      commands_print_usage();
      return -EINVAL;
    }

  /* Route to subcommand handlers */

  if (strcmp(argv[1], "log") == 0)
    {
      return cmd_log(argc, argv);
    }
  else if (strcmp(argv[1], "can") == 0)
    {
      return cmd_can(argc, argv);
    }
  else if (strcmp(argv[1], "rfid") == 0)
    {
      return cmd_rfid(argc, argv);
    }
  else if (strcmp(argv[1], "app") == 0)
    {
      return cmd_app(argc, argv);
    }
  else if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)
    {
      commands_print_usage();
      return 0;
    }
  else
    {
      printf("Unknown command: %s\n", argv[1]);
      printf("Try 'hmi_manager --help' for usage\n");
      return -EINVAL;
    }
}
