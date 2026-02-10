/****************************************************************************
 * apps/hmi_manager/hmi_ctl.c
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
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HMI_SOCKET_PATH "/tmp/hmi_cmd"
#define CMD_BUFFER_SIZE 256

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  struct sockaddr_un addr;
  char cmd_buffer[CMD_BUFFER_SIZE];
  int sock;
  int ret;
  int i;
  int len;

  if (argc < 2)
    {
      printf("Usage: hmi_ctl <command> [args...]\n");
      printf("Try 'hmi_ctl --help' for more information\n");
      return 1;
    }

  printf("DEBUG hmi_ctl: argc=%d\n", argc);
  for (i = 0; i < argc; i++)
    {
      printf("DEBUG hmi_ctl: argv[%d]='%s'\n", i, argv[i]);
    }

  /* Create STREAM socket */
  sock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock < 0)
    {
      printf("ERROR: socket() failed: %d\n", errno);
      return 1;
    }

  printf("DEBUG hmi_ctl: socket created fd=%d\n", sock);

  /* Setup address */
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, HMI_SOCKET_PATH, sizeof(addr.sun_path) - 1);

  printf("DEBUG hmi_ctl: connecting to %s\n", addr.sun_path);

  /* Connect to daemon */
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0)
    {
      printf("ERROR: connect() failed: %d\n", errno);
      printf("Is hmi_manager daemon running?\n");
      close(sock);
      return 1;
    }

  printf("DEBUG hmi_ctl: connected!\n");

  /* Serialize command: "cmd arg1 arg2 arg3" */
  len = 0;
  for (i = 1; i < argc && len < CMD_BUFFER_SIZE - 1; i++)
    {
      if (i > 1)
        {
          cmd_buffer[len++] = ' ';
        }

      strncpy(cmd_buffer + len, argv[i], CMD_BUFFER_SIZE - len - 1);
      len += strlen(argv[i]);
    }

  cmd_buffer[len] = '\0';

  printf("DEBUG hmi_ctl: sending '%s' (len=%d)\n", cmd_buffer, len);

  /* Send command to daemon */
  ret = send(sock, cmd_buffer, len + 1, 0);
  if (ret < 0)
    {
      printf("ERROR: send() failed: %d\n", errno);
      close(sock);
      return 1;
    }

  printf("DEBUG hmi_ctl: sent %d bytes\n", ret);

  close(sock);
  return 0;
}
