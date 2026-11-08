/****************************************************************************
 * apps/hmi_manager/can/can_handler.c
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
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <nuttx/can.h>
#include <netutils/netlib.h>

#include "can_handler.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_can_fd = -1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_handler_init
 *
 * Description:
 *   Initialize CAN interface - bring up, create socket, bind.
 *
 * Input Parameters:
 *   ifname - Interface name (e.g., "can0")
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int can_handler_init(const char *ifname)
{
  struct sockaddr_can addr;
  struct ifreq ifr;
  int ret;

  if (g_can_fd >= 0)
    {
      return -EBUSY;
    }

  /* Bring interface up */

  ret = netlib_ifup(ifname);
  if (ret < 0)
    {
      printf("ERROR: netlib_ifup(%s) failed: %d\n", ifname, ret);
      return ret;
    }

  /* Create CAN socket */

  g_can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (g_can_fd < 0)
    {
      printf("ERROR: socket(PF_CAN) failed: %d\n", errno);
      return -errno;
    }

  /* Get interface index */

  strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (ifr.ifr_ifindex == 0)
    {
      printf("ERROR: if_nametoindex(%s) failed\n", ifname);
      close(g_can_fd);
      g_can_fd = -1;
      return -ENODEV;
    }

  /* Bind socket to interface */

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  ret = bind(g_can_fd, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0)
    {
      printf("ERROR: bind() failed: %d\n", errno);
      close(g_can_fd);
      g_can_fd = -1;
      return -errno;
    }

  printf("CAN: Initialized on %s (fd=%d)\n", ifname, g_can_fd);
  return 0;
}

/****************************************************************************
 * Name: can_handler_get_fd
 *
 * Description:
 *   Get CAN socket file descriptor for select().
 *
 * Returned Value:
 *   File descriptor, or -1 if not initialized.
 ****************************************************************************/

int can_handler_get_fd(void)
{
  return g_can_fd;
}

/****************************************************************************
 * Name: can_handler_send
 *
 * Description:
 *   Send CAN frame.
 *
 * Input Parameters:
 *   id       - CAN ID (11-bit or 29-bit)
 *   extended - true for 29-bit extended ID
 *   data     - Payload (up to 8 bytes)
 *   dlc      - Data length (0-8)
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int can_handler_send(uint32_t id, bool extended, const uint8_t *data,
                     uint8_t dlc)
{
  struct can_frame frame;
  ssize_t nbytes;

  if (g_can_fd < 0)
    {
      return -EBADF;
    }

  if (dlc > 8)
    {
      return -EINVAL;
    }

  memset(&frame, 0, sizeof(frame));

  frame.can_id = id;
  if (extended)
    {
      frame.can_id |= CAN_EFF_FLAG;
    }

  frame.can_dlc = dlc;

  if (data != NULL && dlc > 0)
    {
      memcpy(frame.data, data, dlc);
    }

  nbytes = write(g_can_fd, &frame, sizeof(frame));
  if (nbytes != sizeof(frame))
    {
      printf("ERROR: write() failed: %d\n", errno);
      return -errno;
    }

  return 0;
}

/****************************************************************************
 * Name: can_handler_read
 *
 * Description:
 *   Read CAN frame (non-blocking).
 *
 * Input Parameters:
 *   frame - Pointer to can_frame structure to fill.
 *
 * Returned Value:
 *   0 if frame read successfully, -EAGAIN if no data, negative errno
 *   on error.
 ****************************************************************************/

int can_handler_read(struct can_frame *frame)
{
  ssize_t nbytes;

  if (g_can_fd < 0)
    {
      return -EBADF;
    }

  if (frame == NULL)
    {
      return -EINVAL;
    }

  nbytes = read(g_can_fd, frame, sizeof(struct can_frame));
  if (nbytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          return -EAGAIN;
        }

      return -errno;
    }

  if (nbytes != sizeof(struct can_frame))
    {
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: can_handler_close
 *
 * Description:
 *   Close CAN socket.
 ****************************************************************************/

void can_handler_close(void)
{
  if (g_can_fd >= 0)
    {
      close(g_can_fd);
      g_can_fd = -1;
    }
}
