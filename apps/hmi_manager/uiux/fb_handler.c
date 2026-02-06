/****************************************************************************
 * apps/hmi_manager/uiux/fb_handler.c
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
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <nuttx/video/fb.h>
#include <lvgl/lvgl.h>

#include "fb_handler.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_fb_fd = -1;
static void *g_fb_mem = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fb_flush_cb
 *
 * Description:
 *   LVGL flush callback for framebuffer.
 ****************************************************************************/

static void fb_flush_cb(lv_display_t *disp, const lv_area_t *area,
                        uint8_t *px_map)
{
  struct fb_area_s fb_area;

  if (g_fb_fd < 0)
    {
      lv_display_flush_ready(disp);
      return;
    }

  /* Convert LVGL area to NuttX fb_area */

  fb_area.x = area->x1;
  fb_area.y = area->y1;
  fb_area.w = area->x2 - area->x1 + 1;
  fb_area.h = area->y2 - area->y1 + 1;

  /* Update framebuffer */

  ioctl(g_fb_fd, FBIO_UPDATE, (unsigned long)&fb_area);

  /* Notify LVGL flush complete */

  lv_display_flush_ready(disp);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fb_handler_init
 *
 * Description:
 *   Initialize framebuffer and LVGL display.
 *
 * Input Parameters:
 *   devpath - Framebuffer device path (e.g., "/dev/fb0")
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int fb_handler_init(const char *devpath)
{
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
  lv_display_t *disp;
  int ret;

  if (g_fb_fd >= 0)
    {
      return -EBUSY;
    }

  /* Open framebuffer */

  g_fb_fd = open(devpath, O_RDWR);
  if (g_fb_fd < 0)
    {
      printf("ERROR: open(%s) failed: %d\n", devpath, errno);
      return -errno;
    }

  /* Get video info */

  ret = ioctl(g_fb_fd, FBIOGET_VIDEOINFO, &vinfo);
  if (ret < 0)
    {
      printf("ERROR: FBIOGET_VIDEOINFO failed: %d\n", errno);
      close(g_fb_fd);
      g_fb_fd = -1;
      return -errno;
    }

  /* Get plane info */

  ret = ioctl(g_fb_fd, FBIOGET_PLANEINFO, &pinfo);
  if (ret < 0)
    {
      printf("ERROR: FBIOGET_PLANEINFO failed: %d\n", errno);
      close(g_fb_fd);
      g_fb_fd = -1;
      return -errno;
    }

  printf("FB: %dx%d @ %d bpp\n", vinfo.xres, vinfo.yres, pinfo.bpp);

  /* Map framebuffer memory */

  g_fb_mem = mmap(NULL, pinfo.fblen, PROT_READ | PROT_WRITE,
                  MAP_SHARED, g_fb_fd, 0);
  if (g_fb_mem == MAP_FAILED)
    {
      printf("ERROR: mmap() failed: %d\n", errno);
      close(g_fb_fd);
      g_fb_fd = -1;
      return -errno;
    }

  /* Initialize LVGL */

  lv_init();

  /* Create LVGL display */

  disp = lv_display_create(vinfo.xres, vinfo.yres);
  if (disp == NULL)
    {
      printf("ERROR: lv_display_create() failed\n");
      munmap(g_fb_mem, pinfo.fblen);
      close(g_fb_fd);
      g_fb_fd = -1;
      return -ENOMEM;
    }

  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(disp, fb_flush_cb);
  lv_display_set_buffers(disp, g_fb_mem, NULL, pinfo.fblen,
                          LV_DISPLAY_RENDER_MODE_DIRECT);

  printf("FB: LVGL initialized\n");
  return 0;
}

/****************************************************************************
 * Name: fb_handler_close
 *
 * Description:
 *   Close framebuffer.
 ****************************************************************************/

void fb_handler_close(void)
{
  if (g_fb_mem != NULL && g_fb_mem != MAP_FAILED)
    {
      /* Note: Cannot get fblen here, assume cleanup handled by OS */

      g_fb_mem = NULL;
    }

  if (g_fb_fd >= 0)
    {
      close(g_fb_fd);
      g_fb_fd = -1;
    }
}
