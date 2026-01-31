/****************************************************************************
 * apps/examples/bev_cluster/bev_cluster_main.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <nuttx/video/fb.h>
#include <lvgl/lvgl.h>

static int fb_fd;
static void *fb_mem;
static lv_obj_t *speed_label;
static lv_obj_t *battery_bar;
static lv_obj_t *battery_label;
static lv_obj_t *battery_icon;
static int current_speed = 0;
static int speed_dir = 2;
static int current_battery = 100;
static int battery_dir = -1;

static void fb_flush(lv_display_t *disp, const lv_area_t *area, 
                     uint8_t *px_map)
{
  struct fb_area_s fb_area;
  fb_area.x = area->x1;
  fb_area.y = area->y1;
  fb_area.w = area->x2 - area->x1 + 1;
  fb_area.h = area->y2 - area->y1 + 1;
  ioctl(fb_fd, FBIO_UPDATE, (unsigned long)&fb_area);
  lv_display_flush_ready(disp);
}

static void update_speed(lv_timer_t *timer)
{
  char buf[16];
  printf("speed\n");
  current_speed += speed_dir;
  if (current_speed >= 120) speed_dir = -2;
  if (current_speed <= 0) speed_dir = 2;
  snprintf(buf, sizeof(buf), "%d", current_speed);
  lv_label_set_text(speed_label, buf);
}

static void update_battery(lv_timer_t *timer)
{
  char buf[16];
  printf("battery\n");
  lv_color_t color;
  
  current_battery += battery_dir;
  if (current_battery >= 100) battery_dir = -1;
  if (current_battery <= 10) battery_dir = 1;
  
  if (current_battery > 50)
    {
      color = lv_color_hex(0x00FF00);
      lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_FULL);
    }
  else if (current_battery > 20)
    {
      color = lv_color_hex(0xFFAA00);
      lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_2);
    }
  else
    {
      color = lv_color_hex(0xFF0000);
      lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_EMPTY);
    }
  
  lv_obj_set_style_bg_color(battery_bar, color, LV_PART_INDICATOR);
  lv_bar_set_value(battery_bar, current_battery, LV_ANIM_ON);
  snprintf(buf, sizeof(buf), "%d%%", current_battery);
  lv_label_set_text(battery_label, buf);
}

int main(int argc, char *argv[])
{
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
  lv_obj_t *screen, *speed_cont, *battery_cont, *speed_unit;
  
  fb_fd = open("/dev/fb0", O_RDWR);
  if (fb_fd < 0) return 1;
  
  ioctl(fb_fd, FBIOGET_VIDEOINFO, &vinfo);
  ioctl(fb_fd, FBIOGET_PLANEINFO, &pinfo);
  
  fb_mem = mmap(NULL, pinfo.fblen, PROT_READ | PROT_WRITE,
                MAP_SHARED, fb_fd, 0);
  if (fb_mem == MAP_FAILED) return 1;
  
  lv_init();
  
  lv_display_t *disp = lv_display_create(vinfo.xres, vinfo.yres);
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(disp, fb_flush);
  lv_display_set_buffers(disp, fb_mem, NULL, pinfo.fblen,
                          LV_DISPLAY_RENDER_MODE_DIRECT);
  
  screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
  
  /* Velocímetro */
  speed_cont = lv_obj_create(screen);
  lv_obj_set_size(speed_cont, 300, 160);
  lv_obj_align(speed_cont, LV_ALIGN_CENTER, 0, -40);
  lv_obj_set_style_bg_color(speed_cont, lv_color_hex(0x1a1a1a), 0);
  lv_obj_set_style_border_color(speed_cont, lv_color_hex(0x00ff00), 0);
  lv_obj_set_style_border_width(speed_cont, 3, 0);
  lv_obj_set_style_radius(speed_cont, 15, 0);
  
  speed_label = lv_label_create(speed_cont);
  lv_label_set_text(speed_label, "0");
  lv_obj_set_style_text_color(speed_label, lv_color_hex(0x00ff00), 0);
  lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_48, 0);
  lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -10);
  
  speed_unit = lv_label_create(speed_cont);
  lv_label_set_text(speed_unit, "km/h");
  lv_obj_set_style_text_color(speed_unit, lv_color_hex(0x00ff00), 0);
  lv_obj_set_style_text_font(speed_unit, &lv_font_montserrat_20, 0);
  lv_obj_align(speed_unit, LV_ALIGN_CENTER, 0, 30);
  
  /* Bateria */
  battery_cont = lv_obj_create(screen);
  lv_obj_set_size(battery_cont, 360, 60);
  lv_obj_align(battery_cont, LV_ALIGN_BOTTOM_MID, 0, -15);
  lv_obj_set_style_bg_color(battery_cont, lv_color_hex(0x1a1a1a), 0);
  lv_obj_set_style_border_color(battery_cont, lv_color_hex(0xffaa00), 0);
  lv_obj_set_style_border_width(battery_cont, 2, 0);
  lv_obj_set_style_radius(battery_cont, 10, 0);
  
  battery_icon = lv_label_create(battery_cont);
  lv_label_set_text(battery_icon, LV_SYMBOL_BATTERY_FULL);
  lv_obj_set_style_text_color(battery_icon, lv_color_hex(0xffaa00), 0);
  lv_obj_set_style_text_font(battery_icon, &lv_font_montserrat_28, 0);
  lv_obj_align(battery_icon, LV_ALIGN_LEFT_MID, 10, 0);
  
  battery_bar = lv_bar_create(battery_cont);
  lv_obj_set_size(battery_bar, 200, 35);
  lv_obj_align(battery_bar, LV_ALIGN_CENTER, 5, 0);
  lv_bar_set_range(battery_bar, 0, 100);
  lv_bar_set_value(battery_bar, 100, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0x333333), 0);
  lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0x00ff00), 
                             LV_PART_INDICATOR);
  
  battery_label = lv_label_create(battery_cont);
  lv_label_set_text(battery_label, "100%");
  lv_obj_set_style_text_color(battery_label, lv_color_hex(0xffffff), 0);
  lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_20, 0);
  lv_obj_align(battery_label, LV_ALIGN_RIGHT_MID, -10, 0);
  
  /* Create LVGL timers */
  lv_timer_create(update_speed, 100, NULL);
  lv_timer_create(update_battery, 1000, NULL);
  
  printf("BEV Cluster running...\n");
  
  /* LVGL main loop */
  
  uint32_t n = 0;
  
  while (1)
    {
      lv_timer_handler();
      usleep(5000);
      
      /* printf("loop %ld \t", n); 
      *
      * printf("\n");
      * n++ 
      */;
      
      lv_tick_inc(5);lv_tick_inc(5);
    }
  
  return 0;
}
