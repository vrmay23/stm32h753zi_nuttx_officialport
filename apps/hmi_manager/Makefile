############################################################################
# apps/hmi_manager/Makefile
############################################################################

include $(APPDIR)/Make.defs

# HMI Manager - Multi-binary application

# Daemon

PROGNAME  = hmi_manager
PRIORITY  = $(CONFIG_EXAMPLES_HMI_MANAGER_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_HMI_MANAGER_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_HMI_MANAGER)

MAINSRC = app_hmi_manager.c

# Client control tool

PROGNAME += hmi_ctl
PRIORITY += 100
STACKSIZE += 2048
MAINSRC += hmi_ctl.c

# Shared sources (used by both programs)

CSRCS += can/can_handler.c
CSRCS += can/car_can.c
CSRCS += can/can_trace.c
CSRCS += io_handler/button_irq.c
CSRCS += io_handler/led_control.c
CSRCS += uiux/fb_handler.c
CSRCS += uiux/widgets.c
CSRCS += uiux/screen/screen_manager.c
CSRCS += uiux/aux_disp.c
CSRCS += uiux/notifications.c
CSRCS += modules/rfid.c
CSRCS += modules/accel.c
CSRCS += cmd/commands.c
CSRCS += config/config_manager.c
CSRCS += logs/logger.c

# Splash screen asset (selected via menuconfig)

ifdef CONFIG_HMI_SPLASH_BIFROSTOX_FLAT
ifeq ($(wildcard uiux/assets/img_bifrostox_flat.c),)
$(warning img_bifrostox_flat.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_bifrostox_flat.c
endif

ifdef CONFIG_HMI_SPLASH_BIFROSTOX_NORDIC
ifeq ($(wildcard uiux/assets/img_bifrostox_nordic.c),)
$(warning img_bifrostox_nordic.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_bifrostox_nordic.c
endif

# Driver screen theme asset (selected via menuconfig)

ifdef CONFIG_HMI_THEME_BIFROSTOX_FLAT
ifeq ($(wildcard uiux/assets/img_bifrostox_flat.c),)
$(warning img_bifrostox_flat.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_bifrostox_flat.c
endif

ifdef CONFIG_HMI_THEME_BIFROSTOX_NORDIC
ifeq ($(wildcard uiux/assets/img_bifrostox_nordic.c),)
$(warning img_bifrostox_nordic.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_bifrostox_nordic.c
endif

ifdef CONFIG_HMI_THEME_MAIN_CYBERPUNK
ifeq ($(wildcard uiux/assets/img_main_cyberpunk.c),)
$(warning img_main_cyberpunk.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_main_cyberpunk.c
endif

ifdef CONFIG_HMI_THEME_MAIN_MODERN
ifeq ($(wildcard uiux/assets/img_main_modern.c),)
$(warning img_main_modern.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_main_modern.c
endif

ifdef CONFIG_HMI_THEME_MAIN_RETRO
ifeq ($(wildcard uiux/assets/img_main_retro.c),)
$(warning img_main_retro.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_main_retro.c
endif

ifdef CONFIG_HMI_THEME_MAIN_RETRO_T3
ifeq ($(wildcard uiux/assets/img_main_retro_t3.c),)
$(warning img_main_retro.c not found! Run png2code.py)
endif
CSRCS += uiux/assets/img_main_retro_t3.c
endif

# Include paths

CFLAGS += -I$(APPDIR)/hmi_manager
CFLAGS += -I$(APPDIR)/hmi_manager/can
CFLAGS += -I$(APPDIR)/hmi_manager/io_handler
CFLAGS += -I$(APPDIR)/hmi_manager/uiux
CFLAGS += -I$(APPDIR)/hmi_manager/logs
CFLAGS += -I$(APPDIR)/hmi_manager/modules
CFLAGS += -I$(APPDIR)/hmi_manager/cmd
CFLAGS += -I$(APPDIR)/hmi_manager/config

include $(APPDIR)/Application.mk
