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
CSRCS += uiux/fb_handler.c
CSRCS += uiux/widgets.c
CSRCS += uiux/aux_disp.c
CSRCS += uiux/notifications.c
CSRCS += modules/rfid.c
CSRCS += modules/accel.c
CSRCS += cmd/commands.c
CSRCS += config/config_manager.c
CSRCS += logs/logger.c

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
