############################################################################
# apps/hmi_manager/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs

# HMI Manager application

PROGNAME  = $(CONFIG_EXAMPLES_HMI_MANAGER_PROGNAME)
PRIORITY  = $(CONFIG_EXAMPLES_HMI_MANAGER_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_HMI_MANAGER_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_HMI_MANAGER)

# Main application

MAINSRC = app_hmi_manager.c

# CAN module sources

CSRCS += can/can_handler.c
CSRCS += can/car_can.c
CSRCS += can/can_trace.c

# IO handler sources

CSRCS += io_handler/button_irq.c

# UIUX sources

CSRCS += uiux/fb_handler.c
CSRCS += uiux/widgets.c
CSRCS += uiux/aux_disp.c
CSRCS += uiux/notifications.c

# Modules sources

CSRCS += modules/rfid.c
CSRCS += modules/accel.c

# Command sources

CSRCS += cmd/commands.c

# Config sources

CSRCS += config/config_manager.c

# Logs sources

CSRCS += logs/logger.c

# Include paths for subdirectories

CFLAGS += -I$(APPDIR)/hmi_manager
CFLAGS += -I$(APPDIR)/hmi_manager/can
CFLAGS += -I$(APPDIR)/hmi_manager/io_handler
CFLAGS += -I$(APPDIR)/hmi_manager/uiux
CFLAGS += -I$(APPDIR)/hmi_manager/logs
CFLAGS += -I$(APPDIR)/hmi_manager/modules
CFLAGS += -I$(APPDIR)/hmi_manager/cmd
CFLAGS += -I$(APPDIR)/hmi_manager/config

include $(APPDIR)/Application.mk
