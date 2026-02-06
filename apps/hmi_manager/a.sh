#!/bin/bash

BASE="/home/vinicius/Desktop/exemplos_nuttx_can_lvgl/hmi_manager"

cd "$BASE"

# CAN - faltando can_trace
touch can/can_trace.h
touch can/can_trace.c

# IO handler - faltando io_handler
touch io_handler/io_handler.h
touch io_handler/io_handler.c

# Modules - faltando module_manager
touch modules/module_manager.h
touch modules/module_manager.c

# Config - faltando defaults.h
touch config/defaults.h

echo "Arquivos faltantes criados:"
echo "  can/can_trace.h"
echo "  can/can_trace.c"
echo "  io_handler/io_handler.h"
echo "  io_handler/io_handler.c"
echo "  modules/module_manager.h"
echo "  modules/module_manager.c"
echo "  config/defaults.h"

tree "$BASE"
