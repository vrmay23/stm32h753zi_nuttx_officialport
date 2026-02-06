# ##############################################################################
# apps/hmi_manager/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with this
# work for additional information regarding copyright ownership.  The ASF
# licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

if(CONFIG_EXAMPLES_HMI_MANAGER)

  # Main application
  set(SRCS app_hmi_manager.c)

  # CAN module
  list(APPEND SRCS can/can_handler.c can/car_can.c can/can_trace.c)

  # IO handler
  list(APPEND SRCS io_handler/button_irq.c)

  # UIUX
  list(APPEND SRCS uiux/fb_handler.c uiux/widgets.c uiux/aux_disp.c
       uiux/notifications.c)

  # Modules
  list(APPEND SRCS modules/rfid.c modules/accel.c)

  # Commands
  list(APPEND SRCS cmd/commands.c)

  # Config
  list(APPEND SRCS config/config_manager.c)

  # Logs
  list(APPEND SRCS logs/logger.c)

  # Include directories
  set(INCDIR ${CMAKE_CURRENT_SOURCE_DIR})
  list(APPEND INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/can)
  list(APPEND INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/io_handler)
  list(APPEND INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/uiux)
  list(APPEND INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/logs)
  list(APPEND INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/modules)
  list(APPEND INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/cmd)
  list(APPEND INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/config)

  nuttx_add_application(
    NAME
    ${CONFIG_EXAMPLES_HMI_MANAGER_PROGNAME}
    PRIORITY
    ${CONFIG_EXAMPLES_HMI_MANAGER_PRIORITY}
    STACKSIZE
    ${CONFIG_EXAMPLES_HMI_MANAGER_STACKSIZE}
    MODULE
    ${CONFIG_EXAMPLES_HMI_MANAGER}
    SRCS
    ${SRCS}
    INCLUDE_DIRECTORIES
    ${INCDIR})

endif()
