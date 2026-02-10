# ##############################################################################
# apps/hmi_manager/CMakeLists.txt
# ##############################################################################

if(CONFIG_EXAMPLES_HMI_MANAGER)

  # Shared sources
  set(COMMON_SRCS
      can/can_handler.c
      can/car_can.c
      can/can_trace.c
      io_handler/button_irq.c
      uiux/fb_handler.c
      uiux/widgets.c
      uiux/aux_disp.c
      uiux/notifications.c
      modules/rfid.c
      modules/accel.c
      cmd/commands.c
      config/config_manager.c
      logs/logger.c)

  # Include directories
  set(INCDIR
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/can
      ${CMAKE_CURRENT_SOURCE_DIR}/io_handler
      ${CMAKE_CURRENT_SOURCE_DIR}/uiux
      ${CMAKE_CURRENT_SOURCE_DIR}/logs
      ${CMAKE_CURRENT_SOURCE_DIR}/modules
      ${CMAKE_CURRENT_SOURCE_DIR}/cmd
      ${CMAKE_CURRENT_SOURCE_DIR}/config)

  # HMI Manager daemon
  nuttx_add_application(
    NAME
    hmi_manager
    PRIORITY
    ${CONFIG_EXAMPLES_HMI_MANAGER_PRIORITY}
    STACKSIZE
    ${CONFIG_EXAMPLES_HMI_MANAGER_STACKSIZE}
    SRCS
    app_hmi_manager.c
    ${COMMON_SRCS}
    INCLUDE_DIRECTORIES
    ${INCDIR})

  # HMI Control client
  nuttx_add_application(
    NAME
    hmi_ctl
    PRIORITY
    100
    STACKSIZE
    2048
    SRCS
    hmi_ctl.c
    INCLUDE_DIRECTORIES
    ${INCDIR})

endif()
