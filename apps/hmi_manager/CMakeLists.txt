##############################################################################
# apps/hmi_manager/CMakeLists.txt
##############################################################################

if(CONFIG_EXAMPLES_HMI_MANAGER)

  # Shared sources (compiled into both hmi_manager and hmi_ctl via COMMON_SRCS)
  set(COMMON_SRCS
      can/can_handler.c
      can/car_can.c
      can/can_trace.c
      io_handler/button_irq.c
      io_handler/led_control.c
      uiux/fb_handler.c
      uiux/widgets.c
      uiux/screen/screen_manager.c
      uiux/aux_disp.c
      uiux/notifications.c
      modules/rfid.c
      modules/accel.c
      cmd/commands.c
      config/config_manager.c
      logs/logger.c)

  # Splash screen asset (selected via menuconfig)
  if(CONFIG_HMI_SPLASH_BIFROSTOX_FLAT)
    list(APPEND COMMON_SRCS uiux/assets/img_bifrostox_flat.c)
  endif()

  if(CONFIG_HMI_SPLASH_BIFROSTOX_NORDIC)
    list(APPEND COMMON_SRCS uiux/assets/img_bifrostox_nordic.c)
  endif()

  # Theme asset (selected via menuconfig)
  if(CONFIG_HMI_THEME_BIFROSTOX_FLAT)
    list(APPEND COMMON_SRCS uiux/assets/img_bifrostox_flat.c)
  endif()

  if(CONFIG_HMI_THEME_BIFROSTOX_NORDIC)
    list(APPEND COMMON_SRCS uiux/assets/img_bifrostox_nordic.c)
  endif()

  if(CONFIG_HMI_THEME_MAIN_CYBERPUNK)
    list(APPEND COMMON_SRCS uiux/assets/img_main_cyberpunk.c)
  endif()

  if(CONFIG_HMI_THEME_MAIN_MODERN)
    list(APPEND COMMON_SRCS uiux/assets/img_main_modern.c)
  endif()

  if(CONFIG_HMI_THEME_MAIN_RETRO)
    list(APPEND COMMON_SRCS uiux/assets/img_main_retro.c)
  endif()

  if(CONFIG_HMI_THEME_MAIN_RETRO_T3)
    list(APPEND COMMON_SRCS uiux/assets/img_main_retro_t3.c)
  endif()

  # Include directories (shared by both targets)
  set(INCDIR
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/can
      ${CMAKE_CURRENT_SOURCE_DIR}/io_handler
      ${CMAKE_CURRENT_SOURCE_DIR}/uiux
      ${CMAKE_CURRENT_SOURCE_DIR}/logs
      ${CMAKE_CURRENT_SOURCE_DIR}/modules
      ${CMAKE_CURRENT_SOURCE_DIR}/cmd
      ${CMAKE_CURRENT_SOURCE_DIR}/config)

  # ── HMI Manager daemon ────────────────────────────────────────────────
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
      ${INCDIR}
    DEPENDS
      m)   # -lm for sinf/cosf in widgets.c (needle angle computation)

  # ── HMI Control client ────────────────────────────────────────────────
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
