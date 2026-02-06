/****************************************************************************
 * apps/hmi_manager/can/can_trace.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <nuttx/can.h>
#include <nuttx/clock.h>

#include "can_trace.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_trace_enabled = false;
static uint32_t g_id_filter = 0;
static uint32_t g_start_time = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void log_frame(const char *dir, const struct can_frame *frame)
{
  uint32_t id;
  uint32_t timestamp;
  char buf[128];
  int len;
  int i;

  if (!g_trace_enabled)
    {
      return;
    }

  id = frame->can_id & CAN_EFF_MASK;

  /* Apply filter */

  if (g_id_filter != 0 && id != g_id_filter)
    {
      return;
    }

  /* Get relative timestamp in milliseconds */

  timestamp = TICK2MSEC(clock_systime_ticks()) - g_start_time;

  /* Format: timestamp | interface | id | dlc | data */

  len = snprintf(buf, sizeof(buf), "%10lu | can0 | %08lX | %d |",
                 (unsigned long)timestamp,
                 (unsigned long)id,
                 frame->can_dlc);

  /* Append data bytes */

  for (i = 0; i < frame->can_dlc && i < 8; i++)
    {
      len += snprintf(buf + len, sizeof(buf) - len, " %02X",
                      frame->data[i]);
    }

  /* Single syslog call */

  syslog(LOG_INFO, "%s\n", buf);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void can_trace_init(void)
{
  g_trace_enabled = false;
  g_id_filter = 0;
  
  /* Save start time for relative timestamps */
  
  g_start_time = TICK2MSEC(clock_systime_ticks());
}

void can_trace_enable(bool enable)
{
  g_trace_enabled = enable;
  
  if (enable)
    {
      /* Print header when enabling */
      
      syslog(LOG_INFO, "%-10s | %-9s | %-8s | %-3s | %s\n",
             "timestamp", "interface", "id", "dlc", "data");
      syslog(LOG_INFO, "----------------------------------------------\n");
    }
  else
    {
      syslog(LOG_INFO, "CAN trace disabled\n");
    }
}

bool can_trace_is_enabled(void)
{
  return g_trace_enabled;
}

void can_trace_rx(const struct can_frame *frame)
{
  log_frame("RX", frame);
}

void can_trace_tx(const struct can_frame *frame)
{
  log_frame("TX", frame);
}

void can_trace_set_filter(uint32_t id_mask)
{
  g_id_filter = id_mask;
  
  if (id_mask == 0)
    {
      syslog(LOG_INFO, "CAN trace: filter disabled (all IDs)\n");
    }
  else
    {
      syslog(LOG_INFO, "CAN trace: filter set to 0x%08lX\n",
             (unsigned long)id_mask);
    }
}
