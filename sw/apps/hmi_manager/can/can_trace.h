/****************************************************************************
 * apps/hmi_manager/can/can_trace.h
 ****************************************************************************/

#ifndef __APPS_HMI_MANAGER_CAN_CAN_TRACE_H
#define __APPS_HMI_MANAGER_CAN_CAN_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/can.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize CAN trace */

void can_trace_init(void);

/* Enable/disable trace */

void can_trace_enable(bool enable);

/* Check if trace is enabled */

bool can_trace_is_enabled(void);

/* Log RX frame */

void can_trace_rx(const struct can_frame *frame);

/* Log TX frame */

void can_trace_tx(const struct can_frame *frame);

/* Set ID filter (0 = log all) */

void can_trace_set_filter(uint32_t id_mask);

#endif /* __APPS_HMI_MANAGER_CAN_CAN_TRACE_H */
