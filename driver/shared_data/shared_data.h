/**
 * @file share_functions.h
 * @author Zahra Soltani(Zahraslt49@gmail.com)
 * @date 04-2025
 */

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"

#define SPEED_OF_LIGHT (299702547)
#define FRAME_LEN_MAX (127)
#define FRAME_LEN_MAX_EX (1023)

#define RXFLEN_MASK 0x0000007FUL    /* Receive Frame Length (0 to 127) */
#define RXFL_MASK_1023 0x000003FFUL /* Receive Frame Length Extension (0 to 1023) */

#define RESP_MSG_TS_LEN 4
#define FINAL_MSG_TS_LEN 4

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 63898

// ======================================Functions decleration =========================================
void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
void waitforsysstatus(uint32_t *lo_result, uint32_t *hi_result, uint32_t lo_mask, uint32_t hi_mask);

#endif
