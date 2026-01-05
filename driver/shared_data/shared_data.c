/**
 * @file share_functions.c
 * @author Zahra Soltani(Zahraslt49@gmail.com)
 * @date 04-2025
 */

#include "shared_data.h"


void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += (uint32_t)ts_field[i] << (i * 8);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------

 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    uint8_t i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ((uint32_t)ts_field[i] << (i * 8));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    uint8_t i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8_t)ts;
        ts >>= 8;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
 *        response message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts)
{
    uint8_t i;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8_t)(ts >> (i * 8));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------

 * @brief This function will continuously read the system status register until it matches the bits set in the mask
 *        input parameter. It will then exit the function.
 *        This is useful to use when waiting on particular events to occurs. For example, the user could wait for a
 *        good UWB frame to be received and/or no receive errors have occurred.
 *        The lower 32-bits of the system status register will be read in a while loop. Each iteration of the loop will check if a matching
 *        mask value for the higher 32-bits of the system status register is set. If the mask value is set in the higher 32-bits of the system
 *        status register, the function will return that value along with the last recorded value of the lower 32-bits of the system status
 *        register. Thus, the user should be aware that this function will not wait for high and low mask values to be set in both the low and high
 *        system status registers. Alternatively, the user can call this function to *only* check the higher or lower system status registers.
 *
 * input parameters
 * @param lo_result - A pointer to a uint32_t that will contain the final value of the system status register (lower 32 bits).
 *                    Pass in a NULL pointer to ignore returning this value.
 * @param hi_result - A pointer to a uint32_t that will contain the final value of the system status register (higher 32 bits).
 *                    Pass in a NULL pointer to ignore returning this value.
 * @param lo_mask - a uint32 mask value that is used to check for certain bits to be set in the system status register (lower 32 bits).
 *               Example values to use are as follows:
 *               DWT_INT_TXFRS_BIT_MASK - Wait for a TX frame to be sent.
 *               SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR - Wait for frame to be received and no reception errors.
 *               SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR - Wait for frame to be received and no receive timeout errors
 *                                                                                          and no reception errors.
 *               SYS_STATUS_RXFR_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_ND_RX_ERR - Wait for packet to be received and no receive timeout errors
 *                                                                                            and no reception errors.
 *                                                                                            These flags are useful when polling for STS Mode 4 (no data)
 *                                                                                            packets.
 *               0 - The function will not wait for any bits in the system status register (lower 32 bits).
 * @param hi_mask - a uint32 mask value that is used to check for certain bits to be set in the system status register (higher 32 bits).
 *               Example values to use are as follows:
 *               SYS_STATUS_HI_CCA_FAIL_BIT_MASK - Check for CCA fail status.
 *               0 - The function will not wait for any bits in the system status register (lower 32 bits).
 *
 * return None
 */
void waitforsysstatus(uint32_t *lo_result, uint32_t *hi_result, uint32_t lo_mask, uint32_t hi_mask)
{
    uint32_t lo_result_tmp = 0;
    uint32_t hi_result_tmp = 0;

    // If a mask has been passed into the function for the system status register (lower 32-bits)
    if (lo_mask)
    {
        while (!((lo_result_tmp = dwt_readsysstatuslo()) & (lo_mask)))
        {
            // If a mask value is set for the system status register (higher 32-bits)
            if (hi_mask)
            {
                // If mask value for the system status register (higher 32-bits) is found
                if ((hi_result_tmp = dwt_readsysstatushi()) & hi_mask)
                {
                    break;
                }
            }
        }
    }
    // if only a mask value for the system status register (higher 32-bits) is set
    else if (hi_mask)
    {
        while (!((hi_result_tmp = dwt_readsysstatushi()) & (hi_mask)))
        {
        };
    }

    if (lo_result != NULL)
    {
        *lo_result = lo_result_tmp;
    }

    if (hi_result != NULL)
    {
        *hi_result = hi_result_tmp;
    }
}
