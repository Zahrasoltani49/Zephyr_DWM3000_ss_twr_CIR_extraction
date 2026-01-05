#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include <float.h>
#include "shared_data.h"
LOG_MODULE_REGISTER(main, CONFIG_DW3000_LOG_LEVEL);

static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_1024,    /* Preamble length. Used in TX only. */
    DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (1025 + 8 - 32),  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define SPEED_OF_LIGHT (299702547)
#define FRAME_LEN_MAX (127)
#define FRAME_LEN_MAX_EX (1023)

/* Frames used in the ranging process. See NOTE 3 below.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *  *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 1, 'T', 1, '1', 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 1, 'A', 1, '2', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_MSG_ANCHOR_ADDR_IDX 6
#define RESP_MSG_ANCHOR_ADDR_IDX 8
uint8_t i;
float distances_storage[5];
static uint8_t frame_seq_nb = 0;

#define NUMBER_OF_ANCHORS 4

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */

#define POLL_TX_TO_RESP_RX_DLY_UUS 1600
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 1600

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

static dwt_txconfig_t txconfig =
    {
        0x34,       /* PG delay. */
        0xfdfdfdfd, /* TX power. */
        0x0         /*PG count*/
};

/* part of the code for extracting features based on the register map
**************************************************************************** */

uint8_t systemEnable[4];
uint8_t systemStatus[20];
uint8_t header[2];

uint8_t read_buffer_size = 4;
uint8_t h_size = sizeof(header);


void i2c_header(char status[], uint8_t base_address, uint8_t sub_address)
{

        if (strcmp(status, "READ") == 0)
        {
                uint16_t h = 0x4000;
                uint16_t mask_base = 0x1F << 9;
                uint8_t mask_sub = 0x7F;
                h &= ~mask_base;
                h |= base_address << 9;
                h &= ~mask_sub;
                h |= sub_address << 2;
                header[0] = (h & 0xFF00) >> 8;
                header[1] = (h & 0x00FF);
        }

        if (strcmp(status, "WRITE") == 0)
        {
                uint16_t h = 0xC000;
                uint16_t mask_base = 0x1F << 9;
                uint8_t mask_sub = 0x7F;
                h &= ~mask_base;
                h |= base_address << 9;
                h &= ~mask_sub;
                h |= sub_address << 2;
                header[0] = (h & 0xFF00) >> 8;
                header[1] = (h & 0x00FF);
        }
}
/*

**************************************************************************************** */

int main(void)
{

        LOG_ERR("Welcome ss twr Initiator ");
        // uint8_t header[2];
        // uint8_t r_buffer[4];
        // header[0] = 0X40;
        // header[1] = 0x00;

        dw3000_hw_init();

        // while(1){
        dw3000_hw_reset();
        // }
        // while(1){
        dw3000_hw_init_interrupt();
        // }
        dw3000_spi_speed_fast();
        k_msleep(50);
        dw3000_hw_wakeup();
        // while (1)
        // {
        //  int ret = dwt_check_dev_id();
        int ret = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
        if (ret < 0)
        {
                LOG_ERR("DWT Probe failed-1");
                k_msleep(1000);
        }

        if (!dwt_checkidlerc()) // ideal =1
                LOG_INF("DW3000 is NOT in IDLE_RC mode");
        else
                LOG_INF("DW3000  is in IDLE_RC mode");

        if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
        {
                LOG_ERR("DWT init failed");
        }
        else
                LOG_INF("DW3000 init DONE");
        if (!dwt_configure(&config)) // DWT_SUCCESS = 0,
        {
                LOG_INF("DW3000 configuration DONE");
        }
        else
                LOG_INF("DW3000 configuration failed");

        dwt_configuretxrf(&txconfig);

        // Turn on the RX TX LEDs on the DWM3001 board
        dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

        /* Apply default antenna delay value. See NOTE 2 below. */
        dwt_setrxantennadelay(RX_ANT_DLY);
        dwt_settxantennadelay(TX_ANT_DLY);

        /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
         * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        // i2c_header("READ", RXPACC_BASE_ADDRESS, RXPACC_SUB_ADDRESS); // RxPACC
        /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
         * Note, in real low power applications the LEDs should not be used. */
        for (i = 1; i <= NUMBER_OF_ANCHORS; i++)
        {

                if (i == 1)
                        printk("['\n");

                /* Loop forever initiating ranging exchanges. */
                while (1)
                {
                        tx_poll_msg[POLL_MSG_ANCHOR_ADDR_IDX] = i;
                        rx_resp_msg[RESP_MSG_ANCHOR_ADDR_IDX] = i;
                        /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
                        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

                        // i2c_header("READ", 0x00, 0x44);
                        // ret = dw3000_spi_read(h_size, header, read_buffer_size, systemStatus);
                        // k_msleep(5);
                        //  LOG_INF("systemStatus before writing is = %d,%d,%d,%d", systemStatus[0] | systemStatus[1] << 8 | systemStatus[2] << 16 | systemStatus[3] << 24);

                        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                        ret = dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
                        // if (ret == DWT_SUCCESS)
                        //         LOG_INF("dwt_writetxdata  DONE");
                        // else
                        //         LOG_ERR("dwt_writetxdata  FAILED");

                        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

                        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
                         * set by dwt_setrxaftertxdelay() has elapsed. */
                        ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                        // if (ret == DWT_SUCCESS)
                        //         LOG_INF("send  DONE");
                        // // else
                        // //         LOG_ERR("send  FAILED");

                        // dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
                        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
                        // LOG_INF("status is = %d", status_reg);
                        /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                        frame_seq_nb++;
                        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
                        {
                                // LOG_INF("inside the if");
                                uint16_t frame_len;
                                /* Clear good RX frame event in the DW IC status register. */
                                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

                                dwt_writesysstatuslo(DWT_INT_RXPHD_BIT_MASK);
                                uint16_t status2 = dwt_readsysstatuslo();
                                // LOG_INF("status 2: %d", status2);
                                /* A frame has been received, read it into the local buffer. */
                                frame_len = dwt_getframelength();

                                if (frame_len <= sizeof(rx_buffer))
                                {
                                        dwt_readrxdata(rx_buffer, frame_len, 0);

                                        rx_buffer[ALL_MSG_SN_IDX] = 0;
                                        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                                        {

                                                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                                                int32_t rtd_init, rtd_resp;
                                                float clockOffsetRatio;
                                                /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                                                poll_tx_ts = dwt_readtxtimestamplo32();
                                                resp_rx_ts = dwt_readrxtimestamplo32();

                                                /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                                                clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

                                                /* Get timestamps embedded in response message. */
                                                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                                                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                                                /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                                                rtd_init = resp_rx_ts - poll_tx_ts;
                                                rtd_resp = resp_tx_ts - poll_rx_ts;

                                                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                                                distance = tof * SPEED_OF_LIGHT;
                                                // distance = distance * 100; // convert to cm
                                                /* Display computed distance on LCD. */
                                                // printf("DIST %d: %3.2f\n", i, distance);

                                                printk("{'anchor_ID': 0x0%d ,'FP1': %d , 'FP2': %d , 'FP3': %d , 'rxpacc': %d , 'maxgrowth': %d , 'distance': %3.3f}\n ", i, data_raspberryPi_sample.fp1_raspberryPi,
                                                       data_raspberryPi_sample.fp2_raspberryPi, data_raspberryPi_sample.fp3_raspberryPi, data_raspberryPi_sample.rxpacc_raspberryPi,
                                                       data_raspberryPi_sample.maxgrowth_raspberryPi, distance);

                                                distances_storage[i] = distance;
                                                if (i < NUMBER_OF_ANCHORS)
                                                {
                                                        printk(",\n"); // comma between items
                                                }
                                                if (i == NUMBER_OF_ANCHORS)
                                                {
                                                        i = 0;
                                                        printk("\n']\n");
                                                        k_msleep(1000);
                                                        // printf("Distance 1: %3.2f, Distance 2: %3.2f, Distance 3: %3.2f \n", distances_storage[1], distances_storage[2], distances_storage[3]);
                                                }

                                                k_msleep(50);
                                                break;
                                        }
                                }
                        }

                        else
                        {
                                /* Clear RX error/timeout events in the DW IC status register. */
                                dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                        }

                        /* Execute a delay between ranging exchanges. */
                        k_msleep(RNG_DELAY_MS);
                }
        }
}
