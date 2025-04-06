#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

#define APP_NAME "SS TWR AES RESP v1.0"

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

mac_frame_802_15_4_format_t mac_frame;

static dwt_aes_config_t aes_config = {
  AES_key_RAM,
  AES_core_type_CCM,
  MIC_0,
  AES_KEY_Src_Register,
  AES_KEY_Load,
  0,
  AES_KEY_128bit,
  AES_Encrypt
};

#define SRC_ADDR 0x1122334455667788
#define DEST_ADDR 0x8877665544332211
#define DEST_PAN_ID 0x4321

static dwt_config_t config = {
  5,
  DWT_PLEN_128,
  DWT_PAC8,
  9,
  9,
  1,
  DWT_BR_6M8,
  DWT_PHRMODE_STD,
  DWT_PHRRATE_STD,
  (129 + 8 - 8),
  DWT_STS_MODE_OFF,
  DWT_STS_LEN_64,
  DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

static uint8_t rx_poll_msg[] = { 'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e' };
static uint8_t tx_resp_msg[] = { 0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e' };

#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 0
#define RESP_MSG_RESP_TX_TS_IDX 4
#define RESP_MSG_TS_LEN 4

#define RX_BUF_LEN 127
static uint8_t rx_buffer[RX_BUF_LEN];

#define RESPONDER_KEY_INDEX 2
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

extern dwt_txconfig_t txconfig_options;

dwt_aes_job_t aes_job_rx, aes_job_tx;
int8_t status;
uint32_t status_reg;

void setup() {
  UART_init();  
  test_run_info((unsigned char *)APP_NAME);  

  spiBegin(PIN_IRQ, PIN_RST);  
  spiSelect(PIN_SS);  

  delay(2);  

  while (!dwt_checkidlerc())  
  {  
    UART_puts("IDLE FAILED\r\n");  
    while (1);  
  }  

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {  
    UART_puts("INIT FAILED\r\n");  
    while (1);  
  }  

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);  

  if (dwt_configure(&config))  
  {  
    test_run_info((unsigned char *)"CONFIG FAILED     ");  
    while (1) {};  
  }  

  dwt_configuretxrf(&txconfig_options);  
  dwt_setrxantennadelay(RX_ANT_DLY);  
  dwt_settxantennadelay(TX_ANT_DLY);  
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);  

  aes_job_rx.mode = AES_Decrypt;  
  aes_job_rx.src_port = AES_Src_Rx_buf_0;  
  aes_job_rx.dst_port = AES_Dst_Rx_buf_0;  
  aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);  
  aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);  
  aes_job_rx.payload = rx_buffer;  

  aes_job_tx.mode = AES_Encrypt;  
  aes_job_tx.src_port = AES_Src_Tx_buf;  
  aes_job_tx.dst_port = AES_Dst_Tx_buf;  
  aes_job_tx.header_len = aes_job_rx.header_len;  
  aes_job_tx.header = aes_job_rx.header;  
  aes_job_tx.payload = tx_resp_msg;  
  aes_job_tx.payload_len = sizeof(tx_resp_msg);  
}

void loop() {
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {};

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    aes_config.mode = AES_Decrypt;
    PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer;

    status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), 
                             keys_options, DEST_ADDR, SRC_ADDR, &aes_config);

    if (status != AES_RES_OK) {
      do {
        switch (status) {
          case AES_RES_ERROR_LENGTH:
            test_run_info((unsigned char *)"AES ");
            break;
          case AES_RES_ERROR:
            test_run_info((unsigned char *)"AES ");
            break;
          case AES_RES_ERROR_FRAME:
            test_run_info((unsigned char *)" ");
            break;
          case AES_RES_ERROR_IGNORE_FRAME:
            test_run_info((unsigned char *)" ");
            continue;
        }
      } while (1);
    }

    if (memcmp(rx_buffer, rx_poll_msg, aes_job_rx.payload_len) == 0) {
      uint32_t resp_tx_time;
      int ret;
      uint8_t nonce[13];

      poll_rx_ts = get_rx_timestamp_u64();
      resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);
      resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

      resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
      resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

      dwt_set_keyreg_128(&keys_options[RESPONDER_KEY_INDEX - 1]);
      MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = RESPONDER_KEY_INDEX;
      MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame)
      ++;
      mac_frame_update_aux_frame_cnt(&mac_frame, mac_frame_get_aux_frame_cnt(&mac_frame) + 1);

      aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
      aes_job_tx.nonce = nonce;
      aes_config.mode = AES_Encrypt;
      aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
      dwt_configure_aes(&aes_config);

      mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, DEST_ADDR, SRC_ADDR);
      mac_frame_get_nonce(&mac_frame, nonce);

      status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
      if (status < 0) {
        test_run_info((unsigned char *)"AES ");
        while (1)
          ;
      } else if (status & AES_ERRORS) {
        test_run_info((unsigned char *)"AES ");
        while (1)
          ;
      }

      dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1);
      ret = dwt_starttx(DWT_START_TX_DELAYED);

      if (ret == DWT_SUCCESS) {
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      }
    }
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}