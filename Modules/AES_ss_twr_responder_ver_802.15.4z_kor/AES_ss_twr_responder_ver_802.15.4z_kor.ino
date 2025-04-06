// 아래 코드는 https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000/tree/main의
// ex_06f_AES_ss_twr_responder 예제를 한국어로 번역한 것입니다.

#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

#define APP_NAME "SS TWR AES RESP v1.0"

// 연결 핀 정의
const uint8_t PIN_RST = 27;  // 리셋 핀
const uint8_t PIN_IRQ = 34;  // 인터럽트 핀
const uint8_t PIN_SS = 4;    // SPI 선택 핀

/* 
 * 이 SS-TWR(단측 양방향 거리 측정) 예제에서는 
 * `mac_frame_802_15_4_format_t` 구조체로 정의된 
 * IEEE 802.15.4 MAC 데이터 프레임 형식을 사용합니다.
 */
mac_frame_802_15_4_format_t mac_frame;

#if 0
// AES 암호화 설정 (비활성화된 코드)
static dwt_aes_config_t aes_config = {
    .key_load           = AES_KEY_Load,         // AES 엔진에 키를 로드 (아래 참고 사항 14 참조)
    .key_size           = AES_KEY_128bit,       // 128비트 키 사용
    .key_src            = AES_KEY_Src_Register, // 키 소스를 IC 레지스터로 설정
    .aes_core_type      = AES_core_type_CCM,    // CCM 코어 사용
    .aes_key_otp_type   = AES_key_RAM,
    .key_addr           = 0
};
#endif

// AES 암호화 구성
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

// 송신 및 수신 장치 주소 및 PAN ID 정의
#define SRC_ADDR 0x1122334455667788  /* 이니시에이터(발신기) 주소 */
#define DEST_ADDR 0x8877665544332211 /* 리스폰더(응답기) 주소 */
#define DEST_PAN_ID 0x4321           /* PAN ID (네트워크 식별자) */

/* 기본 통신 설정: STS 없이 기본 DW 모드 사용 */
static dwt_config_t config = {
  5,                /* 채널 번호 */
  DWT_PLEN_128,     /* 프리앰블 길이 (TX 전용) */
  DWT_PAC8,         /* 프리앰블 수신 청크 크기 (RX 전용) */
  9,                /* TX 프리앰블 코드 (TX 전용) */
  9,                /* RX 프리앰블 코드 (RX 전용) */
  1,                /* 0: 표준 8 심볼 SFD, 1: 비표준 8 심볼 SFD, 2: 비표준 16 심볼 SFD, 3: 4z 8 심볼 SDF 타입 */
  DWT_BR_6M8,       /* 데이터 전송 속도 */
  DWT_PHRMODE_STD,  /* PHY 헤더 모드 */
  DWT_PHRRATE_STD,  /* PHY 헤더 속도 */
  (129 + 8 - 8),    /* SFD 타임아웃 (프리앰블 길이 + 1 + SFD 길이 - PAC 크기, RX 전용) */
  DWT_STS_MODE_OFF, /* STS 비활성화 */
  DWT_STS_LEN_64,   /* STS 길이 (Enum dwt_sts_lengths_e에서 허용된 값 참조) */
  DWT_PDOA_M0       /* PDOA 모드 비활성화 */
};

/* 64 MHz PRF(펄스 반복 주파수)에 대한 기본 안테나 지연값 (NOTE 2 참조) */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385


/* AUX 보안 헤더의 키 인덱스에 따른 선택적 키 옵션 */
static dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

/* 거리 측정 과정에서 사용되는 MAC 페이로드 데이터. 아래의 주석 3을 참고하세요. */
/* 이니시에이터가 리스폰더에게 보내는 Poll 메시지 */
static uint8_t rx_poll_msg[] = { 'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e' };
/* 이니시에이터에게 보내는 응답(Response) 메시지. 
 * 첫 8바이트는 Poll RX 타임스탬프와 Response TX 타임스탬프를 저장하는 용도로 사용됨. */
static uint8_t tx_resp_msg[] = { 0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e' };

/* 프로세스에서 사용되는 프레임의 특정 필드에 접근하기 위한 인덱스 정의 */
#define ALL_MSG_SN_IDX 2           // MHR(매크 헤더) 내 시퀀스 번호 바이트의 인덱스
#define RESP_MSG_POLL_RX_TS_IDX 0  // MAC 페이로드 내 Poll RX 타임스탬프의 위치 인덱스
#define RESP_MSG_RESP_TX_TS_IDX 4  // MAC 페이로드 내 Response TX 타임스탬프의 위치 인덱스
#define RESP_MSG_TS_LEN 4          // 타임스탬프 길이(4바이트)

/* 수신된 응답 메시지를 저장할 버퍼.
 * 이 예제 코드에서 처리할 수 있는 가장 긴 프레임 크기에 맞춰 크기를 조정함. */
#define RX_BUF_LEN 127 /* STD PHR 모드가 사용될 경우, 수신된 프레임 크기는 127바이트를 초과할 수 없음 */
static uint8_t rx_buffer[RX_BUF_LEN];

/* 주의: 키 인덱스 값이 0인 경우 키 인덱스로 전송하는 것이 금지됨.
 * 따라서, 이 예제에서는 인덱스 1부터 사용함.
 * 이 예제에서는 리스폰더 데이터 암호화를 위해 키 테이블에서 인덱스 2를 사용함. */
#define RESPONDER_KEY_INDEX 2

/* 프레임 간 지연 시간(UWB 마이크로초 단위). 아래 주석 1을 참고하세요.
 * 이 값에는 Poll 프레임 길이(~240us)이 포함됨. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

/* 프레임 송수신 타임스탬프 */
static uint64_t poll_rx_ts; // Poll 메시지 수신 타임스탬프
static uint64_t resp_tx_ts; // Response 메시지 전송 타임스탬프

/* PG_DELAY 및 TX_POWER 레지스터 값은 현재 온도에서 대역폭과 스펙트럼의 출력을 반영함.
 * 이 값들은 참조 측정을 수행하기 전에 보정될 수 있음. 아래 주석 5를 참고하세요. */
extern dwt_txconfig_t txconfig_options;

dwt_aes_job_t aes_job_rx, aes_job_tx; // AES 암호화/복호화 작업 구조체
int8_t status;                        // 현재 상태 변수
uint32_t status_reg;                   // DW3000 상태 레지스터 값


void setup() {
  UART_init();  
  test_run_info((unsigned char *)APP_NAME);  

  /* SPI 속도 설정, DW3000은 최대 38MHz까지 지원 */  
  /* DW IC(집적회로) 리셋 */  
  spiBegin(PIN_IRQ, PIN_RST);  
  spiSelect(PIN_SS);  

  delay(2);  // DW3000이 시작할 시간을 제공 (INIT_RC에서 IDLE_RC로 전환되는 시간, 또는 SPIRDY 이벤트를 기다릴 수도 있음)  

  while (!dwt_checkidlerc())  // DW IC가 IDLE_RC 상태인지 확인한 후 진행해야 함  
  {  
    UART_puts("IDLE FAILED\r\n");  
    while (1);  
  }  

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {  
    UART_puts("INIT FAILED\r\n");  
    while (1);  
  }  

  /* 디버깅을 위해 LED를 활성화하여 TX 시 DW3000 평가 보드의 D1 LED(빨간색)가 깜박이도록 설정  
     * 단, 저전력 애플리케이션에서는 LED를 사용하지 않는 것이 좋음. */  
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);  

  /* DW IC(집적회로) 구성. 아래 NOTE 13을 참고하세요. */  
  if (dwt_configure(&config)) /* dwt_configure가 DWT_ERROR를 반환하면 PLL 또는 RX 보정이 실패한 것이므로  
                                 * 호스트는 장치를 리셋해야 함. */  
  {  
    test_run_info((unsigned char *)"CONFIG FAILED     ");  
    while (1) {};  
  }  

  /* TX 스펙트럼 매개변수(출력 전력, PG 지연 및 PG 카운트) 구성 */  
  dwt_configuretxrf(&txconfig_options);  

  /* 기본 안테나 지연 값 적용. 아래 NOTE 2를 참고하세요. */  
  dwt_setrxantennadelay(RX_ANT_DLY);  
  dwt_settxantennadelay(TX_ANT_DLY);  

  /* GPIO 5 및 6에서 TX/RX 상태 출력을 활성화하여 디버깅을 돕고, TX/RX LED도 활성화 가능  
     * 단, 저전력 애플리케이션에서는 LED를 사용하지 않는 것이 좋음. */  
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);  



  /* TX 및 RX AES 작업을 구성합니다.  
     * TX 작업은 응답(Response) 메시지를 암호화하는 데 사용되며,  
     * RX 작업은 폴(Poll) 메시지를 복호화하는 데 사용됩니다. */  

  aes_job_rx.mode = AES_Decrypt;                               /* 모드를 복호화(Decryption)로 설정 */  
  aes_job_rx.src_port = AES_Src_Rx_buf_0;                      /* RX 버퍼에서 암호화된 프레임을 가져옴 */  
  aes_job_rx.dst_port = AES_Dst_Rx_buf_0;                      /* 복호화된 프레임을 동일한 RX 버퍼에 저장 (원본 RX 프레임이 덮어쓰기됨) */  
  aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);   /* MAC 헤더 길이 설정 (mac_frame에는 MAC 헤더가 포함됨) */  
  aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); /* 암호화되지 않을 평문 헤더의 포인터 설정 */  
  aes_job_rx.payload = rx_buffer;                              /* 복호화된 RX MAC 프레임 페이로드를 IC에서 읽어와 저장할 버퍼 */  

  aes_job_tx.mode = AES_Encrypt;        /* 암호화(Encryption) 작업 설정 */  
  aes_job_tx.src_port = AES_Src_Tx_buf; /* dwt_do_aes가 평문을 TX 버퍼로 가져옴 */  
  aes_job_tx.dst_port = AES_Dst_Tx_buf; /* dwt_do_aes가 원본 평문 TX 버퍼를 암호화된 데이터로 대체 */  
  aes_job_tx.header_len = aes_job_rx.header_len;  
  aes_job_tx.header = aes_job_rx.header;        /* 암호화되지 않을 평문 헤더 */  
  aes_job_tx.payload = tx_resp_msg;             /* 전송할 페이로드 */  
  aes_job_tx.payload_len = sizeof(tx_resp_msg); /* 페이로드 길이 */  

}

void loop() {
  /* 즉시 수신 활성화 */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* 프레임 수신 또는 오류/타임아웃을 감지할 때까지 폴링. 참고: NOTE 6 참조 */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {};

  /* 프레임을 수신하면 페이로드를 읽고 복호화 수행 */
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;

    /* DW IC 상태 레지스터에서 정상적인 RX 프레임 이벤트를 클리어 */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* 수신된 데이터 길이 읽기 */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    /* 프레임이 수신됨: 우선 MHR을 읽고, 해당 프레임이 예상한 것인지 확인해야 함.  
       * 목적지 주소가 송신자의 소스 주소와 일치해야 함 (프레임 필터링을 통해 확인 가능하나, 본 예제에서는 포함되지 않음).  
       * 또한 헤더에 보안이 활성화되어 있어야 함.  
       * 이러한 검사를 통과하지 못하면 rx_aes_802_15_4 함수는 오류를 반환함.
    */

    aes_config.mode = AES_Decrypt;  /* 복호화 모드로 설정 */
    PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer;  /* MAC 프레임 구조체의 페이로드 포인터 설정  
                                                      (이 포인터는 복호화된 데이터를 포함하게 됨) */

    status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), 
                             keys_options, DEST_ADDR, SRC_ADDR, &aes_config);

    if (status != AES_RES_OK) {
      /* 오류 발생 시 처리 */
      do {
        switch (status) {
          case AES_RES_ERROR_LENGTH:
            test_run_info((unsigned char *)"AES 길이 오류");
            break;
          case AES_RES_ERROR:
            test_run_info((unsigned char *)"AES 오류");
            break;
          case AES_RES_ERROR_FRAME:
            test_run_info((unsigned char *)"프레임 오류");
            break;
          case AES_RES_ERROR_IGNORE_FRAME:
            test_run_info((unsigned char *)"해당 프레임은 대상이 아님");
            continue;  // 잘못된 목적지 주소를 가진 프레임 무시
        }
      } while (1);
    }

    /* MAC 프레임의 페이로드가 "SS TWR AES initiator" 예제에서 보내야 할 예상되는 Poll 메시지와 일치하는지 확인 */
    if (memcmp(rx_buffer, rx_poll_msg, aes_job_rx.payload_len) == 0) {
      uint32_t resp_tx_time;
      int ret;
      uint8_t nonce[13];

      /* Poll 수신 타임스탬프를 가져옴 */
      poll_rx_ts = get_rx_timestamp_u64();

      /* 응답 메시지 전송 시간을 계산. 참고: NOTE 7 참조 */
      resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      /* 응답 TX 타임스탬프는 우리가 프로그래밍한 전송 시간에 안테나 지연 시간을 더한 값 */
      resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

      /* 최종 메시지에 모든 타임스탬프를 작성. 참고: NOTE 8 참조 */
      resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
      resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

      /* 이제 프레임을 전송하기 전에 암호화해야 함 */

      /* 사용될 올바른 키를 프로그램 */
      dwt_set_keyreg_128(&keys_options[RESPONDER_KEY_INDEX - 1]);
      /* 프레임에 사용할 키 인덱스를 설정 */
      MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = RESPONDER_KEY_INDEX;

      /* 시퀀스 번호 증가 */
      MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame)
      ++;

      /* 프레임 카운트 업데이트 */
      mac_frame_update_aux_frame_cnt(&mac_frame, mac_frame_get_aux_frame_cnt(&mac_frame) + 1);

      /* AES 작업을 구성 */
      aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
      aes_job_tx.nonce = nonce; /* MHR 설정 후 아래에서 설정됨 */
      aes_config.mode = AES_Encrypt;
      aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
      dwt_configure_aes(&aes_config);

      /* MHR 업데이트 (수신된 MHR 재사용, 따라서 SRC/DEST 주소를 바꿔야 함) */
      mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, DEST_ADDR, SRC_ADDR);

      /* MHR에서 nonce 생성 */
      mac_frame_get_nonce(&mac_frame, nonce);

      /* 암호화 수행, TX 버퍼는 암호화된 페이로드가 포함된 전체 MAC 프레임을 가질 것 */
      status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
      if (status < 0) {
        test_run_info((unsigned char *)"AES 길이 오류");
        while (1)
          ; /* 오류 */
      } else if (status & AES_ERRORS) {
        test_run_info((unsigned char *)"AES 오류");
        while (1)
          ; /* 오류 */
      }

      /* 프레임 제어를 구성하고 전송 시작 */
      dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1); /* TX 버퍼의 오프셋을 0으로 설정, ranging. */
      ret = dwt_starttx(DWT_START_TX_DELAYED);

      /* dwt_starttx()가 오류를 반환하면, 이 ranging 교환을 포기하고 다음 것으로 진행. 참고: NOTE 10 참조. */
      if (ret == DWT_SUCCESS) {
        /* TX 프레임 전송 이벤트가 설정될 때까지 DW IC를 폴링. 참고: NOTE 6 참조. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};

        /* TXFRS 이벤트를 클리어. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      }
    }
  } else {
    /* DW IC 상태 레지스터에서 RX 오류 이벤트를 클리어. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.
 *
 *    Initiator: |Poll TX| ..... |Resp RX|
 *    Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - time of Poll TX/RX
 *                                   ^|R RMARKER|    - time of Resp TX/RX
 *
 *                       <--TDLY->                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->            - RESP_RX_TIMEOUT_UUS   (length of response frame)
 *                    <----RDLY------>               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder can turn around and reply)
 *
 *
 * 2. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 0 -> 13: poll message reception timestamp.
 *     - byte 4 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 6. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 7. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW IC
 *    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *    8 bits.
 * 8. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *    time-of-flight computation) can be handled by a 32-bit subtraction.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 10. When running this example on the EVB1000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_06a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 11. The user is referred to DecaRanging ARM application (distributed with DW3000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 * 14. When CCM core type is used, AES_KEY_Load needs to be set prior to each encryption/decryption operation, even if the AES KEY used has not changed.
 ****************************************************************************************************************************************************/
