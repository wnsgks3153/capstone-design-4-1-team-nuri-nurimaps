#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"
#define APP_NAME "SS TWR AES INIT v1.0"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

mac_frame_802_15_4_format_t mac_frame = {
  { { 0x09, 0xEC },
    0x00,
    { 0x21, 0x43 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x0F, { 0x00, 0x00, 0x00, 0x00 }, 0x00 } },
  0x00
};

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

#define DEST_ADDR 0x1122334455667788
#define SRC_ADDR 0x8877665544332211
#define DEST_PAN_ID 0x4321

static dwt_config_t config = {
  5,
  DWT_PLEN_128,
  DWT_PAC8,
  9,
  9,
  3,
  DWT_BR_6M8,
  DWT_PHRMODE_STD,
  DWT_PHRRATE_STD,
  (129 + 8 - 8),
  DWT_STS_MODE_1,
  DWT_STS_LEN_128,
  DWT_PDOA_M0
};

static dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

#define RNG_DELAY_MS 1000

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t tx_poll_msg[] = { 'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e' };
static uint8_t rx_resp_msg[] = { 0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e' };

#define START_RECEIVE_DATA_LOCATION 8

#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 0
#define RESP_MSG_RESP_TX_TS_IDX 4
#define RESP_MSG_TS_LEN 4

#define INITIATOR_KEY_INDEX 1

#define RX_BUF_LEN 127
static uint8_t rx_buffer[RX_BUF_LEN];

#define POLL_TX_TO_RESP_RX_DLY_UUS 1720
#define RESP_RX_TIMEOUT_UUS 250

static double tof;
static double distance;

extern dwt_txconfig_t txconfig_options;

static uint32_t frame_cnt = 0;
static uint8_t seq_cnt = 0x0A;
uint32_t status_reg;
uint8_t nonce[13];
dwt_aes_job_t aes_job_tx, aes_job_rx;
int8_t status;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("Long name works now");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2);

  while (!dwt_checkidlerc())
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  if (dwt_configure(&config)) {
    test_run_info((unsigned char *)"CONFIG FAILED     ");
    while (1) {};
  }

  dwt_configuretxrf(&txconfig_options);

  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  aes_job_tx.mode = AES_Encrypt;
  aes_job_tx.src_port = AES_Src_Tx_buf;
  aes_job_tx.dst_port = AES_Dst_Tx_buf;
  aes_job_tx.nonce = nonce;
  aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);
  aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
  aes_job_tx.payload = tx_poll_msg;
  aes_job_tx.payload_len = sizeof(tx_poll_msg);

  aes_job_rx.mode = AES_Decrypt;
  aes_job_rx.src_port = AES_Src_Rx_buf_0;
  aes_job_rx.dst_port = AES_Dst_Rx_buf_0;
  aes_job_rx.header_len = aes_job_tx.header_len;
  aes_job_rx.header = aes_job_tx.header;
  aes_job_rx.payload = rx_buffer;
}

void loop() {
  dwt_set_keyreg_128(&keys_options[INITIATOR_KEY_INDEX - 1]);
  MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = INITIATOR_KEY_INDEX;

  mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, DEST_ADDR, SRC_ADDR);
  mac_frame_get_nonce(&mac_frame, nonce);

  aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
  aes_config.mode = AES_Encrypt;
  aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
  dwt_configure_aes(&aes_config);

  status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
  if (status < 0) {
    test_run_info((unsigned char *)"AES length error");
    while (1)
      ;
  } else if (status & AES_ERRORS) {
    test_run_info((unsigned char *)"ERROR AES");
    while (1)
      ;
  }

  dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1);

  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

  MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame) = ++seq_cnt;
  mac_frame_update_aux_frame_cnt(&mac_frame, ++frame_cnt);

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    aes_config.mode = AES_Decrypt;
    PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer;

    status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), keys_options, DEST_ADDR, SRC_ADDR, &aes_config);
    if (status != AES_RES_OK) {
      do {
        switch (status) {
          case AES_RES_ERROR_LENGTH:
            test_run_info((unsigned char *)"Length AES error");
            break;
          case AES_RES_ERROR:
            test_run_info((unsigned char *)"ERROR AES");
            break;
          case AES_RES_ERROR_FRAME:
            test_run_info((unsigned char *)"Error Frame");
            break;
          case AES_RES_ERROR_IGNORE_FRAME:
            test_run_info((unsigned char *)"Frame not for us");
            continue;
        }
      } while (1);
    }

    if (memcmp(&rx_buffer[START_RECEIVE_DATA_LOCATION], &rx_resp_msg[START_RECEIVE_DATA_LOCATION],
               aes_job_rx.payload_len - START_RECEIVE_DATA_LOCATION)
        == 0) {
      uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
      int32_t rtd_init, rtd_resp;
      float clockOffsetRatio;

      poll_tx_ts = dwt_readtxtimestamplo32();
      resp_rx_ts = dwt_readrxtimestamplo32();

      clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
      resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

      rtd_init = resp_rx_ts - poll_tx_ts;
      rtd_resp = resp_tx_ts - poll_rx_ts;

      tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
      distance = tof * SPEED_OF_LIGHT;

      snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
      test_run_info((unsigned char *)dist_str);
    }

  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }

  Sleep(RNG_DELAY_MS);
}



/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. 여기에서 구현된 단방향 양방향 거리 측정 방식은 장치 간의 시계 오프셋 오류와 프레임 간 응답 지연 시간에 매우 민감하므로 거리를 정확하게 측정하려면
 *    주의 깊게 고려해야 합니다. 최상의 정확도를 얻기 위해서는 이 응답 지연 시간을 가능한 한 낮게 유지해야 합니다. 이를 위해 이 예제에서는 6.8 Mbps 데이터 속도를 사용하고,
 *    프레임 간 응답 지연 시간을 가능한 한 낮게 정의합니다. 단방향 양방향 거리 측정 과정에 대한 자세한 내용은 사용자 설명서를 참조하십시오. 참고: 11번을 참조하십시오.
 *
 *    Initiator: |Poll TX| ..... |Resp RX|
 *    Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - Poll TX/RX의 시간
 *                                   ^|R RMARKER|    - Resp TX/RX의 시간
 *
 *                       <--TDLY->                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->            - RESP_RX_TIMEOUT_UUS   (응답 프레임의 길이)
 *                    <----RDLY------>               - POLL_RX_TO_RESP_TX_DLY_UUS (응답자가 얼마나 빨리 응답을 전송할 수 있는지에 따라 다름)
 *
 *
 * 2. 값들의 합은 TX와 RX 안테나 지연 시간이며, 이는 보정 과정을 통해 실험적으로 결정해야 합니다. 여기서는 하드코딩된 값을 사용하고 있으며, 
 *    이는 결과 거리 추정에서 약간의 낮은 값을 보여줄 것으로 예상됩니다. 실제 생산 응용 프로그램에서는 각 장치가 적절하게 보정된 안테나 지연 시간을 사용해야
 *    정확한 범위 측정을 수행할 수 있습니다.
 * 3. 여기에서 사용된 프레임은 Decawave 특정 거리 측정 프레임으로, IEEE 802.15.4 표준 데이터 프레임 인코딩을 따릅니다. 프레임은 다음과 같습니다:
 *     - Initiator가 전송하는 Poll 메시지, 거리 측정을 시작하는 역할.
 *     - 응답자가 전송하는 Response 메시지, Initiator가 비행 시간(거리)을 계산할 수 있도록 필요한 모든 정보를 제공합니다.
 *    첫 10바이트는 공통이며, 다음과 같은 필드를 포함합니다:
 *     - 바이트 0/1: 프레임 제어 (0x8841, 16비트 주소 지정 데이터 프레임을 나타냄).
 *     - 바이트 2: 시퀀스 번호, 각 새로운 프레임마다 증가.
 *     - 바이트 3/4: PAN ID (0xDECA).
 *     - 바이트 5/6: 대상 주소, 아래 4번 참고.
 *     - 바이트 7/8: 출발지 주소, 아래 4번 참고.
 *     - 바이트 9: 기능 코드 (거리 측정 과정 중 어떤 메시지인지 나타내는 특정 값).
 *    나머지 바이트는 각 메시지에 따라 다릅니다:
 *    Poll 메시지:
 *     - 추가 데이터 없음
 *    Response 메시지:
 *     - 바이트 0 -> 13: Poll 메시지 수신 타임스탬프.
 *     - 바이트 4 -> 17: 응답 메시지 전송 타임스탬프.
 *    모든 메시지는 DW IC에 의해 자동으로 설정된 2바이트 체크섬으로 끝납니다.
 * 4. 이 예제에서는 소스 주소와 대상 주소가 하드코딩된 상수로 설정되어 있지만, 실제 제품에서는 각 장치에 고유한 ID가 있어야 합니다. 여기서는 16비트 주소 지정 방식을 사용하여
 *    메시지를 최대한 짧게 유지하고 있지만, 실제 응용 프로그램에서는 거리 측정을 위한 각 장치의 주소를 정의하는 특정 메시지 교환 후에만 이 주소가 사용되어야 합니다.
 * 5. 이 타임아웃은 프레임을 완전히 수신하는 데 필요한 시간으로, 예상되는 프레임의 길이를 고려하여 설정해야 합니다. 여기서 값은 임의로 선택되었으며, 응답자가 6.8M 데이터 속도로
 *    전송하는 응답 프레임을 수신하는 데 충분한 시간을 확보할 수 있도록 충분히 크게 설정되었습니다 (약 200µs).
 * 6. 실제 응용 프로그램에서는 규제 한도 내에서 최적의 성능을 발휘하기 위해 TX 펄스 대역폭 및 TX 전력 (dwt_configuretxrf API 호출 사용)을 장치별로 보정된 값으로 설정할 수 있습니다.
 * 7. dwt_writetxdata()는 전체 메시지 크기를 매개변수로 받지만, DW IC에서 자동으로 체크섬이 추가되므로 (크기 - 2) 바이트만 복사합니다. 
 *    즉, 우리의 변수는 2바이트 짧을 수 있지만 데이터 손실 없이 사용할 수 있습니다 (단, sizeof는 더 이상 작동하지 않으며 전체 프레임 길이를 dwt_writetxdata()에 표시해야 합니다).
 * 8. 여기에서는 가능한 한 예제를 단순하게 유지하기 위해 폴링 모드를 사용하지만, 모든 상태 이벤트는 인터럽트를 생성하는 데 사용할 수 있습니다. "인터럽트"에 대한 자세한 내용은 DW IC
 *    사용자 설명서를 참조하십시오. 또한 STATUS 레지스터는 5바이트 길이이지만, 우리가 사용하는 이벤트는 레지스터의 첫 번째 바이트에 모두 있기 때문에, 전체 5바이트를 읽는 대신
 *    간단한 dwt_read32bitreg() API 호출로 접근할 수 있습니다.
 * 9. 각 40비트 타임스탬프의 고위 바이트는 여기서 버려집니다. 이는 각 장치에서 타임스탬프 간 차이가 2**32 장치 시간 단위(약 67ms)를 넘지 않기 때문에,
 *    왕복 지연 계산을 32비트 빼기로 처리할 수 있기 때문에 허용됩니다.
 * 10. 사용자는 EVK1000 제품과 함께 배포된 DecaRanging ARM 응용 프로그램에서 추가적인 사용 예제를 참고하거나, DW IC API 가이드를 참조하여 DW IC 드라이버 함수에 대한 자세한 내용을 확인할 수 있습니다.
 * 11. TOF 계산을 수정하기 위해 클록 오프셋 값을 사용하는 것은, 원격 응답 장치의 클록이 로컬 Initiator 장치의 클록에서 PPM 차이를 가지고 있을 때 SS-TWR 결과를 현저하게 개선합니다.
 *     2번 참고: 안테나 지연이 보정되고 올바르게 설정되지 않으면 고정된 범위 오프셋이 나타날 수 있습니다.
 * 12. 이 예제에서는 dwt_initialise()를 호출한 후 DW IC가 IDLE 상태로 설정됩니다. 이후에는 최대 20 MHz의 빠른 SPI 속도를 사용할 수 있습니다.
 * 13. 이 예제에서는 프레임 카운터가 0으로 설정되어 있습니다. 실제로 프레임 카운터는 매 프레임마다 증가해야 합니다. 프레임 카운터가 최대값(uint32_t)에 도달하면,
 *     키를 교체해야 합니다.
 * 14. 사용자가 원하는 설정은 현재 프로그래밍된 설정과 다를 수 있습니다. dwt_configure는 원하는 설정을 설정하는 데 사용됩니다.
 * 15. CCM 코어 유형을 사용할 때는 AES_KEY_Load가 각 암호화/복호화 작업 전에 설정되어야 하며, AES KEY가 변경되지 않더라도 마찬가지입니다.
 ****************************************************************************************************************************************************/
