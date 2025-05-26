// ESP32 UWB DW3000 모듈을 사용한 암호화 통신 기반 3D 실내 위치 추적 앵커 코드
// 원본 코드: AES_ss_twr_responder_ver_802.15.4z_kor + 다중 앵커 지원 추가 + BLE 통신 + 최적화

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

// 애플리케이션 설정
#define APP_NAME "3D AES ANCHOR v1.0"
#define DEBUG_ENABLE 1  // 디버깅 메시지 활성화(1) 또는 비활성화(0)
#define POWER_SAVE 1    // 전력 절약 모드 활성화(1) 또는 비활성화(0)

// BLE 설정
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_ADV_INTERVAL 1600  // BLE 광고 간격 (1600*0.625ms = 1초)

// 하드웨어 설정
const uint8_t PIN_RST = 27;  // 리셋 핀
const uint8_t PIN_IRQ = 34;  // 인터럽트 핀
const uint8_t PIN_SS = 4;    // SPI 선택 핀

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

// 802.15.4 MAC 프레임 정의
mac_frame_802_15_4_format_t mac_frame;

// ------ 앵커 위치 및 ID 설정 ------
// 앵커 설정을 여기에서 변경하세요
// 형식: 0xFYYYnnnnnnnnnnnn (F: 층 코드 A=1층,B=2층,C=3층; YYY: 위치 코드)
#define ANCHOR_ID 0xA001000000000000  // 앵커 ID: 1-1 (1층 첫 번째 앵커)
#define ANCHOR_FLOOR 0                 // 0 = 1층, 1 = 2층, 2 = 3층
#define ANCHOR_X 0.0                   // X 좌표 (미터)
#define ANCHOR_Y 0.0                   // Y 좌표 (미터)
#define ANCHOR_Z 1.5                   // Z 좌표 (미터)

// 앵커 ID 참고 (층 코드: A=1층, B=2층, C=3층)
/* 
 * 1층 앵커 코드 예시:
 * 0xA001 = (0,0) / 0xA002 = (5,0) / 0xA003 = (0,5) / 0xA004 = (5,5)
 * 
 * 2층 앵커 코드 예시:
 * 0xB001 = (0,0) / 0xB002 = (5,0) / 0xB003 = (0,5)
 * 
 * 3층 앵커 코드 예시:
 * 0xC001 = (0,0) / 0xC002 = (5,0) / 0xC003 = (0,5)
 */

// 이 앵커(Responder)의 주소 정의
#define SRC_ADDR ANCHOR_ID   // 앵커 ID를 주소로 사용
#define DEST_PAN_ID 0x4321   // PAN ID (네트워크 식별자)

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

/* 64 MHz PRF(펄스 반복 주파수)에 대한 기본 안테나 지연값 */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* AUX 보안 헤더의 키 인덱스에 따른 선택적 키 옵션 */
static dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

/* 거리 측정 과정에서 사용되는 MAC 페이로드 데이터 */
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

/* 수신된 응답 메시지를 저장할 버퍼 */
#define RX_BUF_LEN 127 /* STD PHR 모드가 사용될 경우, 수신된 프레임 크기는 127바이트를 초과할 수 없음 */
static uint8_t rx_buffer[RX_BUF_LEN];

/* 이 예제에서는 리스폰더 데이터 암호화를 위해 키 테이블에서 인덱스 2를 사용함 */
#define RESPONDER_KEY_INDEX 2

/* 프레임 간 지연 시간(UWB 마이크로초 단위) */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

/* 프레임 송수신 타임스탬프 */
static uint64_t poll_rx_ts; // Poll 메시지 수신 타임스탬프
static uint64_t resp_tx_ts; // Response 메시지 전송 타임스탬프

/* PG_DELAY 및 TX_POWER 레지스터 값 */
extern dwt_txconfig_t txconfig_options;

// 전역 변수
dwt_aes_job_t aes_job_rx, aes_job_tx; // AES 암호화/복호화 작업 구조체
int8_t status;                        // 현재 상태 변수
uint32_t status_reg;                  // DW3000 상태 레지스터 값
unsigned long lastActiveTime = 0;     // 마지막 활동 시간 (저전력 모드용)
unsigned long inactiveCount = 0;      // 비활동 카운터 (저전력 모드용)

// 디버그 출력 함수 - DEBUG_ENABLE=0이면 출력 안함
void debug_print(const char* msg) {
  #if DEBUG_ENABLE
  Serial.println(msg);
  #endif
}

void debug_printf(const char* format, ...) {
  #if DEBUG_ENABLE
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.println(buffer);
  #endif
}

// 앵커 정보 출력 함수
void printAnchorInfo() {
  char info[100];
  uint32_t id_high = (uint32_t)(ANCHOR_ID >> 32); // 상위 32비트만 출력 (더 간결한 ID)
  
  snprintf(info, sizeof(info), "앵커 ID: 0x%08X, 층: %d, 위치: X=%.1f, Y=%.1f, Z=%.1f", 
          id_high, ANCHOR_FLOOR + 1, ANCHOR_X, ANCHOR_Y, ANCHOR_Z);
  
  Serial.println(info);
  test_run_info((unsigned char *)"초기화 완료, 태그 신호 대기 중...");
}

// 저전력 모드 처리 함수
void handlePowerSave() {
  #if POWER_SAVE
  // 일정 시간 동안 활동이 없으면 짧은 슬립
  unsigned long currentTime = millis();
  if (currentTime - lastActiveTime > 5000) { // 5초 이상 비활동
    // 경량 슬립 - 다음 태그 요청까지 배터리 절약
    esp_sleep_enable_timer_wakeup(20000); // 20ms sleep
    esp_light_sleep_start();
    inactiveCount++;
    
    // 30분마다 상태 메시지 출력 (5초*6*60=1800초=30분)
    if (inactiveCount % 360 == 0) {
      debug_printf("앵커 0x%08X 대기 중... %u분 경과", 
                   (uint32_t)(ANCHOR_ID >> 32), inactiveCount/6);
    }
  }
  #endif
}

// UWB 초기화 함수
void initUWB() {
  /* SPI 속도 설정, DW3000은 최대 38MHz까지 지원 */  
  spiBegin(PIN_IRQ, PIN_RST);  
  spiSelect(PIN_SS);  

  delay(2);  // DW3000이 시작할 시간을 제공

  int idle_retry = 0;
  while (!dwt_checkidlerc()) {  // DW IC가 IDLE_RC 상태인지 확인
    debug_print("IDLE 상태 확인 실패");
    delay(1000);
    if (++idle_retry > 5) {
      ESP.restart(); // 5번 실패시 재시작
    }
  }  

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {  
    debug_print("DW3000 초기화 실패");
    ESP.restart();
  }  

  // LED는 디버그 모드에서만 활성화
  #if DEBUG_ENABLE
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  #endif

  if (dwt_configure(&config)) {
    debug_print("DW3000 설정 실패");
    ESP.restart();
  }  

  /* 기본 설정 적용 */
  dwt_configuretxrf(&txconfig_options);    // TX 스펙트럼 매개변수 구성
  dwt_setrxantennadelay(RX_ANT_DLY);       // RX 안테나 지연 설정  
  dwt_settxantennadelay(TX_ANT_DLY);       // TX 안테나 지연 설정  
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);  // TX/RX 상태 출력 활성화
  
  /* AES 작업 구성 */
  // RX 복호화 작업 설정
  aes_job_rx.mode = AES_Decrypt;
  aes_job_rx.src_port = AES_Src_Rx_buf_0;
  aes_job_rx.dst_port = AES_Dst_Rx_buf_0;
  aes_job_rx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
  aes_job_rx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);
  aes_job_rx.payload = rx_buffer;

  // TX 암호화 작업 설정
  aes_job_tx.mode = AES_Encrypt;
  aes_job_tx.src_port = AES_Src_Tx_buf;
  aes_job_tx.dst_port = AES_Dst_Tx_buf;
  aes_job_tx.header_len = aes_job_rx.header_len;
  aes_job_tx.header = aes_job_rx.header;
  aes_job_tx.payload = tx_resp_msg;
  aes_job_tx.payload_len = sizeof(tx_resp_msg);
}

// BLE 초기화 함수
void initBLE() {
  char idString[20];
  sprintf(idString, "0x%llX", SRC_ADDR);
  
  BLEDevice::init(idString);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE
  );

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  
  // 광고 간격 설정 (전력 소모 최적화)
  pAdvertising->setMinInterval(BLE_ADV_INTERVAL);
  pAdvertising->setMaxInterval(BLE_ADV_INTERVAL * 2);
  
  BLEDevice::startAdvertising();
  
  debug_printf("BLE 광고 시작 - 앵커 ID: %s", idString);
}

void setup() {
  Serial.begin(115200);
  UART_init();  
  
  Serial.println();
  Serial.println(APP_NAME);
  test_run_info((unsigned char *)APP_NAME);  

  // UWB 초기화
  initUWB();
  
  // 앵커 정보 출력
  printAnchorInfo();
  
  // BLE 초기화
  initBLE();
  
  // 초기 활동 시간 설정
  lastActiveTime = millis();
}

// 태그 폴 메시지 처리 함수
void processPollMessage(uint64_t source_addr) {
  uint32_t resp_tx_time;
  int ret;
  uint8_t nonce[13];
  
  // 활동 시간 갱신
  lastActiveTime = millis();
  inactiveCount = 0;
  
  // 태그 ID 출력 (디버그 모드에서만)
  debug_printf("태그 0x%08X로부터 Poll 수신", (uint32_t)(source_addr >> 32));

  // Poll 수신 타임스탬프 획득
  poll_rx_ts = get_rx_timestamp_u64();

  // 응답 메시지 전송 시간 계산
  resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
  dwt_setdelayedtrxtime(resp_tx_time);

  // 응답 TX 타임스탬프 계산 (프로그래밍된 전송 시간 + 안테나 지연)
  resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  // 메시지에 타임스탬프 데이터 추가
  resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
  resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

  // 암호화 준비
  dwt_set_keyreg_128(&keys_options[RESPONDER_KEY_INDEX - 1]);  // 키 설정
  MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = RESPONDER_KEY_INDEX;  // 키 인덱스 설정
  MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame)++;  // 시퀀스 번호 증가
  mac_frame_update_aux_frame_cnt(&mac_frame, mac_frame_get_aux_frame_cnt(&mac_frame) + 1);  // 프레임 카운트 업데이트

  // AES 작업 설정
  aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
  aes_job_tx.nonce = nonce;
  aes_config.mode = AES_Encrypt;
  aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
  dwt_configure_aes(&aes_config);

  // MAC 헤더 업데이트 (소스/목적지 주소 변경)
  mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, source_addr, SRC_ADDR);

  // Nonce 생성
  mac_frame_get_nonce(&mac_frame, nonce);

  // 암호화 수행
  status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
  
  if (status < 0 || (status & AES_ERRORS)) {
    debug_print("응답 암호화 실패");
    return;
  }
  
  // 프레임 전송 준비 및 시작
  dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1);
  ret = dwt_starttx(DWT_START_TX_DELAYED);

  if (ret == DWT_SUCCESS) {
    // 전송 완료 대기 (타임아웃 추가)
    unsigned long tx_start = millis();
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
      // 100ms 이상 대기하면 오류로 간주
      if (millis() - tx_start > 100) {
        debug_print("TX 타임아웃");
        break;
      }
    }

    // 전송 완료 이벤트 클리어
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    debug_printf("태그 0x%08X에 응답 송신 완료", (uint32_t)(source_addr >> 32));
  } else {
    debug_print("TX 시작 실패");
  }
}

// 주 루프 함수
void loop() {
  // 저전력 모드 처리
  handlePowerSave();
  
  // UWB 수신 활성화
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // 타임아웃 설정 (폴링 시간 제한)
  unsigned long rx_start = millis();
  bool rx_timeout = false;
  
  // 프레임 수신 또는 오류 감지 대기 (최대 100ms)
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
    // 100ms 이상 기다렸으면 타임아웃
    if (millis() - rx_start > 100) {
      rx_timeout = true;
      break;
    }
  }
  
  // 타임아웃이면 다시 시작
  if (rx_timeout) {
    return;
  }

  // 프레임 수신 성공 처리
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;
    uint64_t source_addr = 0;

    // 상태 레지스터 클리어
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    // 수신 데이터 길이 확인
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    // 복호화 준비
    aes_config.mode = AES_Decrypt;
    PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer;

    // AES 복호화 수행
    status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), 
                           keys_options, SRC_ADDR, 0, &aes_config);

    // 송신자 주소 추출
    uint8_t *mhr_ptr = (uint8_t *)MHR_802_15_4_PTR(&mac_frame);
    uint8_t *src_addr_ptr = mhr_ptr + 7; // 소스 주소 위치
    
    // 64비트 주소 조합 (리틀 엔디안)
    for (int i = 0; i < 8; i++) {
      if (src_addr_ptr) {
        source_addr |= ((uint64_t)src_addr_ptr[i]) << (i * 8);
      }
    }

    // 복호화 성공 확인
    if (status == AES_RES_OK) {
      // 메시지 유형 확인 (Poll 메시지인지)
      if (memcmp(rx_buffer, rx_poll_msg, aes_job_rx.payload_len) == 0) {
        // Poll 메시지 처리
        processPollMessage(source_addr);
      } else {
        debug_print("예상과 다른 페이로드 수신");
      }
    } else {
      // 오류 처리 (디버그 모드에서만)
      #if DEBUG_ENABLE
      switch (status) {
        case AES_RES_ERROR_LENGTH:
          debug_print("AES 길이 오류");
          break;
        case AES_RES_ERROR:
          debug_print("AES 오류");
          break;
        case AES_RES_ERROR_FRAME:
          debug_print("프레임 오류");
          break;
        case AES_RES_ERROR_IGNORE_FRAME:
          debug_print("대상이 아닌 프레임");
          break;
      }
      #endif
    }
  } else {
    // 수신 오류 이벤트 클리어
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}