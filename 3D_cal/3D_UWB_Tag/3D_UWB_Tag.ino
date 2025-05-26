// ESP32 UWB DW3000 모듈을 사용한 암호화 통신 기반 3D 실내 위치 추적 및 BLE 전송 코드
// 원본 코드: AES_ss_twr_initiator_ver_802.15.4z_BLE_kor + tag_multi_floor + 동적 BLE 앵커 탐색

#include <cmath>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>
#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

// 유효한 거리 측정 카운트 변수 (전역으로 이동)
int valid_ranges_count = 0;

// 디버그 출력 함수 (DEBUG_ENABLE=0이면 출력 안함)
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

// 애플리케이션 설정
#define APP_NAME "AES 3D POSITIONING v1.0"
#define DEBUG_ENABLE 1     // 디버깅 메시지 활성화(1) 또는 비활성화(0)
#define POWER_SAVE 1       // 전력 절약 모드 활성화(1) 또는 비활성화(0)
#define SCAN_INTERVAL 10000 // BLE 스캔 간격 (ms)
#define SCAN_DURATION 3000  // BLE 스캔 지속 시간 (ms)

// BLE 서비스 및 특성 UUID
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// 앵커 정의
#define NUM_ANCHORS 10 // 앵커 총 개수

// 3차원 좌표를 나타내는 구조체
struct Point {
  float x;
  float y;
  float z;
};

// 앵커 데이터를 저장하는 구조체
struct AnchorData {
  uint64_t addr;      // 앵커 주소 (64비트)
  Point pos;         // 앵커 위치 (x, y, z)
  double distance;   // 마지막으로 측정된 거리
  bool distance_valid; // 마지막 측정이 성공적이었는지 여부
  uint64_t last_heard_ts; // 마지막 성공적인 거리 측정 타임스탬프
  uint8_t floor;     // 앵커가 설치된 층 (0=1층, 1=2층, 2=3층...)
};

// 앵커 배열 선언
AnchorData anchors[NUM_ANCHORS] = {
  // 1층 앵커 (1층 = 0) - 주소 형식: 0xA001xxxxxxxxxxxx
  {0xA001000000000000, {0.0, 0.0, 1.5}, -1.0, false, 0, 0},     // 앵커 1 (1층)
  {0xA002000000000000, {5.0, 0.0, 1.5}, -1.0, false, 0, 0},     // 앵커 2 (1층)
  {0xA003000000000000, {0.0, 5.0, 1.5}, -1.0, false, 0, 0},     // 앵커 3 (1층)
  {0xA004000000000000, {5.0, 5.0, 1.5}, -1.0, false, 0, 0},     // 앵커 4 (1층)

  // 2층 앵커 (2층 = 1)
  {0xB001000000000000, {0.0, 0.0, 4.5}, -1.0, false, 0, 1},     // 앵커 5 (2층)
  {0xB002000000000000, {5.0, 0.0, 4.5}, -1.0, false, 0, 1},     // 앵커 6 (2층)
  {0xB003000000000000, {0.0, 5.0, 4.5}, -1.0, false, 0, 1},     // 앵커 7 (2층)

  // 3층 앵커 (3층 = 2)
  {0xC001000000000000, {0.0, 0.0, 7.5}, -1.0, false, 0, 2},     // 앵커 8 (3층)
  {0xC002000000000000, {5.0, 0.0, 7.5}, -1.0, false, 0, 2},     // 앵커 9 (3층)
  {0xC003000000000000, {0.0, 5.0, 7.5}, -1.0, false, 0, 2}      // 앵커 10 (3층)
};

BLECharacteristic *pCharacteristic;
BLEScan* pBLEScan;
BLEAdvertisedDevice* myDevice;
bool deviceConnected = false;
unsigned long lastScanTime = 0;  // 마지막 BLE 스캔 시각

// BLE 서버 콜백 클래스
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    debug_print("BLE 연결됨");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    debug_print("BLE 연결 해제됨");
    delay(500);
    pServer->getAdvertising()->start();
    debug_print("클라이언트 연결 대기 중...");
  }
};

// BLE 광고 패킷 수신 콜백 클래스
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // MAC 주소 가져오기
    uint8_t* addr = (uint8_t*)advertisedDevice.getAddress().getNative();
    
    // 앵커 장치 필터링 - 모든 ESP32 장치 확인 (AC:XX:XX 패턴)
    if (addr[0] == 0xAC) {
      // 장치 이름 확인
      String deviceName = advertisedDevice.getName();
      
      // 0x로 시작하는 16진수 문자열 이름 확인 (앵커 주소 형식)
      if (deviceName.startsWith("0x") && deviceName.length() == 18) {
        debug_printf("UWB 앵커 발견: %s", deviceName.c_str());
        
        // 디바이스 이름에서 16진수 문자열 추출 (0x 이후 부분)
        const char* hexStr = deviceName.c_str() + 2;
        uint64_t anchorAddr = strtoull(hexStr, NULL, 16);
        
        // 앵커 아이디 코드 추출 (상위 32비트)
        uint32_t anchorId = (uint32_t)(anchorAddr >> 32);
        
        // 앵커 코드의 첫 문자로 층 확인 (A=1층, B=2층, C=3층)
        char floorCode = ((char*)&anchorId)[3]; // 빅 엔디안 기준 첫 바이트
        
        // 발견된 앵커 정보 출력
        debug_printf("앵커 ID: 0x%08X (주소: 0x%016llX)", 
                     anchorId, (unsigned long long)anchorAddr);
        
        // 등록된 앵커 목록에서 해당 앵커 찾기
        bool found = false;
        for (int i = 0; i < NUM_ANCHORS; i++) {
          if (anchors[i].addr == anchorAddr) {
            debug_printf("기존 등록 앵커 발견. 인덱스: %d, 층: %d", 
                         i, anchors[i].floor + 1);
            found = true;
            break;
          }
        }
        
        if (!found) {
          debug_printf("등록되지 않은 새 앵커 발견: 0x%08X", anchorId);
        }
        
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
      }
    }
  }
};


// BLE 특성 콜백 클래스
class CharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *dhtCharacteristic) {
    String value = pCharacteristic->getValue();
    debug_printf("수신된 값: %s", value.c_str());
  }
};

// 하드웨어 연결 핀 정의
const uint8_t PIN_RST = 27;  // 리셋 핀
const uint8_t PIN_IRQ = 34;  // 인터럽트 핀
const uint8_t PIN_SS = 4;    // SPI 선택 핀

// UWB 통신 기본 설정
static dwt_config_t config = {
  5,                // 채널 번호
  DWT_PLEN_128,     // 프리앰블 길이 (TX 전용)
  DWT_PAC8,         // 프리앰블 수신 청크 크기 (RX 전용)
  9,                // TX 프리앰블 코드
  9,                // RX 프리앰블 코드
  1,                // SFD 타입: 비표준 8 심볼 SFD
  DWT_BR_6M8,       // 데이터 전송 속도: 6.8Mbps
  DWT_PHRMODE_STD,  // PHY 헤더 모드: 표준
  DWT_PHRRATE_STD,  // PHY 헤더 속도: 표준
  (129 + 8 - 8),    // SFD 타임아웃 설정 
  DWT_STS_MODE_OFF, // STS 비활성화
  DWT_STS_LEN_64,   // STS 길이
  DWT_PDOA_M0       // PDOA 모드 비활성화
};

// 거리 측정 간격 및 안테나 지연 설정
#define RNG_DELAY_MS 100             // 거리 측정 간 딜레이 (밀리초)
#define INTRA_RANGING_DELAY_MS 30    // 다른 앵커 측정 사이의 짧은 지연
#define TX_ANT_DLY 16385             // TX 안테나 지연 값
#define RX_ANT_DLY 16385             // RX 안테나 지연 값

// 802.15.4 MAC 프레임 정의
mac_frame_802_15_4_format_t mac_frame = {
  { { 0x09, 0xEC },      // 프레임 제어: 데이터 프레임, 보안 활성화
    0x00,                // 시퀀스 번호 초기값
    { 0x21, 0x43 },      // 목적지 PAN ID
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 목적지 주소
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 소스 주소
    { 0x0F, { 0x00, 0x00, 0x00, 0x00 }, 0x00 } },       // 보안 헤더
  0x00  // 페이로드 길이
};

// AES 암호화 설정
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

// 통신 주소 정의
#define DEST_PAN_ID 0x4321           // 사용할 PAN ID
#define SRC_ADDR 0x8877665544332211  // Initiator(발신자)의 주소

// AES 키 옵션 정의 (키 인덱스에 따라 선택)
#define NUM_OF_KEY_OPTIONS 3  // 키 옵션의 수
static dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

// 메시지 정의
static uint8_t tx_poll_msg[] = { 'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e' };
static uint8_t rx_resp_msg[] = { 0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e' };

// 메시지의 특정 위치 정의
#define START_RECEIVE_DATA_LOCATION 8  // 응답 메시지에서 데이터 시작 위치
#define ALL_MSG_SN_IDX 2               // 시퀀스 번호 위치
#define RESP_MSG_POLL_RX_TS_IDX 0      // Poll RX 타임스탬프 위치
#define RESP_MSG_RESP_TX_TS_IDX 4      // Response TX 타임스탬프 위치
#define RESP_MSG_TS_LEN 4              // 타임스탬프 길이

// 키 인덱스 설정
#define INITIATOR_KEY_INDEX 1  // Initiator의 암호화 키 인덱스

// 수신 버퍼 크기
#define RX_BUF_LEN 127  // 최대 프레임 크기
static uint8_t rx_buffer[RX_BUF_LEN];

// 통신 타이밍 설정
#define POLL_TX_TO_RESP_RX_DLY_UUS 1720  // Poll 전송 후 응답 수신까지 지연 시간
#define RESP_RX_TIMEOUT_UUS 250          // 응답 수신 타임아웃

// 결과 저장 변수
static double tof;      // 비행 시간(Time of Flight)
static double distance; // 계산된 거리
static char txString[100]; // BLE로 전송할 문자열

// 에러 처리를 위한 상수
#define MAX_ERROR_RETRY 3  // 최대 재시도 횟수

// --- 3D 위치 추적 관련 변수 ---

// 계산된 위치 결과를 저장하는 구조체
Point current_position = {0.0, 0.0, 0.0};
bool position_valid = false; // 위치 계산 성공 여부

// 층 결정 변수
uint8_t current_floor = 0; // 현재 결정된 층 (0=1층, 1=2층, 2=3층...)
uint8_t floor_change_count = 0; // 층 변경 안정화 카운터
uint8_t floor_vote_threshold = 3; // 층 변경을 위한 카운터 임계값

// 외부 참조 변수
extern dwt_txconfig_t txconfig_options;

// 카운터 및 상태 변수
static uint32_t frame_cnt = 0;  // 프레임 카운터
static uint8_t seq_cnt = 0x0A;  // 시퀀스 카운터
uint32_t status_reg;            // 상태 레지스터
uint8_t nonce[13];              // AES nonce (13바이트)
dwt_aes_job_t aes_job_tx, aes_job_rx; // AES 작업 정의
int8_t status;                  // 상태 코드
int error_count = 0;            // 에러 카운터

// --- 함수 프로토타입 ---
void findNearestThreeAnchors(AnchorData* nearest[3]); // 가장 가까운 앵커 3개 찾기
bool calculatePosition(AnchorData* a1, AnchorData* a2, AnchorData* a3, Point& result_pos); // 위치 계산 (삼각측량)
void determineFloor(); // 현재 층 결정
void serialPrintPosition(); // 위치 정보 출력 함수
void sendPositionViaBLE(); // BLE를 통해 위치 정보 전송

// BLE 초기화 함수
void BLEInitialize() {
  debug_print("BLE 초기화 시작...");

  // 디바이스 이름 설정
  BLEDevice::init("ESP32-UWB-3D-POS");

  // BLE 서버 설정 (결과를 모바일 앱으로 전송)
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | 
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new CharacteristicCallback());

  pService->start();

  // BLE 광고 설정
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  // BLE 스캔 설정 (앵커 발견용)
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);  // 1349ms마다 스캔 루프 실행
  pBLEScan->setWindow(449);     // 449ms동안 스캔
  pBLEScan->setActiveScan(true);  // 활성 스캔 (장치 이름 등 추가 정보 요청)

  pServer->getAdvertising()->start();
  debug_print("BLE 초기화 완료, 클라이언트 연결 대기 중...");
}

// 시스템 초기화 및 설정
void setup() {
  BLEInitialize();  // BLE 초기화
  UART_init();      // UART 초기화
  test_run_info((unsigned char *)APP_NAME);

  // SPI 초기화 및 DW3000 연결
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  // DW3000 시작 대기
  delay(2);  // DW3000 시작 시간 (IDLE_RC 상태로 전환)

  // DW3000 IDLE 상태 확인
  int idle_check_count = 0;
  while (!dwt_checkidlerc()) {
    idle_check_count++;
    if (idle_check_count > 10) {
      Serial.println("DW3000 IDLE 상태 확인 실패");
      delay(1000);
      idle_check_count = 0;
    }
  }

  // DW3000 초기화
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("DW3000 초기화 실패");
    delay(1000);
    ESP.restart();  // 오류 시 재시작
  }

  // LED 활성화 (디버깅용)
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // DW3000 설정
  if (dwt_configure(&config)) {
    Serial.println("DW3000 설정 실패");
    delay(1000);
    ESP.restart();  // 오류 시 재시작
  }

  // TX 스펙트럼 설정
  dwt_configuretxrf(&txconfig_options);

  // 안테나 지연 설정
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  // 통신 타이밍 설정
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  // TX/RX 상태 출력 활성화 (디버깅용)
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // AES 작업 설정
  // TX 작업 - Poll 메시지 암호화
  aes_job_tx.mode = AES_Encrypt;                               // 암호화 모드
  aes_job_tx.src_port = AES_Src_Tx_buf;                        // 소스: TX 버퍼
  aes_job_tx.dst_port = AES_Dst_Tx_buf;                        // 목적지: TX 버퍼
  aes_job_tx.nonce = nonce;                                    // nonce 포인터
  aes_job_tx.header = (uint8_t *)MHR_802_15_4_PTR(&mac_frame); // 헤더 포인터
  aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);   // 헤더 길이
  aes_job_tx.payload = tx_poll_msg;                            // 페이로드 포인터
  aes_job_tx.payload_len = sizeof(tx_poll_msg);                // 페이로드 길이

  // RX 작업 - Response 메시지 복호화
  aes_job_rx.mode = AES_Decrypt;          // 복호화 모드
  aes_job_rx.src_port = AES_Src_Rx_buf_0; // 소스: RX 버퍼 0
  aes_job_rx.dst_port = AES_Dst_Rx_buf_0; // 목적지: RX 버퍼 0
  aes_job_rx.header_len = aes_job_tx.header_len;
  aes_job_rx.header = aes_job_tx.header;  // 헤더 포인터
  aes_job_rx.payload = rx_buffer;         // 복호화된 데이터 저장 위치

  Serial.println("초기화 완료");
}

// 저전력 모드 처리 함수
void handlePowerSave() {
  #if POWER_SAVE
  static unsigned long lastActivityTime = 0;
  static int inactiveCount = 0;
  
  // 5초 이상 비활동 상태일 때 경량 슬립 모드로 전환
  if (millis() - lastActivityTime > 5000) {
    // 짧은 슬립으로 배터리 절약
    esp_sleep_enable_timer_wakeup(20000); // 20ms sleep
    esp_light_sleep_start();
    inactiveCount++;
    
    // 30분마다 상태 출력 (5초*6*60=1800초)
    if (inactiveCount % 360 == 0) {
      debug_printf("태그 대기 중... %u분 경과", inactiveCount/6);
      // 비활동 상태가 길면 더 긴 슬립 사용
      if (inactiveCount > 3600) { // 5시간 이상 비활동
        esp_sleep_enable_timer_wakeup(500000); // 500ms 슬립
      }
    }
  } else {
    // 활동 있으면 카운터 초기화
    inactiveCount = 0;
  }
  #endif
}

// 메인 루프
void loop() {
  // BLE 스캔 주기적 실행
  unsigned long currentMillis = millis();
  if (currentMillis - lastScanTime >= SCAN_INTERVAL) {
    lastScanTime = currentMillis;
    debug_print("BLE 스캔 시작...");
    pBLEScan->start(SCAN_DURATION / 1000, false);  // 비동기 스캔
  }
  
  // 저전력 모드 처리
  handlePowerSave();
  
  valid_ranges_count = 0; // 유효한 거리 측정 횟수 카운트 초기화
  
  // 모든 앵커와 거리 측정
  for (int i = 0; i < NUM_ANCHORS; i++) {
    // 앵커 거리 측정 전 초기화
    anchors[i].distance_valid = false;
    anchors[i].distance = -1.0;
    
    // 대상 앵커 주소 설정
    uint64_t DEST_ADDR = anchors[i].addr;
    
    // 키 설정
    dwt_set_keyreg_128(&keys_options[INITIATOR_KEY_INDEX - 1]);
    // 프레임의 키 인덱스 설정
    MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = INITIATOR_KEY_INDEX;

    // MAC 프레임 주소 업데이트 및 nonce 생성
    mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, DEST_ADDR, SRC_ADDR);
    mac_frame_get_nonce(&mac_frame, nonce);

    // AES 암호화 설정
    aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
    aes_config.mode = AES_Encrypt;
    aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
    dwt_configure_aes(&aes_config);

    // Poll 메시지 암호화 및 전송
    status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
    
    // 암호화 에러 처리
    if (status < 0) {
      Serial.println("AES 길이 에러");
      error_count++;
      if (error_count > MAX_ERROR_RETRY) {
        Serial.println("최대 에러 횟수 초과. 재시작합니다.");
        ESP.restart();
      }
      delay(100);
      continue;  // 다음 앵커로 이동
    } else if (status & AES_ERRORS) {
      Serial.println("AES 에러");
      error_count++;
      if (error_count > MAX_ERROR_RETRY) {
        Serial.println("최대 에러 횟수 초과. 재시작합니다.");
        ESP.restart();
      }
      delay(100);
      continue;  // 다음 앵커로 이동
    }
    
    // 에러가 없으면 에러 카운터 초기화
    error_count = 0;

    // 프레임 컨트롤 설정 및 전송 준비
    dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1);

    // Poll 메시지 전송 시작
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // 응답 수신 또는 타임아웃 대기
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
            (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
      // 타임아웃을 위한 안전장치
      delay(1);
    }

    // 시퀀스 번호 및 프레임 카운터 증가
    MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame) = ++seq_cnt;
    mac_frame_update_aux_frame_cnt(&mac_frame, ++frame_cnt);

    // 응답 메시지 처리
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) { // 응답 수신 성공
      uint32_t frame_len;

      // 상태 레지스터 클리어
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

      // 수신 데이터 길이 확인
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

      // 응답 메시지 복호화
      aes_config.mode = AES_Decrypt;
      PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer;

      // AES 복호화 수행
      status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, 
                              sizeof(rx_buffer), keys_options, DEST_ADDR, SRC_ADDR, &aes_config);
      
      // 복호화 결과 확인
      if (status != AES_RES_OK) {
        Serial.print("AES 복호화 에러: ");
        
        switch (status) {
          case AES_RES_ERROR_LENGTH:
            Serial.println("길이 에러");
            break;
          case AES_RES_ERROR:
            Serial.println("AES 에러");
            break;
          case AES_RES_ERROR_FRAME:
            Serial.println("프레임 에러");
            break;
          case AES_RES_ERROR_IGNORE_FRAME:
            Serial.println("대상이 아닌 프레임");
            break;
          default:
            Serial.println("알 수 없는 에러");
        }
        
        // 에러 카운트 증가 및 처리
        error_count++;
        if (error_count > MAX_ERROR_RETRY) {
          Serial.println("최대 에러 횟수 초과");
          error_count = 0;  // 에러 카운터 초기화
        }
      } else {
        // 정상 복호화된 경우 - 응답 메시지 검증
        if (memcmp(&rx_buffer[START_RECEIVE_DATA_LOCATION], 
                  &rx_resp_msg[START_RECEIVE_DATA_LOCATION],
                  aes_job_rx.payload_len - START_RECEIVE_DATA_LOCATION) == 0) {
          
          uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
          int32_t rtd_init, rtd_resp;
          float clockOffsetRatio;

          // 타임스탬프 읽기
          poll_tx_ts = dwt_readtxtimestamplo32();
          resp_rx_ts = dwt_readrxtimestamplo32();

          // 클록 오프셋 비율 계산
          clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

          // 응답 메시지에서 타임스탬프 추출
          resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
          resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

          // 왕복 지연 시간 계산
          rtd_init = resp_rx_ts - poll_tx_ts;
          rtd_resp = resp_tx_ts - poll_rx_ts;

          // 비행 시간(ToF) 및 거리 계산
          tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
          distance = tof * SPEED_OF_LIGHT;

          // 현재 앵커에 대한 결과 저장
          anchors[i].distance = distance;
          anchors[i].distance_valid = true;
          anchors[i].last_heard_ts = dwt_readsystimestamphi32();
          valid_ranges_count++;

          // 계산된 거리 출력
          char dist_str[50];
          snprintf(dist_str, sizeof(dist_str), "앵커 0x%llX (층:%d): %.2f m", 
                  anchors[i].addr, anchors[i].floor + 1, anchors[i].distance);
          Serial.println(dist_str);
        } else {
          Serial.println("예상과 다른 응답 메시지 수신");
        }
      }
    } else {
      // 응답 수신 실패 또는 타임아웃
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      Serial.print("앵커 0x");
      Serial.print((uint32_t)(anchors[i].addr >> 32), HEX);
      Serial.println(": 응답 수신 실패 또는 타임아웃");
    }

    // 다른 앵커와의 거리 측정 시도 사이에 짧은 지연
    if (i < NUM_ANCHORS - 1) {
      delay(INTRA_RANGING_DELAY_MS);
    }
  }

  // --- 위치 계산 ---
  if (valid_ranges_count >= 3) { // 최소 3개의 유효한 거리가 있어야 삼각측량 가능
    AnchorData* nearest[3] = {nullptr, nullptr, nullptr}; // 가장 가까운 앵커 3개를 저장할 포인터 배열
    findNearestThreeAnchors(nearest); // 가장 가까운 3개 앵커 찾기

    if (nearest[0] && nearest[1] && nearest[2]) { // 3개의 앵커를 성공적으로 찾았다면
      Serial.println("가장 가까운 앵커 3개 발견. 위치 계산 중...");
      position_valid = calculatePosition(nearest[0], nearest[1], nearest[2], current_position); // 위치 계산

      if (position_valid) { // 위치 계산 성공
        // 층 결정 알고리즘 실행
        determineFloor();
        
        // 위치 정보 출력 (시리얼)
        serialPrintPosition();
        
        // BLE로 위치 정보 전송
        sendPositionViaBLE();
      } else { // 위치 계산 실패 (기하학적/수학적 오류)
        Serial.println("삼각측량 실패 (기하학적/수학적 오류).");
      }
    } else {
      Serial.println("오류: 유효 범위 >=3개지만 가장 가까운 3개 선택 불가.");
      position_valid = false;
    }
  } else { // 유효한 거리가 3개 미만인 경우
    Serial.print("삼각측량 위해 유효 범위 >=3개 필요, ");
    Serial.print(valid_ranges_count);
    Serial.println("개 발견.");
    position_valid = false;
  }

  // 거리 측정 간 딜레이
  delay(RNG_DELAY_MS);
}

/*
 * @brief 가장 적합한 앵커 3개를 찾습니다.
 *        현재 층의 앵커를 우선 선택하고, 부족한 경우 다른 층의 앵커를 사용합니다.
 */
void findNearestThreeAnchors(AnchorData* nearest[3]) {
  // 포인터 초기화
  nearest[0] = nearest[1] = nearest[2] = nullptr;
  
  // 층별 앵커 목록 만들기
  AnchorData* current_floor_anchors[NUM_ANCHORS]; // 현재 층 앵커
  AnchorData* other_floor_anchors[NUM_ANCHORS];   // 다른 층 앵커
  double current_floor_dists[NUM_ANCHORS];        // 현재 층 앵커 거리
  double other_floor_dists[NUM_ANCHORS];          // 다른 층 앵커 거리
  
  int current_floor_count = 0; // 현재 층의 유효한 앵커 수
  int other_floor_count = 0;   // 다른 층의 유효한 앵커 수
  
  // 앵커를 층별로 분류
  for (int i = 0; i < NUM_ANCHORS; ++i) {
    if (anchors[i].distance_valid && anchors[i].distance >= 0) {
      if (anchors[i].floor == current_floor) {
        // 현재 층의 앵커
        current_floor_anchors[current_floor_count] = &anchors[i];
        current_floor_dists[current_floor_count] = anchors[i].distance;
        current_floor_count++;
      } else {
        // 다른 층의 앵커
        other_floor_anchors[other_floor_count] = &anchors[i];
        other_floor_dists[other_floor_count] = anchors[i].distance;
        other_floor_count++;
      }
    }
  }
  
  // 현재 층 앵커 정렬 (삽입 정렬 - 거리 기준)
  for (int i = 1; i < current_floor_count; i++) {
    AnchorData* key_anchor = current_floor_anchors[i];
    double key_dist = current_floor_dists[i];
    int j = i - 1;
    
    while (j >= 0 && current_floor_dists[j] > key_dist) {
      current_floor_anchors[j + 1] = current_floor_anchors[j];
      current_floor_dists[j + 1] = current_floor_dists[j];
      j--;
    }
    current_floor_anchors[j + 1] = key_anchor;
    current_floor_dists[j + 1] = key_dist;
  }
  
  // 다른 층 앵커 정렬 (삽입 정렬 - 거리 기준)
  for (int i = 1; i < other_floor_count; i++) {
    AnchorData* key_anchor = other_floor_anchors[i];
    double key_dist = other_floor_dists[i];
    int j = i - 1;
    
    while (j >= 0 && other_floor_dists[j] > key_dist) {
      other_floor_anchors[j + 1] = other_floor_anchors[j];
      other_floor_dists[j + 1] = other_floor_dists[j];
      j--;
    }
    other_floor_anchors[j + 1] = key_anchor;
    other_floor_dists[j + 1] = key_dist;
  }
  
  // 현재 층의 앵커를 우선적으로 사용
  int nearest_count = 0;
  
  // 1. 현재 층의 앵커 추가
  for (int i = 0; i < current_floor_count && nearest_count < 3; i++) {
    nearest[nearest_count++] = current_floor_anchors[i];
  }
  
  // 2. 부족한 경우 다른 층의 앵커로 채움
  for (int i = 0; i < other_floor_count && nearest_count < 3; i++) {
    nearest[nearest_count++] = other_floor_anchors[i];
  }
  
  // 선택된 앵커 디버그 출력
  for (int i = 0; i < 3; i++) {
    if (nearest[i]) {
      Serial.print("선택된 앵커 ");
      Serial.print(i+1);
      Serial.print(": 0x");
      Serial.print((uint32_t)(nearest[i]->addr >> 32), HEX);
      Serial.print(" 층:");
      Serial.print(nearest[i]->floor+1);
      Serial.print(" D:");
      Serial.print(nearest[i]->distance);
      Serial.println("m");
    } else {
      Serial.print("선택된 앵커 ");
      Serial.print(i+1);
      Serial.println(": 없음");
    }
  }
}

/*
 * @brief 삼각측량을 사용하여 위치를 계산합니다.
 */
bool calculatePosition(AnchorData* a1, AnchorData* a2, AnchorData* a3, Point& result_pos) {
  // 가독성을 위해 데이터 추출 (2D 좌표만 사용)
  double x1 = a1->pos.x, y1 = a1->pos.y, r1 = a1->distance;
  double x2 = a2->pos.x, y2 = a2->pos.y, r2 = a2->distance;
  double x3 = a3->pos.x, y3 = a3->pos.y, r3 = a3->distance;
  
  // 거리가 비정상적으로 크거나 작은 경우 처리
  const double MAX_VALID_DISTANCE = 30.0; // 최대 30m 가정
  if (r1 > MAX_VALID_DISTANCE || r2 > MAX_VALID_DISTANCE || r3 > MAX_VALID_DISTANCE) {
    Serial.println("삼각측량 오류: 비정상적으로 큰 거리 측정값");
    return false;
  }
  
  // 동일 지점 앵커 확인
  if ((abs(x1 - x2) < 1e-6 && abs(y1 - y2) < 1e-6) ||
      (abs(x1 - x3) < 1e-6 && abs(y1 - y3) < 1e-6) ||
      (abs(x2 - x3) < 1e-6 && abs(y2 - y3) < 1e-6)) {
    Serial.println("삼각측량 오류: 앵커가 동일 지점에 있습니다.");
    return false;
  }
  
  // 앵커가 동일 선상에 있는지 확인
  double slope1 = (x2 == x1) ? 1e9 : (y2 - y1) / (x2 - x1); // A1->A2 직선의 기울기
  double slope2 = (x3 == x1) ? 1e9 : (y3 - y1) / (x3 - x1); // A1->A3 직선의 기울기
  
  if (abs(slope1 - slope2) < 1e-6) {
    Serial.println("삼각측량 오류: 앵커 3개가 동일 선상에 있습니다.");
    return false;
  }
  
  // 삼변측량 (trilateration) 계산
  // 원의 방정식을 연립하여 교점 계산
  
  // 원의 방정식: (x-xi)^2 + (y-yi)^2 = ri^2, i = 1,2,3
  // 연립 방정식을 선형 방정식으로 변환
  
  // A1과 A2의 방정식을 빼서 선형화
  double A = 2 * (x2 - x1);
  double B = 2 * (y2 - y1);
  double C = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
  
  // A1과 A3의 방정식을 빼서 선형화
  double D = 2 * (x3 - x1);
  double E = 2 * (y3 - y1);
  double F = r1*r1 - r3*r3 - x1*x1 + x3*x3 - y1*y1 + y3*y3;
  
  // 두 선형 방정식의 교점 계산
  double det = A*E - B*D;
  if (abs(det) < 1e-6) {
    Serial.println("삼각측량 오류: 수학적 계산 실패 (특이행렬)");
    return false;
  }
  
  // 태그의 X, Y 좌표 계산
  result_pos.x = (C*E - B*F) / det;
  result_pos.y = (A*F - C*D) / det;
  
  // Z 좌표는 현재 층 높이로 설정 (각 층은 3m 높이 가정)
  const float FLOOR_HEIGHT = 3.0; // 층 높이 (미터)
  result_pos.z = current_floor * FLOOR_HEIGHT + 1.5; // 바닥 + 1.5m 높이에 태그 위치
  
  // 계산 결과 검증
  // 계산된 위치가 앵커들로부터 측정된 거리와 일치하는지 대략 확인
  double calc_dist1 = sqrt(pow(result_pos.x - x1, 2) + pow(result_pos.y - y1, 2));
  double calc_dist2 = sqrt(pow(result_pos.x - x2, 2) + pow(result_pos.y - y2, 2));
  double calc_dist3 = sqrt(pow(result_pos.x - x3, 2) + pow(result_pos.y - y3, 2));
  
  // 계산된 거리와 측정 거리 사이의 오차가 너무 큰 경우 경고
  const double MAX_ERROR = 2.0; // 최대 허용 오차 2m
  if (abs(calc_dist1 - r1) > MAX_ERROR ||
      abs(calc_dist2 - r2) > MAX_ERROR ||
      abs(calc_dist3 - r3) > MAX_ERROR) {
    char err_str[80];
    snprintf(err_str, sizeof(err_str), "삼각측량 경고: 오차 큼 (%.1f, %.1f, %.1f)m",
             abs(calc_dist1 - r1), abs(calc_dist2 - r2), abs(calc_dist3 - r3));
    Serial.println(err_str);
    // 경고만 출력하고 계산은 계속 진행
  }
  
  return true; // 계산 성공
}

/**
 * @brief 현재 층을 결정하는 알고리즘
 */
void determineFloor() {
  // 층별 가중치 계산
  double floor_weights[3] = {0, 0, 0}; // 최대 3개 층 지원
  int floor_anchor_counts[3] = {0, 0, 0}; // 층별 유효 앵커 수 저장

  // 각 층마다 앵커까지의 거리의 역수를 가중치로 사용
  // 더 가까울수록 더 높은 가중치를 가짐
  for (int i = 0; i < NUM_ANCHORS; i++) {
    if (anchors[i].distance_valid && anchors[i].distance > 0) {
      // 앵커 층 인덱스 (0부터 시작)
      uint8_t anchor_floor = anchors[i].floor;
      
      // 거리에 대한 가중치 계산 (거리의 역수)
      double weight = 1.0 / anchors[i].distance;
      
      // 해당 층 가중치에 더함
      floor_weights[anchor_floor] += weight;
      
      // 해당 층 유효 앵커 수 증가
      floor_anchor_counts[anchor_floor]++;
    }
  }

  // 층별 평균 가중치 계산 (앵커 수로 나눔)
  for (int f = 0; f < 3; f++) {
    if (floor_anchor_counts[f] > 0) {
      floor_weights[f] /= floor_anchor_counts[f];
    }
  }

  // 가장 높은 가중치를 가진 층 찾기
  uint8_t highest_weight_floor = 0;
  double highest_weight = floor_weights[0];
  
  for (int f = 1; f < 3; f++) {
    if (floor_weights[f] > highest_weight) {
      highest_weight = floor_weights[f];
      highest_weight_floor = f;
    }
  }

  // 층 변경 히스테리시스 로직
  if (highest_weight_floor != current_floor) {
    // 층 변경 카운터 증가
    floor_change_count++;
    
    // 일정 횟수 이상 동일 층으로 계산된 경우 층 변경
    if (floor_change_count >= floor_vote_threshold) {
      // 층 변경
      Serial.print("층 변경: ");
      Serial.print(current_floor+1);
      Serial.print("층 -> ");
      Serial.print(highest_weight_floor+1);
      Serial.println("층");
      
      current_floor = highest_weight_floor;
      floor_change_count = 0; // 카운터 리셋
    }
  } else {
    // 동일한 층으로 계속 측정되면 카운터 리셋
    floor_change_count = 0;
  }
  
  // 층별 가중치 디버그 출력
  Serial.print("층 가중치: 1층=");
  Serial.print(floor_weights[0]);
  Serial.print(", 2층=");
  Serial.print(floor_weights[1]);
  Serial.print(", 3층=");
  Serial.println(floor_weights[2]);
}

/**
 * @brief 현재 계산된 위치 정보를 시리얼로 출력하는 함수
 */
void serialPrintPosition() {
  // JSON 형태의 위치 정보 출력
  char json_str[100];
  snprintf(json_str, sizeof(json_str), 
        "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"floor\":%d,\"valid\":%s}", 
        current_position.x, current_position.y, current_position.z, 
        current_floor + 1, position_valid ? "true" : "false");
  
  // 웹페이지나 앱에서 파싱하기 쉽게 태그를 붙임
  Serial.print("<POS>");
  Serial.print(json_str);
  Serial.println("</POS>");
  
  // 가독성을 위한 일반 텍스트 정보 (디버깅용)
  Serial.print("위치: X=");
  Serial.print(current_position.x);
  Serial.print("m, Y=");
  Serial.print(current_position.y);
  Serial.print("m, Z=");
  Serial.print(current_position.z);
  Serial.print("m, 층=");
  Serial.println(current_floor + 1);
}

/**
 * @brief BLE를 통해 위치 정보를 전송하는 함수
 */
void sendPositionViaBLE() {
  if (deviceConnected) {
    // 위치 유효성에 따라 데이터 형식 다르게 처리
    if (position_valid) {
      // JSON 형태의 위치 정보 생성 (유효한 위치)
      snprintf(txString, sizeof(txString), 
            "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"floor\":%d,\"anchors\":%d,\"valid\":true}", 
            current_position.x, current_position.y, current_position.z, 
            current_floor + 1, valid_ranges_count);
    } else {
      // 위치 계산 실패 시 간단한 형식
      snprintf(txString, sizeof(txString), 
            "{\"valid\":false,\"anchors\":%d}", valid_ranges_count);
    }
    
    // BLE 특성에 값 설정 및 알림
    pCharacteristic->setValue(txString);
    pCharacteristic->notify();
    debug_printf("BLE로 전송: %s", txString);
  } else {
    debug_print("BLE 연결 없음, 위치 정보 전송 건너뜀");
  }
}