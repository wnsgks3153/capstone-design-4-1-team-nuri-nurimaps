// 아래 코드는 https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000/tree/main의
// ex_06e_AES_ss_twr_initiator 예제를 한국어로 번역한 것입니다.
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "dw3000.h"
#include "dw3000_mac_802_15_4.h"

#define APP_NAME "SS TWR AES INIT v1.0"

// 연결 핀
const uint8_t PIN_RST = 27;  // 리셋 핀
const uint8_t PIN_IRQ = 34;  // 인터럽트 핀
const uint8_t PIN_SS = 4;    // SPI 선택 핀

/* 802.15.4 프레임 샘플 */
#if 0
mac_frame_802_15_4_format_t mac_frame =
{
    /*
    * 프레임 제어[0] = 0x09 = 데이터 프레임, 보안 활성화, PEND 비설정, ACK 불필요, PAN ID 압축 비활성화(소스에 PAN ID 없음)
    * 프레임 제어[1] = 0xEC = 시퀀스 번호 포함, IEs 없음, 확장 주소 사용, 프레임 버전 2(IEEE Std 802.15.4)
    */
    .mhr_802_15_4.frame_ctrl[0] = 0x09,
    .mhr_802_15_4.frame_ctrl[1] = 0xEC,

    /* 시퀀스 번호 초기값 */
    .mhr_802_15_4.sequence_num = 0x00,

    .mhr_802_15_4.dest_pan_id[0] = 0x21,
    .mhr_802_15_4.dest_pan_id[1] = 0x43,

    /* 보조 보안 헤더의 보안 제어 필드 설정
    *  보안 제어 = 0xF:
    *      보안 수준: 0x7 = MIC 16(데이터 기밀성 OFF, 데이터 무결성 확인),
    *      키 식별자 모드: 0x1 = 키 인덱스 필드에서 키 결정,
    *      프레임 카운터 억제: 0x0 = 프레임 카운터 포함 및 이를 사용하여 nonce 생성.
    *      ASN을 Nonce로 사용: 0x0 = 프레임 카운터를 사용하여 nonce 생성
    *        (CCM* nonce = 소스 주소(8바이트) + 프레임 카운터(4바이트) + 보안 수준(1바이트) - 위에서 0x7로 설정)
    *  즉, AUX 헤더의 형식은 보안 제어(1바이트) + 프레임 카운터(4바이트) + 키 식별자(1바이트) = 총 6바이트
    */
    .mhr_802_15_4.aux_security.security_ctrl = 0x0F,
};
#endif

mac_frame_802_15_4_format_t mac_frame = {
  { { 0x09, 0xEC },
    0x00,
    { 0x21, 0x43 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x0F, { 0x00, 0x00, 0x00, 0x00 }, 0x00 } },
  0x00
};

#if 0
static dwt_aes_config_t aes_config =
{
    .key_load           = AES_KEY_Load,         // AES 엔진에 키를 로드 (아래 참고 사항 15 참조)
    .key_size           = AES_KEY_128bit,       // 128비트 키 사용
    .key_src            = AES_KEY_Src_Register, // 키 소스는 IC 레지스터
    .aes_core_type      = AES_core_type_CCM,    // CCM 코어 사용
    .aes_key_otp_type   = AES_key_RAM,
    .key_addr           = 0
};
#endif

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

/* Initiator(발신자) 데이터 */
#define DEST_PAN_ID 0x4321          /* 이 예제에서 사용되는 PAN ID */
#define SRC_ADDR 0x8877665544332211 /* Initiator(발신자)의 주소 */
static uint64_t DEST_ADDR;          /* Responder(수신자)의 주소 */

unsigned long lastScanTime = 0;  // 마지막 스캔 시각

/* 기본 통신 구성. 기본 비-STS DW 모드를 사용함. */
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

/* AUX 보안 헤더의 키 인덱스에 따른 선택적 키 옵션( 기존 예제 코드에서 임의로 설정되어있던 AES 키 데이터)
static dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0xFFEEDDCC, 0xBBAA9988, 0x77665544, 0x33221100, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
}; */

/* 다중 통신 간 테스트 과정에서 AES 키 값을 모두 같도록 설정 */
static dwt_aes_key_t keys_options[NUM_OF_KEY_OPTIONS] = {
  { 0x00010101, 0x00010101, 0x00010101, 0x00010101, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x00010101, 0x00010101, 0x00010101, 0x00010101, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
  { 0x00010101, 0x00010101, 0x00010101, 0x00010101, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }
};

/* 64 MHz PRF에 대한 기본 안테나 지연 값. 아래 참고 사항 2 참조. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* 거리 측정 과정에서 사용되는 프레임의 MAC 페이로드 데이터. 아래 참고 사항 3 참조. */
/* Initiator(발신자)가 Responder(수신자)에게 보내는 Poll 메시지 */
static uint8_t tx_poll_msg[] = { 'P', 'o', 'l', 'l', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e' };
/* Initiator(발신자)에게 보내는 응답 메시지. 처음 8바이트는 Poll RX 시간과 Response TX 시간을 저장하는 데 사용됨. */
static uint8_t rx_resp_msg[] = { 0, 0, 0, 0, 0, 0, 0, 0, 'R', 'e', 's', 'p', 'o', 'n', 's', 'e' };

#define START_RECEIVE_DATA_LOCATION 8  // MAC 페이로드 사용자 데이터가 시작하는 인덱스 (위의 응답 메시지에서 'R'이 위치한 곳)

/* 위에서 정의한 프레임의 특정 필드에 접근하기 위한 인덱스 */
#define ALL_MSG_SN_IDX 2           // MHR(메시지 헤더)에서 시퀀스 번호 바이트의 인덱스
#define RESP_MSG_POLL_RX_TS_IDX 0  // MAC 페이로드에서 Poll RX 시간의 인덱스
#define RESP_MSG_RESP_TX_TS_IDX 4  // MAC 페이로드에서 Response TX 시간의 인덱스
#define RESP_MSG_TS_LEN 4

/* 키 인덱스 0은 전송 시 사용이 금지됨. 따라서 1이 첫 번째 키 인덱스가 됨.
 * 이 예제에서는 Initiator(발신자)의 데이터 암호화를 위해 이 키 테이블 인덱스를 사용함. */
#define INITIATOR_KEY_INDEX 1

/* 수신된 응답 메시지를 저장하는 버퍼.
 * 이 버퍼의 크기는 이 코드에서 처리할 수 있는 가장 긴 프레임을 기준으로 조정됨. */
#define RX_BUF_LEN 127 /* STD PHR 모드를 사용할 경우 수신된 프레임 크기는 127을 초과할 수 없음 */
static uint8_t rx_buffer[RX_BUF_LEN];

/* 프레임 간 딜레이(단위: UWB 마이크로초). 아래 참고 사항 1 참조. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 1720
/* 응답 수신 타임아웃. 아래 참고 사항 5 참조. */
#define RESP_RX_TIMEOUT_UUS 250

/* 계산된 비행 시간(Time of Flight, ToF)과 거리 값을 저장하여 디버그 시점에서 참조할 수 있도록 함. */
static double tof;
static double distance;
static double distanceSUM[3] = { 0 };  // 오차를 줄이기 위한 평균 계산용 소수 배열 변수
double distanceAVG;                    // 오차를 줄이기 위한 평균 계산용 소수 배열 변수

/* PG_DELAY 및 TX_POWER 레지스터의 값은 현재 온도에서 대역폭과 스펙트럼 전력을 반영함.
 * 이러한 값은 기준 측정을 수행하기 전에 보정할 수 있음. 아래 참고 사항 2 참조. */
extern dwt_txconfig_t txconfig_options;

static uint32_t frame_cnt = 0; /* 참고 사항 13 참조 */
static uint8_t seq_cnt = 0x0A; /* 프레임 시퀀스 번호, 전송 후 증가됨. */
uint32_t status_reg;
uint8_t nonce[13]; /* IEEE 802.15.4 규격에 따라 사용되는 13바이트 nonce */
dwt_aes_job_t aes_job_tx, aes_job_rx;
int8_t status;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"         // BLE SERVIECE UUID       - 별도의 서비스 ID를 부여하고 싶은 경우 변경
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // BLE CHARICTERISTIC UUID - 별도의 서비스 ID를 부여하고 싶은 경우 변경

// 연결하려는 원격 서비스
//static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID serviceUUID("a3aa");
static BLEUUID charUUID("d368");
// 원격 서비스에서 관심 있는 특성
//static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

BLECharacteristic* pCharacteristic;
BLEScan* pBLEScan;

char bleMessage[50];
bool deviceConnected = false;  // 장치 연결 여부 플래그

// BLE 서버에 연결/해제 시 호출되는 콜백 클래스
class MyServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("장치와 연결되었습니다. [MyServerCallback.onConnect]");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("장치와 연결이 끊어졌습니다. [MyServerCallback.onDisconnect]");
    delay(500);

    pServer->getAdvertising()->start();
    Serial.println("waiting for connection from client");
  }
};

/* BLE 클라이언트에 연결/해제 시 호출되는 콜백 클래스 하지만, 출력되는 경우가 발생하지 않음
   태그 입장에서 앵커와 BLE로 직접 연결되는 경우가 발생하지 않기 때문 */
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("MyClientCallback.onСonnect");
  }

  void onDisconnect(BLEClient* pclient) {
    Serial.println("MyClientCallback.onDisconnect");
  }
};

// BLE 광고 패킷 수신 시 호출되는 콜백 클래스
class CharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* dhtCharacteristic) {
    String value = pCharacteristic->getValue();
    Serial.print("Received Value: ");
    Serial.println(value.c_str());
  }
};

#define MAX_DEVICES 30
#define MAX_NAME_LEN 30

char filteredDeviceNames[MAX_DEVICES][MAX_NAME_LEN];  // 필터링된 이름 저장용 배열
int anchorIds[MAX_DEVICES];                           // 어떤 앵커와 통신했는지 기록하는 배열
int filteredDeviceCount = 0;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {

    uint8_t* addr = (uint8_t*)advertisedDevice.getAddress().getNative();

    /* 이전에 사용하던 장치들의 MAC 주소는 AC:15:18로 시작됬음, 새로운 장치가 다른 주소를 사용하는 경우 변경할 예정 */
    if (addr[0] == 0xAC && addr[1] == 0x15 && addr[2] == 0x18) {
      Serial.print("UWB Anchor Device found: ");            // 디버그용
      Serial.println(advertisedDevice.toString().c_str());  // 디버그용

      /* 장치 이름이 "0x02"로 시작하고 길이가 정확히 18인지 확인
       * [0-1]  분류: 태그(01), 앵커(02), 마스터앵커(03, 마스터 앵커는 필요 시 구현 예정) 
       * [2-4]  순번: 001 ~ 099로 각 장치에 번호를 부여
       * [5-7]  위치: 복도(001), 계단(002)
       * [8-15] 층수: 3층(00300030), 4층(00400040), 3.5층(00350035)
       * 이상하거나, 맘에안들거나, 개선하고싶다면 언제든지 제안할 것.
       */

      String nameStr = advertisedDevice.getName();
      if (nameStr.startsWith("0x02") && nameStr.length() == 18) {
        if (filteredDeviceCount < MAX_DEVICES) {
          // 이름 저장
          nameStr.toCharArray(filteredDeviceNames[filteredDeviceCount], MAX_NAME_LEN);

          // 앵커 순번 추출: 4~6번째 인덱스 -> substring(4, 7)
          String anchorIdStr = nameStr.substring(5, 7);  // "03" 같은 부분 추출
          int anchorId = anchorIdStr.toInt();            // 정수로 변환하여 저장
          anchorIds[filteredDeviceCount] = anchorId;

          filteredDeviceCount++;
        }
      }

      myDevice = new BLEAdvertisedDevice(advertisedDevice);
    }
  }
};

/* 태그가 앵커와 통신하는 코드
   태그와 앵커가 여러개가 되며 앵커의 주소를 동적으로 가져와 순차적으로 통신할 때 사용하도록
   구조체로 분리하여 전에 사용했던 하드코딩 방식을 개선

   이 함수를 사용할 때 미리 ESP32를 통해 UWB 앵커 주소를 받은 다음 앵커의 주소를 메서드로 넣어 호출만 하면 됨
   하단 loop() 부분을 참고
*/
float rangeDelay;              // 거리 측정 간 딜레이 기간(단위: 밀리초)
int conversionMS = -10000000;  // clockoffset에 conversionMS를 곱하여 딜레이용 시간으로 변환하기위한 변수

void communicateWithAnchors() {
  for (int i = 0; i < 5; i++) {    // 5번을 측정하여 평균을 계산하여 오차를 보정(성능 저하가 생기는 경우 변경 예정)
    for (int j = 0; j < 3; j++) {  // 앵커 리스트에서 차례대로 3개의 주소를 꺼내와 통신
      if (strncmp(filteredDeviceNames[j], "0x", 2) == 0 && strlen(filteredDeviceNames[j]) == 18) {

        uint64_t DEST_ADDR = strtoull(filteredDeviceNames[j], NULL, 16);

        /* 사용할 올바른 키를 설정 */
        dwt_set_keyreg_128(&keys_options[INITIATOR_KEY_INDEX - 1]);
        /* 프레임에 사용할 키 인덱스 설정 */
        MAC_FRAME_AUX_KEY_IDENTIFY_802_15_4(&mac_frame) = INITIATOR_KEY_INDEX;

        /* MHR을 올바른 SRC 및 DEST 주소로 업데이트하고 13바이트 nonce를 생성
         * (같은 MAC 프레임 구조가 수신된 데이터와 전송된 데이터를 모두 저장하는 데 사용되므로,
         * 전송 전에 SRC 및 DEST 주소를 업데이트해야 함) */
        mac_frame_set_pan_ids_and_addresses_802_15_4(&mac_frame, DEST_PAN_ID, DEST_ADDR, SRC_ADDR);
        mac_frame_get_nonce(&mac_frame, nonce);
        aes_job_tx.mic_size = mac_frame_get_aux_mic_size(&mac_frame);
        aes_config.mode = AES_Encrypt;
        aes_config.mic = dwt_mic_size_from_bytes(aes_job_tx.mic_size);
        dwt_configure_aes(&aes_config);

        /* AES 작업이 TX 프레임 데이터를 가져와 DW IC TX 버퍼에 복사한 후 전송함. 아래 참고 사항 7 참조. 
         * 이 부분은 아직 오류가 발생한 적이 없기 때문에 무한 오류가 출력되는 부분을 변경하는 것은 보류
         * 예외가 발생하는 경우 통신 불가능 상태로 변하는 것을 막기 위해 수정할 예정
         */
        status = dwt_do_aes(&aes_job_tx, aes_config.aes_core_type);
        /* 오류 검사 */
        if (status < 0) {
          test_run_info((unsigned char*)"AES length error");
          while (1)
            ; /* 오류 발생 */
        } else if (status & AES_ERRORS) {
          test_run_info((unsigned char*)"ERROR AES");
          while (1)
            ; /* 오류 발생 */
        }

        /* 프레임 컨트롤을 설정하고 전송 시작 */
        dwt_writetxfctrl(aes_job_tx.header_len + aes_job_tx.payload_len + aes_job_tx.mic_size + FCS_LEN, 0, 1); /* TX 버퍼 오프셋 0, ranging 사용. */

        /* 전송 시작, 응답이 예상됨을 나타내므로 프레임 전송 후 설정된 지연(dwt_setrxaftertxdelay()로 설정됨)이 경과하면 자동으로 수신이 활성화됨. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* 전송이 올바르게 수행되었다고 가정하고, 프레임 수신 또는 오류/타임아웃을 폴링. 아래 참고 사항 8 참조. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

        /* Poll 메시지 전송 후 프레임 시퀀스 번호(256 모듈로) 및 프레임 카운터 증가 */
        MAC_FRAME_SEQ_NUM_802_15_4(&mac_frame) = ++seq_cnt;
        mac_frame_update_aux_frame_cnt(&mac_frame, ++frame_cnt);

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) { /* 응답을 수신함 */
          uint32_t frame_len;

          /* DW IC 상태 레지스터에서 정상적인 RX 프레임 이벤트를 클리어 */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

          /* 수신된 데이터 길이를 읽음 */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

          /* 프레임을 수신했으므로, 먼저 MHR을 읽고 기대한 프레임인지 확인해야 함:
           * 대상 주소는 소스 주소와 일치해야 함 (이 확인을 위해 프레임 필터링을 설정할 수도 있으나,
           * 이 예제에서는 다루지 않음); 또한 헤더에 보안이 활성화되어 있어야 함.
           * 이러한 검사 중 하나라도 실패하면 rx_aes_802_15_4는 오류를 반환함. 
           */

          aes_config.mode = AES_Decrypt;
          PAYLOAD_PTR_802_15_4(&mac_frame) = rx_buffer; /* MAC 페이로드 포인터 설정 */

          /* 이 예제에서는 Initiator와 Responder가 암호화된 데이터를 전송한다고 가정함 */
          status = rx_aes_802_15_4(&mac_frame, frame_len, &aes_job_rx, sizeof(rx_buffer), keys_options, DEST_ADDR, SRC_ADDR, &aes_config);

          /* 오류 발생 시 처리 
           * 기존 예제 코드는 에러가 발생하는 경우 무한으로 에러 상태를 출력하도록 설정되어있음
           * 에러를 무한 출력하며, 통신 불가능 상태가 되는 while문 제거 및 본인(이 코드를 사용하는 태그)을 향한 메세지가 아니면 
           * 메세지를 무시하고 다시 메세지를 받는 상태로 변환되도록 return으로 설정 변경
           */
          if (status != AES_RES_OK) {
            switch (status) {
              case AES_RES_ERROR_LENGTH:
                Serial.println("AES 길이 오류");
                break;
              case AES_RES_ERROR:
                Serial.println("AES 오류");
                break;
              case AES_RES_ERROR_FRAME:
                Serial.println("프레임 오류");
                break;
              case AES_RES_ERROR_IGNORE_FRAME:
                Serial.println("해당 프레임은 대상이 아님");
                break;  // 잘못된 목적지 주소를 가진 프레임 무시
            }
            delay(100);
            return;
          }

          /* 프레임이 예상된 응답인지 확인 ("SS TWR AES responder" 예제에서의 응답).
             * 응답 메시지의 처음 8바이트는 Poll과 Response 타임스탬프를 포함하므로 이를 무시 */
          if (memcmp(&rx_buffer[START_RECEIVE_DATA_LOCATION], &rx_resp_msg[START_RECEIVE_DATA_LOCATION], aes_job_rx.payload_len - START_RECEIVE_DATA_LOCATION) == 0) {
            uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32_t rtd_init, rtd_resp;
            float clockOffsetRatio;

            /* Poll 전송 및 응답 수신 타임스탬프를 읽어옴. 아래의 NOTE 9 참조. */
            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            /* 캐리어 적분기 값을 읽어와서 클록 오프셋 비율을 계산. 아래의 NOTE 11 참조. */
            clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

            /* 응답 메시지에서 타임스탬프를 가져옴. */
            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            /* 비행 시간(time of flight)과 거리를 계산, 로컬과 원격 클록 속도의 차이를 보정하기 위해 클록 오프셋 비율을 사용 */
            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;

            distanceSUM[j] += distance;

            rangeDelay = clockOffsetRatio * conversionMS + 50;  // 항상 일정범위의 랜덤한 수로 변하는 ClockOffset을 응용하여 랜덤 딜레이 사용
          }

        } else {
          /* DW IC 상태 레지스터에서 RX 오류/타임아웃 이벤트를 클리어 */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* 거리 측정 교환 간에 딜레이를 실행 */
        Sleep(rangeDelay);
      }
    }
  }
}

void initUWB() {
  /* SPI 속도 설정, DW3000은 최대 38 MHz 지원 */
  /* DW IC 초기화 */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2);  // DW3000이 시작하는 데 필요한 시간 (INIT_RC에서 IDLE_RC로 전환, 또는 SPIRDY 이벤트를 기다릴 수도 있음)

  while (!dwt_checkidlerc())  // 진행하기 전에 DW IC가 IDLE_RC 상태인지 확인해야 함
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

  // 디버그용으로 LED 활성화. TX 발생 시 DW3000 red eval-shield 보드에서 D1 LED가 점멸함.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* DW IC 구성. 아래 참고 사항 14 참조. */
  if (dwt_configure(&config)) /* dwt_configure가 DWT_ERROR를 반환하면 PLL 또는 RX 보정이 실패한 것이므로 호스트는 장치를 리셋해야 함 */
  {
    test_run_info((unsigned char*)"CONFIG FAILED     ");
    while (1) {};
  }

  /* TX 스펙트럼 파라미터(출력, PG 지연 및 PG 카운트) 설정 */
  dwt_configuretxrf(&txconfig_options);

  /* 기본 안테나 지연 값 적용. 아래 참고 사항 2 참조. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* 예상되는 응답 지연 및 타임아웃 설정. 아래 참고 사항 1 및 5 참조.
   * 이 예제는 SS-TWR Responder와 짝을 이루며, 지연/타이밍이 변경될 경우
   * 양쪽에서 동일하게 변경되어야 함. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* 다음으로 GPIO 5 및 6에서 TX/RX 상태 출력을 활성화하여 디버그 가능하도록 설정 */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  /* TX 및 RX AES 작업을 설정. TX 작업은 Poll 메시지를 암호화하는 데 사용됨,
   * RX 작업은 Response 메시지를 복호화하는 데 사용됨. */
  aes_job_tx.mode = AES_Encrypt;                              /* 암호화 작업 */
  aes_job_tx.src_port = AES_Src_Tx_buf;                       /* dwt_do_aes가 TX 버퍼에서 평문을 가져옴 */
  aes_job_tx.dst_port = AES_Dst_Tx_buf;                       /* dwt_do_aes가 원래 평문 TX 버퍼를 암호화된 데이터로 대체 */
  aes_job_tx.nonce = nonce;                                   /* nonce 구조체의 포인터 */
  aes_job_tx.header = (uint8_t*)MHR_802_15_4_PTR(&mac_frame); /* 암호화되지 않는 평문 헤더 */
  aes_job_tx.header_len = MAC_FRAME_HEADER_SIZE(&mac_frame);
  aes_job_tx.payload = tx_poll_msg;             /* 암호화할 페이로드 */
  aes_job_tx.payload_len = sizeof(tx_poll_msg); /* 암호화할 페이로드 크기 */

  aes_job_rx.mode = AES_Decrypt;          /* 복호화 작업 */
  aes_job_rx.src_port = AES_Src_Rx_buf_0; /* 복호화할 데이터의 소스는 IC RX 버퍼 */
  aes_job_rx.dst_port = AES_Dst_Rx_buf_0; /* 복호화된 데이터를 IC RX 버퍼에 저장 (원본 RX 프레임이 삭제됨) */
  aes_job_rx.header_len = aes_job_tx.header_len;
  aes_job_rx.header = aes_job_tx.header; /* 암호화되지 않는 평문 헤더 */
  aes_job_rx.payload = rx_buffer;        /* IC에서 읽어와 복호화된 데이터를 저장할 위치의 포인터 */
}

void initBLE() {
  BLEDevice::init("UWB TAG 01");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallback());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new CharacteristicCallback());
  pService->start();

  // BLE 광고 설정
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  /* BLE 스캔 설정을 비동기 방식(스캔을 실행해두고 다음 줄로 넘어감)으로 설정해두고 5초간 위 스캔 루프를 반복 실행
     즉, 1초 동안 0.3초간 2번 스캔 루프를 실행*/
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(300);     // 300ms마다 스캔 루프 실행
  pBLEScan->setWindow(150);       // 150ms동안 스캔
  pBLEScan->setActiveScan(true);  // 추가 Response를 받기 위해서 ActiveScan을 활성화(= 장치이름, UUID, 등을 추가로 더 받는다는 뜻)
}

void setup() {
  UART_init();
  initUWB();
  initBLE();
}

void loop() {
  unsigned long currentMillis = millis();
  char msg_buffer[10];   // 결과 포맷용 임시 버퍼
  bleMessage[0] = '\0';  // 메세지 초기화

  // 3초(3000ms)마다 스캔 수행
  if (currentMillis - lastScanTime >= 3000) {
    filteredDeviceCount = 0;
    lastScanTime = currentMillis;
    pBLEScan->start(1, false);  // 1초 동안 스캔 (false = 비동기)
  }

  communicateWithAnchors();  // 앵커의 주소를 가져오는 것을 함수 내부에서 동작하도록 변경

  for (int i = 0; i < 3; i++) {
    distanceAVG = distanceSUM[i] / 5.0;
    int convertINT = round(distanceAVG * 100);
    int anchorID = anchorIds[i];

    Serial.print(anchorIds[i]);
    Serial.print(": ");
    Serial.println(distanceAVG);

    sprintf(msg_buffer, "%02d%04d", anchorID, convertINT);
    strcat(bleMessage, msg_buffer);  // 결과 문자열에 추가
    distanceSUM[i] = { 0.0 };
  }

  Serial.print("Formatted txString: ");
  Serial.println(bleMessage);

  delay(5000);  // 디버그용

  //실제 휴대폰으로 데이터 전송용 값 저장 및 notify() 호출을 통해 전달
  pCharacteristic->setValue(bleMessage);
  pCharacteristic->notify();
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
