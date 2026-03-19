/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "ring_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { TYPE_NONE=0, TYPE_CHO, TYPE_JUNG } InputType;

typedef struct { //입력된 자음 모음을 저장할 구조체
    InputType type; //타입이 초성인지 중성인지 종성인지
    int index; //실제 초성, 중성, 종성의 번호
} InputData;

typedef struct { //imu에서 데이터를 읽어서 저장할 구조체
    float roll, pitch, yaw; //자세(각도)
    float acc_x, acc_y, acc_z;  //가속도
    float gyro_x, gyro_y, gyro_z; //회전 각속도
    float mag_x, mag_y, mag_z; //자기장
} ImuData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STABLE_TIME 400
#define THRESHOLD 2000
#define MAX_INPUT 50

#define WT901C_RX_BUFFER_SIZE 1
#define WT_PACKET_SIZE        11
#define ACCEL_DATA_ID         0x51
#define GYRO_DATA_ID          0x52
#define ANGLE_DATA_ID         0x53
#define MAG_DATA_ID           0x54

// 서보모터 범위 (출력)
#define SERVO_MIN   500  // 0도, 폈을 때
#define SERVO_MAX   2500  // 180도, 구부렸을 때

// 센서 범위 (입력)
// (본인 센서 값: 2300=폄, 1600=굽힘)
#define ADC_FLAT    2300  // 0도일 때 센서 값
#define ADC_BENT    1600  // 180도일 때 센서 값
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*** flex 변수 ***/
volatile uint16_t adc_values[5];
uint8_t rx_data_flex;
uint8_t send_flag = 0;

const int CHO_MAP[16] = {-1,0,2,3,5,6,7,9,11,12,14,15,16,17,18,-1};
const int JUNG_MAP[16] = {-1,0,2,4,6,8,12,13,17,18,20,1,5,-1,-1,-1};

InputData input_buffer[MAX_INPUT];
int input_idx = 0;

uint8_t sentence_buffer[100];
int sentence_idx = 0;

/*** IMU 변수 ***/
uint8_t wt901c_rx_buffer[1]; //imu에서 들어온 uart 8비트 임시 저장할 버퍼
RingBuffer imu_ring_buffer; //연속적으로 들어올 데이터 저장할 버퍼
ImuData_t imu_data = {0}; //최종 파싱된 값 저장
volatile uint8_t data_updated_flag = 0; //4종류 데이터 다 들어왔는지 체크할 플레그

volatile uint8_t send_count = 0;
volatile float assemble_roll = 0.0f;
volatile float curr;
volatile float base;
volatile float d;

/***자동완성 변수***/
typedef struct {
    const uint8_t *key;
    int key_len;
    const uint8_t *value;
    int value_len;
} MappingEntry;

//매핑 키
uint8_t key_annyeong[] = { 0xEC,0x95,0x88, 0xEB,0x85,0x95 }; // 안녕
uint8_t key_i[]    = {0xEC,0x9D,0xB4}; // 이
uint8_t key_an[]   = {0xEC,0x95,0x88}; // 안
uint8_t key_ip[]   = {0xEC,0x9E,0x85}; // 입
uint8_t key_iseung[] = {0xEC,0x9D,0xB4, 0xEC,0x8A,0xB9}; // 이승
uint8_t key_gamsa[]  = {0xEA,0xB0,0x90, 0xEC,0x82,0xAC}; // 감사

// 매핑 결과 값들
uint8_t val_annyeonghaseyo[] = { 0xEC,0x95,0x88, 0xEB,0x85,0x95, 0xED,0x95,0x98, 0xEC,0x84,0xB8, 0xEC,0x9A,0x94 }; // 안녕하세요
uint8_t val_i[]      = { 0xEC,0x9D,0xB4, 0xEC,0x9D,0x80, 0xEC,0xA7,0x80 }; // 이은지
uint8_t val_an[]     = { 0xEC,0x95,0x88, 0xED,0x95,0x9C, 0xEC,0x98,0x81 }; // 안한영
uint8_t val_ip[]     = { 0xEC,0x9E,0x85, 0xEB,0x8B,0x88, 0xEB,0x8B,0xA4 }; // 입니다
uint8_t val_iseung[] = { 0xEC,0x9D,0xB4, 0xEC,0x8A,0xB9, 0xED,0x99,0x98, 0xEA,0xB5,0x90, 0xEC,0x88,0x98, 0xEB,0x8B,0x98 }; // 이승환교수님
uint8_t val_gamsa[]  = { 0xEA,0xB0,0x90, 0xEC,0x82,0xAC, 0xED,0x95,0xA9, 0xEB,0x8B,0x88, 0xEB,0x8B,0xA4 }; // 감사합니다

MappingEntry table[] = {
    { key_annyeong, sizeof(key_annyeong), val_annyeonghaseyo, sizeof(val_annyeonghaseyo) },
    { key_i,        sizeof(key_i),        val_i,        sizeof(val_i)        },
    { key_an,       sizeof(key_an),       val_an,       sizeof(val_an)       },
    { key_ip,       sizeof(key_ip),       val_ip,       sizeof(val_ip)       },
    { key_iseung,   sizeof(key_iseung),   val_iseung,   sizeof(val_iseung)   },
    { key_gamsa,    sizeof(key_gamsa),    val_gamsa,    sizeof(val_gamsa)    }
};
int mapping_count = sizeof(table) / sizeof(table[0]);

//블루투스 전송을 위한 매핑
// 단독 초성 UTF-8 (최종 충돌 회피용 값으로 대체)
const uint8_t CHO_UTF8[19][3] = {
    {0xE3,0x84,0xB1},  // 0: ㄱ (U+3131)
    {0,0,0},           // 1: ㄲ (미사용)
    {0xE3,0x84,0xB4},  // 2: ㄴ (U+3134)
    {0xE3,0x84,0xB7},  // 3: ㄷ (U+3137)
    {0,0,0},           // 4: ㄸ (미사용)
    {0xE3,0x84,0xB9},  // 5: ㄹ (U+3139)
   {0xE1,0x84,0x86},  // 5: ㅁ (대체)
   {0xE1,0x84,0x87},  // 6: ㅂ (대체)
    {0,0,0},           // 8: ㅃ (미사용)
   {0xE1,0x84,0x89},  // 7: ㅅ (대체)
    {0,0,0},           // 10: ㅆ (미사용)
    {0xE3,0x85,0x87},  // 11: ㅇ (U+3147)
    {0xE3,0x85,0x88},  // 12: ㅈ (U+3148)
    {0,0,0},           // 13: ㅉ (미사용)
    {0xE3,0x85,0x8A},  // 14: ㅊ (U+314A)
    {0xE3,0x85,0x8B},  // 15: ㅋ (U+314B)
    {0xE3,0x85,0x8C},  // 16: ㅌ (U+314C)
    {0xE3,0x85,0x8D},  // 17: ㅍ (U+314D)
    {0xE3,0x85,0x8E}   // 18: ㅎ (U+314E)
};

//단독 중성 UTF-8
const uint8_t JUNG_UTF8[21][3] = {
    {0xE3,0x85,0x8F}, {0xE3,0x85,0x90}, {0xE3,0x85,0x91}, {0xE3,0x85,0x92}, {0xE3,0x85,0x93},  // 0:ㅏ 1:ㅐ 2:ㅑ 3:ㅒ 4:ㅓ
    {0xE3,0x85,0x94}, {0xE3,0x85,0x95}, {0xE3,0x85,0x96}, {0xE3,0x85,0x97}, {0xE3,0x85,0x98},  // 5:ㅔ 6:ㅕ 7:ㅖ 8:ㅗ 9:ㅘ
    {0xE3,0x85,0x99}, {0xE3,0x85,0x9A}, {0xE3,0x85,0x9B}, {0xE3,0x85,0x9C}, {0xE3,0x85,0x9D},  // 10:ㅙ 11:ㅚ 12:ㅛ 13:ㅜ 14:ㅝ
    {0xE3,0x85,0x9E}, {0xE3,0x85,0x9F}, {0xE3,0x85,0xA0}, {0xE3,0x85,0xA1}, {0xE3,0x85,0xA2},  // 15:ㅞ 16:ㅟ 17:ㅠ 18:ㅡ 19:ㅢ
    {0xE3,0x85,0xA3}                                                                           // 20:ㅣ
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/*** FLEX ***/
int get_finger_pattern(void);
void process_hand_gesture(void);
void assemble_and_send(void);
int cho_to_jong(int cho_idx);
void make_hangul_utf8(int cho, int jung, int jong, uint8_t* buf);

/*** IMU ***/
static void parse_imu_packets(void);

/*** 로봇손 동작제어 ***/
void update_robot_hand_linear(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
long constrain(long x, long a, long b);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //FLEX 센서
  //5개 ADC를 DMA로 계속 읽음
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 5);
  HAL_UART_Receive_IT(&huart2, &rx_data_flex, 1); //PC 통신
  // IMU 센서
  //UART1으로 센서 값 받음
  RingBuffer_Init(&imu_ring_buffer); //링버퍼 초기화
  HAL_UART_Receive_IT(&huart1, wt901c_rx_buffer, 1); //UART 수신 대기 상태

  //IMU Write protocol
  // 1. Unlock
  uint8_t cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  HAL_UART_Transmit(&huart1, cmd_unlock, 5, 100);
  HAL_Delay(100);

  // 2. 원하는 설정 변경 (예: 100Hz)
  uint8_t cmd_set_rate[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
  HAL_UART_Transmit(&huart1, cmd_set_rate, 5, 100);
  HAL_Delay(100);

  uint8_t cmd_baud_rate[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
  HAL_UART_Transmit(&huart1, cmd_baud_rate, 5, 100);
  HAL_Delay(100);

  // 3. 변경된 설정 저장
  uint8_t cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
  HAL_UART_Transmit(&huart1, cmd_save, 5, 100);
  HAL_Delay(100);

  // TIM1 (PA8, PA9, PA10, PA11)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 엄저
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // 검지
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // 중지
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // 약지

  // TIM2 (PA0)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 새끼
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   // FLEX 센서
   process_hand_gesture(); // 손가락 패턴 읽고 2초 유지되면 자음이나 모음 하나를 input_buffer에 추가하는 동작을 함.

   if (send_flag == 1) //UART2에 1이 들어오면
   {
      send_flag = 0;
      assemble_and_send(); //지금까지 등록된 자음과 모음을 한글로 조합함.
      input_idx = 0;
      memset(input_buffer, 0, sizeof(input_buffer));

      //글자 조합할 때 roll 값 저장
      assemble_roll=imu_data.roll;
      send_count =0;
   }

   else if (send_flag == 2)
   {
       HAL_UART_Transmit(&huart2, (uint8_t*)"2\n", 2, 100);

       send_flag = 0;
       send_count =0;


       int eff_len = sentence_idx;

       //공백, 띄어쓰기 제거
       while (eff_len > 0 &&
             (sentence_buffer[eff_len - 1] == '\r' ||
              sentence_buffer[eff_len - 1] == '\n' ||
              sentence_buffer[eff_len - 1] == ' '))
       {
           eff_len--;
       }

       int replaced = 0;
       for (int i = 0; i < mapping_count; i++)
       {
          // 단축키일 때 전송
           if (eff_len == table[i].key_len &&
               memcmp(sentence_buffer, table[i].key, table[i].key_len) == 0) //키일 때
           {
               HAL_UART_Transmit(&huart2, table[i].value, table[i].value_len, 100);
               HAL_UART_Transmit(&huart3, table[i].value, table[i].value_len, 100);

               HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
//               HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 100);
               replaced = 1;
               break;
           }
       }

       //기본 전송
       if (!replaced)
       {
           HAL_UART_Transmit(&huart2, sentence_buffer, eff_len, 100);
           HAL_UART_Transmit(&huart3, sentence_buffer, eff_len, 100);

           HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
//           HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 100);

           HAL_Delay(100);
       }

       sentence_idx = 0;
       memset(sentence_buffer, 0, sizeof(sentence_buffer));
       send_count=0;
   }

   // IMU 센서
   parse_imu_packets(); //파싱 함수 호출. uart로 들어온 연속적인 데이터를 패킷 단위로 파싱.

   HAL_Delay(5);

   // IMU roll 기능
   curr= imu_data.roll;
   base = assemble_roll;

   d=curr-base;

   if(fabs(d)>=180.0f){
      send_count=1;
   }

   if(send_count) {
      send_flag=2;
   }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{



    // IMU UART1
    if (huart->Instance == USART1)
    {
        RingBuffer_Write(&imu_ring_buffer, wt901c_rx_buffer[0]);
        //RingBuffer_Write() 함수 호출. imu가 보낸 신호 wt901c_rx_buffer[0] 값을 &imu_ring_buffer에 저장.
        HAL_UART_Receive_IT(&huart1, wt901c_rx_buffer, 1);
    }
}

//IMU 파싱코드

void parse_imu_packets(void)
{
    uint8_t header[2];
    uint8_t data_bytes[8];
    uint8_t sum;
    uint8_t check_sum = 0;

    // 패킷 헤더(0x55)가 나타날 때까지 링버퍼를 읽음
    while (RingBuffer_GetCount(&imu_ring_buffer) >= WT_PACKET_SIZE)
    {
       //RingBuffer_GetCount() 함수: 콜백함수에서 입력 받은 &imu_ring_buffer가 WT_PACKET_SIZE, 11바이트일 경우 실행

       //RingBuffer_Peek() 함수: 링버퍼 안에서 꺼내지 않고 특정 위치의 값을 미리 보는 함수
        header[0] = RingBuffer_Peek(&imu_ring_buffer, 0); //첫번째 바이트 확인해서 header[0]에 넣기
        header[1] = RingBuffer_Peek(&imu_ring_buffer, 1); //두번째 바이트 확인해서 header[1]에 넣기

        if (header[0] != 0x55) //첫 바이트, 헤더가 0x55가 아닐 경우 검사
        {
            RingBuffer_Read(&imu_ring_buffer); // 잘못된 헤더면 한 바이트 버림
            continue;
        }

        // 헤더 두 바이트 버림
        RingBuffer_Read(&imu_ring_buffer);
        RingBuffer_Read(&imu_ring_buffer);

        //체크 썸 시작
        check_sum = header[0] + header[1];

        // 8바이트 데이터와 1바이트 체크섬 읽기
        for (int i = 0; i < 8; i++)
        {
            if (RingBuffer_IsEmpty(&imu_ring_buffer)) return;
            data_bytes[i] = RingBuffer_Read(&imu_ring_buffer);
            //&imu_ring_buffer 값 읽어서 전부 data_bytes[] 배열에 넣음
            check_sum += data_bytes[i];
            //체크썸 계산에 포함
        }

        if (RingBuffer_IsEmpty(&imu_ring_buffer)) return;
        sum = RingBuffer_Read(&imu_ring_buffer);

        if (check_sum != sum) //계산한 체크썸이랑 sum이랑 다르면 잘못된 패킷. 버림.
           continue;

        //DATAL, DATAH 결합해서 16bit raw Data 만들기
        int16_t x_raw = (data_bytes[1] << 8) | data_bytes[0];
        int16_t y_raw = (data_bytes[3] << 8) | data_bytes[2];
        int16_t z_raw = (data_bytes[5] << 8) | data_bytes[4];

        //TYPE 값 따라 데이터 계산하도록
        switch (header[1])
        {
            case ANGLE_DATA_ID: //0x53
                imu_data.roll  = x_raw / 32768.0f * 180.0f;
                imu_data.pitch = y_raw / 32768.0f * 180.0f;
                imu_data.yaw   = z_raw / 32768.0f * 180.0f;
                data_updated_flag |= 0x01;
                break;

            case ACCEL_DATA_ID: //0x51
                imu_data.acc_x = x_raw / 32768.0f * 16.0f;
                imu_data.acc_y = y_raw / 32768.0f * 16.0f;
                imu_data.acc_z = z_raw / 32768.0f * 16.0f;
                data_updated_flag |= 0x02;
                break;

            case GYRO_DATA_ID: //0x52
                imu_data.gyro_x = x_raw / 32768.0f * 2000.0f;
                imu_data.gyro_y = y_raw / 32768.0f * 2000.0f;
                imu_data.gyro_z = z_raw / 32768.0f * 2000.0f;
                data_updated_flag |= 0x04;
                break;

            case MAG_DATA_ID: //0x54
                imu_data.mag_x = x_raw / 32768.0f * 4912.0f;
                imu_data.mag_y = y_raw / 32768.0f * 4912.0f;
                imu_data.mag_z = z_raw / 32768.0f * 4912.0f;
                data_updated_flag |= 0x08;
                break;
        }
    }
}

/*******************************************************************************
   FLEX 기능
*******************************************************************************/
int get_finger_pattern(void)
{
   //각 adc마다 문턱전압 1.6V 이하로 내려가면 = 굽힘 = 패턴 값 바꿈
    int pattern = 0;
    if (adc_values[1] < THRESHOLD) pattern |= 1; //0001
    if (adc_values[2] < THRESHOLD) pattern |= 2; //0010
    if (adc_values[3] < THRESHOLD * 0.9) pattern |= 4; //0100
    if (adc_values[4] < THRESHOLD) pattern |= 8; //1000
    return pattern;
    // 1번(검지) 2번(중지) 굽히면 0001+0010=0011, 즉 패턴 3번.
}

void process_hand_gesture(void)
{
   update_robot_hand_linear();

    if (adc_values[0] < THRESHOLD &&  // 엄지
        adc_values[1] < THRESHOLD &&  // 검지
        adc_values[2] < THRESHOLD &&  // 중지
        adc_values[3] < THRESHOLD &&  // 약지
        adc_values[4] < THRESHOLD)    // 새끼
    {
        send_flag = 1;
        return;   // 자음·모음 인식 코드 실행 안 함
    }

    static int last_pattern = 0;
    static int last_mode = -1;
    static uint32_t stable_start_tick = 0;
    static int is_registered = 0;

    //get_finger_pattern 함수에서 패턴 값 받아옴.
    int curr_pattern = get_finger_pattern();
    //0번(엄지) 값으로 mode 결정
    int curr_mode = (adc_values[0] < THRESHOLD);

    //전부 펼친 상태. 리셋.
    if (curr_pattern == 0)
    {
        last_pattern = 0;
        last_mode = -1;
        stable_start_tick = 0;
        is_registered = 0;
        return;
    }

    if (curr_pattern == last_pattern && curr_mode == last_mode)
    {
       //현재 패턴이 지난 패턴과 동일한 지 확인.
        if (is_registered == 0)
        {
            if (HAL_GetTick() - stable_start_tick >= STABLE_TIME)
               //그 상태가 2초동안 유지되면 입력으로 판단함.
            {
                if (input_idx < MAX_INPUT)
                {
                    int mapped_idx = -1;
                    InputType type = TYPE_NONE;

                    //adc_values[0]으로 자음 모음 타입 구분
                    if (curr_mode == 0)
                    {
                        mapped_idx = CHO_MAP[curr_pattern];
                        type = TYPE_CHO;
                    }
                    else
                    {
                        mapped_idx = JUNG_MAP[curr_pattern];
                        type = TYPE_JUNG;
                    }

                    if (mapped_idx != -1)
                    {
                        input_buffer[input_idx].type = type;
                        input_buffer[input_idx].index = mapped_idx;
                        input_idx++;

                        ///////////바로바로 자음 모음 전송///////////
                        if (type == TYPE_CHO)
                        {
                            HAL_UART_Transmit(&huart2, CHO_UTF8[mapped_idx], 3, 100);
                        }
                        else if (type == TYPE_JUNG)
                        {
                            HAL_UART_Transmit(&huart2, JUNG_UTF8[mapped_idx], 3, 100);
                        }
                    }
                }
                is_registered = 1;
            }
        }
    }

    //손 모양이 바꼈을 경우, 새로운 입력 시작
    else
    {
        last_pattern = curr_pattern;
        last_mode = curr_mode;
        stable_start_tick = HAL_GetTick();
        is_registered = 0;
    }
}

void assemble_and_send(void)
{
    int i = 0;
    uint8_t utf8_char[5];

    while (i < input_idx)
    {
       //input_buffer안에 자음 모음이 쌓여 있음.
        if (input_buffer[i].type == TYPE_CHO &&
            (i+1 < input_idx) &&
            input_buffer[i+1].type == TYPE_JUNG)
           //초성+종성 조합이면
        {
            int cho = input_buffer[i].index; //초성 인덱스를 cho에 저장
            int jung = input_buffer[i+1].index; //중성 인덱스를 jung에 저장
            int jong = 0; //초성+중성 조합이니까 받침은 없음.
            int consumed = 2;

            //뒤에 받침이 있는지 검사
            if ((i+2 < input_idx) && input_buffer[i+2].type == TYPE_CHO)
            {
               //cho_to_jong으로 세번째 자음이 종성이 가능한지 확인해서 t에 저장.
                int t = cho_to_jong(input_buffer[i+2].index);
                if (t != 0)
                {
                   //가능하면 t값이 jong이 됨.
                    jong = t;
                    consumed = 3;
                }
            }

            //UTF-8로 변환.
            make_hangul_utf8(cho, jung, jong, utf8_char);
            // UART2에 글자 출력
            HAL_UART_Transmit(&huart2, utf8_char, 3, 100);

            if (sentence_idx + 3 < 100)
            {
              //UTF-8 변환한 결과를 sentence_buffer에 저장함.
              memcpy(&sentence_buffer[sentence_idx], utf8_char, 3);
              sentence_idx += 3;
            }

            i += consumed;
        }
        else
        {
            i++;
        }
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
}

int cho_to_jong(int cho_idx)
{
   //초성을 종성으로 변환하는 함수. 초성 중성 배열만 있고 종성은 없었으니까.
    switch(cho_idx) {
        case 0: return 1;
        case 2: return 4;
        case 3: return 7;
        case 5: return 8;
        case 6: return 16;
        case 7: return 17;
        case 9: return 19;
        case 11: return 21;
        case 12: return 22;
        case 14: return 23;
        case 15: return 24;
        case 16: return 25;
        case 17: return 26;
        case 18: return 27;
        default: return 0;
    }
}

void make_hangul_utf8(int cho, int jung, int jong, uint8_t* buf)
{
   //유니코드 만드는 공식
    uint16_t unicode = 0xAC00 + (cho * 588) + (jung * 28) + jong;

    //UTF-8로 변환
    buf[0] = 0xE0 | ((unicode >> 12) & 0x0F);
    buf[1] = 0x80 | ((unicode >> 6) & 0x3F);
    buf[2] = 0x80 | (unicode & 0x3F);
    buf[3] = '\0';
}

/*******************************************************************************
  서보 모터
*******************************************************************************/

//adc 값을 pwm duty 값으로 변환
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // adc_values[] 값이 ADC_FLAT에서부터 얼마나 떨어져 있는지 계산하고 입력 값이 전체 범위에서 몇 % 지점인지 계산할 것.
}

//범위 제한
long constrain(long x, long a, long b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

// 로봇 손 제어
void update_robot_hand_linear(void)
{
    for (int i = 0; i < 5; i++)
    {
        long adc_val = adc_values[i];

        // 맵핑: 2300(폄) -> 1000(0도), 1600(굽힘) -> 2000(180도)
        // map 함수로 입력값(2300~1600)을 출력값(1000~2000)으로 변환해서 pulse 변수에 저장.
        // ADC_FLAT은 0도 일 때 센서 값, ADC_BENT는 180도 일 때 센서 값.
        long pulse = map(adc_val, ADC_FLAT, ADC_BENT, SERVO_MIN, SERVO_MAX);

        // 펄스 값이 서보모터 허용 범위(1000~2000)를 벗어나지 않도록 제한하고 다시 pulse 변수에 저장.
        pulse = constrain(pulse, SERVO_MIN, SERVO_MAX);

        // map 함수로 계산한 PWM 펄스값을 타이머에 넣음
        switch (i)
        {
            case 0: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse); break; // 엄지 (PA8)
            case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse); break; // 검지 (PA9)
            case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse); break; // 중지 (PA10)
            case 3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse); break; // 약지 (PA11)
            case 4: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); break; // 새끼 (PB6)
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
