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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RESPONSE_PARAM_LEN_MAX  128

typedef struct {
  uint16_t len;
  uint8_t  data[QUEUE_MSG_SIZE];
} UartQueueData_t;

typedef struct {
    uint16_t name_len;
    uint16_t param_len;
    uint16_t total_len;
    uint8_t  type;
    uint8_t  idx;
    uint8_t  name[RESPONSE_PARAM_LEN_MAX];
    uint8_t  param[RESPONSE_PARAM_LEN_MAX];
} response_t;

typedef int (*func_ptr_t)(const uint8_t *input, uint16_t in_len, uint8_t *param, uint16_t *param_len);

typedef struct FuncEntry {
  func_ptr_t func;
  struct FuncEntry *next;
  const char *name;
} FuncEntry_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Buffer sizes */
#define USER_PAYLOAD_SIZE_MAX       (512)   /* max size of user frame over UART in KBytes */
#define UART_DMA_BUFFER_SIZE        (2 * USER_PAYLOAD_SIZE_MAX)
#define STREAM_BUFFER_SIZE          (USER_PAYLOAD_SIZE_MAX)
#define STREAM_TRIGGER_LEVEL        1
#define MSGQUEUE_OBJECTS            3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for Main */
osThreadId_t MainHandle;
const osThreadAttr_t Main_attributes = {
  .name = "Main",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartRx */
osThreadId_t UartRxHandle;
const osThreadAttr_t UartRx_attributes = {
  .name = "UartRx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTx */
osThreadId_t UartTxHandle;
const osThreadAttr_t UartTx_attributes = {
  .name = "UartTx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PayloadQueueRx */
osMessageQueueId_t PayloadQueueRxHandle;
const osMessageQueueAttr_t PayloadQueueRx_attributes = {
  .name = "PayloadQueueRx"
};
/* Definitions for PayloadQueueTx */
osMessageQueueId_t PayloadQueueTxHandle;
const osMessageQueueAttr_t PayloadQueueTx_attributes = {
  .name = "PayloadQueueTx"
};
/* USER CODE BEGIN PV */

uint8_t uart1_dma_buffer_rx[UART_DMA_BUFFER_SIZE];
uint8_t uart1_dma_buffer_tx[UART_DMA_BUFFER_SIZE];
uint8_t uart_task_buf_rx[USER_PAYLOAD_SIZE_MAX];
uint8_t uart_task_buf_tx[USER_PAYLOAD_SIZE_MAX];

StreamBufferHandle_t UartRxStream;
static FuncEntry_t *g_funcs_head = NULL;
uint16_t leftover_len = 0;
uint8_t  leftover[2*USER_PAYLOAD_SIZE_MAX];
uint8_t  assembled_buf[2*USER_PAYLOAD_SIZE_MAX];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MainTask(void *argument);
void UartRxTask(void *argument);
void UartTxTask(void *argument);

/* USER CODE BEGIN PFP */
int UART1_Send(uint8_t *buf, uint16_t len);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
static void register_func(const char *name, func_ptr_t f);
static void UnpackUartFrame(uint8_t *buf, uint16_t len);
static int UnpackPayload(uint8_t *buf, uint16_t len, response_t *resp);
static int UnpackQueryMsg(uint8_t *buf, uint16_t len, response_t *resp);
static int my_str(const uint8_t *input, uint16_t in_len, uint8_t *param, uint16_t *param_len);
static int my_power(const uint8_t *input, uint16_t in_len, uint8_t *param, uint16_t *param_len);
static int my_sum(const uint8_t *input, uint16_t in_len, uint8_t *param, uint16_t *param_len);
static int MakeAppPayload(uint8_t* buf, response_t *msg, uint16_t *total_len);
static uint8_t crc8(const uint8_t *data, size_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CRC-8/ATM: poly 0x07, init 0x00, refin=1, refout=1, xorout=0 */
__attribute__((unused))
uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00; // initial value

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C; // 0x8C is 0x07 reflected
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

__attribute__((unused))
static void print_in_hex(const uint8_t *buf, size_t len)
{
  if (buf == NULL || len == 0)
    return;

// newline every 16 bytes
#define LINE_MAX_SYMBOLS_PRINTED        16

  for (size_t i = 0; i < len; i++) {
      printf("%02X ", buf[i]);
      if ((i + 1) % LINE_MAX_SYMBOLS_PRINTED == 0) {
          printf("\r\n");
      }
  }
  if (len % LINE_MAX_SYMBOLS_PRINTED != 0) {
      printf("\r\n");
  }
}

__attribute__((unused))
static void register_func(const char *name, func_ptr_t f)
{
  FuncEntry_t *node = (FuncEntry_t *)pvPortMalloc(sizeof(FuncEntry_t));
  if (!node) {
    printf("[register_func] ERROR: OOM %s\r\n", name);
    return;
  }
  node->name = name;
  node->func = f;
  node->next = g_funcs_head;
  g_funcs_head = node;
}

/* input: 2 x uint8_t bytes; out: 2 bytes LE = sum of input */
__attribute__((unused))
static int my_sum(const uint8_t *input, uint16_t in_len, uint8_t *param, uint16_t *param_len)
{
  uint16_t rez;

  if ((input == NULL) || (in_len < 2) || (param == NULL) || (param_len == 0)) {
    printf("[my_sum] ERROR: input param\r\n");
    return -1;
  }

  rez = (uint16_t)input[0] + (uint16_t)input[1];
  printf("\r\n[my_sum] arg[0]=%d (0x%x), arg[0]=%d (0x%x), rez=%d (0x%x)\r\n",
         input[0], input[0], input[1], input[1], rez, rez);

  /* Output uint16_t */
  param[0] = (uint8_t)(rez & 0xFF);
  param[1] = (uint8_t)(rez >> 8);
  *param_len = 2; // length of parameters in response message in bytes
  return 0;
}

/* input: 2 x uint8_t bytes;; out: 2 bytes LE = v*v (16-bit) */
__attribute__((unused))
static int my_power(const uint8_t *input, uint16_t in_len, uint8_t *param, uint16_t *param_len)
{
  uint16_t rez;

  if ((input == NULL) || (in_len < 2) || (param == NULL) || (param_len == 0)) {
    printf("[my_power] ERROR: input param\r\n");
    return -1;
  }

  rez = (uint16_t)(input[0]) * (uint16_t)(input[1]);
  printf("\r\n[my_power] arg[0]=%d (0x%x), arg[0]=%d (0x%x), rez=%d (0x%x)\r\n",
         input[0], input[0], input[1], input[1], rez, rez);

  /* Output uint16_t */
  param[0] = (uint8_t)(rez & 0xFF);
  param[1] = (uint8_t)(rez >> 8);
  *param_len = 2;
  return 0;
}

/* input: bytes; out: 2 bytes LE = strlen(input-as-bytes) */
__attribute__((unused))
static int my_str(const uint8_t *input, uint16_t in_len, uint8_t *param, uint16_t *param_len)
{
  (void)input;

  if ((input == NULL) || (in_len == 0) || (param == NULL) || (param_len == 0)) {
    printf("[my_str] ERROR: input param\r\n");
    return -1;
  }

  uint16_t rez = strlen((char*)input);
  /* Output uint16_t */
  param[0] = (uint8_t)(rez & 0xFF);
  param[1] = (uint8_t)(rez >> 8);
  *param_len = 2;
  return 0;
}

__attribute__((unused))
static void UnpackUartFrame(uint8_t *buf, uint16_t len)
{
  uint16_t idx = 0;
  uint16_t frame_len;
  uint8_t  calc_crc;
  uint16_t remain;
  uint8_t *hdr; // current header
  UartQueueData_t pm;

  if (buf == NULL) {
    return;
  }

  while (idx < len) {
    if (buf[idx] != FRAME_START_BYTE) {
      idx++;
      continue;
    }

    // FRAME_START_BYTE found, validating the rest
    if ((idx + FRAME_HEADER_CRC_OFFSET) > len)
      break; /* drop it if we don't get 3-bytes header */

    hdr = &buf[idx]; // current starting point

    // If header CRC is wrong, look for another start
    calc_crc = crc8(hdr, FRAME_HEADER_CRC_OFFSET);
    if (calc_crc != hdr[FRAME_HEADER_CRC_OFFSET]) {
      printf("\r\n[UnpackUartFrame] header CRC mismatch @%u (got 0x%02X exp 0x%02X)\r\n",
             idx, hdr[FRAME_HEADER_CRC_OFFSET], calc_crc);
      idx++;
      continue;
    }

    // Check if complete frame received
    frame_len = (uint16_t)(hdr[FRAME_HEADER_LSB_IDX] | ((uint16_t)hdr[FRAME_HEADER_MSB_IDX] << 8));

    if ((idx + frame_len) > len) {
      /* incomplete frame received */
      printf("\r\n[UnpackUartFrame] Invalid fame: got %u bytes, but expecting %u bytes\r\n", len, frame_len);

      /* Incomplete packet -> stash leftover from i..end */
      remain = (uint16_t)(len - idx);
      if (remain <= sizeof(leftover)) {
        memcpy(leftover, &buf[idx], remain);
        leftover_len = remain;
      } else {
        leftover_len = 0;
      }
      return; /* wait for next buffer to complete */
    }

    // We have a complete frame in the buffer
    // Check FRAME_PAYLOAD_START
    if (hdr[FRAME_PAYLOAD_OFFSET-1] != FRAME_PAYLOAD_START) {
      printf("\r\n[UnpackUartFrame] Invalid fame: missing payload-start 0x%02x\r\n", hdr[FRAME_PAYLOAD_OFFSET-1]);
      idx++;
      continue;
    }

    // Check FRAME_STOP_BYTE
    if (hdr[frame_len - 1] != FRAME_STOP_BYTE) {
      printf("\r\n[UnpackUartFrame] Invalid fame: missing frame stop byte 0x%02x\r\n", hdr[frame_len - 1]);
      idx++;
      continue;
    }

    /* CRC of the frame (except 2 last bytes) */
    calc_crc = crc8(hdr, (uint32_t)(frame_len - FRAME_TAIL_SIZE));
    if (calc_crc != hdr[frame_len - FRAME_TAIL_SIZE]) {
      printf("\r\n[UnpackUartFrame] header CRC mismatch @%u (got 0x%02X exp 0x%02X)\r\n",
             idx, hdr[frame_len - FRAME_TAIL_SIZE], calc_crc);
      idx++;
      continue;
    }

    pm.len = frame_len - FRAME_PAYLOAD_OFFSET - FRAME_TAIL_SIZE;
    memcpy(pm.data, &hdr[FRAME_PAYLOAD_OFFSET], pm.len);

    printf("\r\n[UnpackUartFrame] UART frame_len=%u, payload len=%u\r\n", frame_len, pm.len);
    print_in_hex(pm.data, pm.len);

    if (osMessageQueuePut(PayloadQueueRxHandle, &pm, 0U, 0U) != osOK) {
      printf("\r\n[UnpackUartFrame] PayloadQueueRxHandle full, drop\r\n");
    }

    idx += (uint16_t)frame_len; /* check next packet in the same buffer (if any) */
  }

  if (idx < len) {
    remain = (uint16_t)(len - idx);
    if (remain <= sizeof(leftover)) {
      memcpy(leftover, &buf[idx], remain);
      leftover_len = remain;
    } else {
      leftover_len = 0;
    }
  } else {
    leftover_len = 0;
  }
  return;
}

/**
 * @brief Unpack and call func
 * @param buf - input buffer
 * @param len - length of input buffer
 * @param resp - empty response structure to fill in
 * @retval 0 - if OK
 */
__attribute__((unused))
int UnpackQueryMsg(uint8_t *buf, uint16_t len, response_t *resp)
{
  uint8_t *ptr  = &buf[PROTO_MSG_NAME_OFFSET];
  uint16_t name_len = strlen((char*)ptr); // shortcut, no err handling
  uint16_t param_offset = name_len + 1;
  func_ptr_t func_p = NULL;
  FuncEntry_t *fe = NULL;
  int rez = 0;

  // update resp struct
  resp->type = PROTO_MSG_TYPE_ERROR;
  resp->idx = buf[1];
  memcpy(resp->name, ptr, name_len);
  resp->name_len = name_len;

  for (fe = g_funcs_head; fe != NULL; fe = fe->next) {
    if (strcmp((char*)fe->name, (char*)ptr) == 0) {
      func_p = fe->func;
      break;
    }
  }

  if (func_p == NULL) {
    printf("\r\n[UnpackQueryMsg] not found: %s\r\n", ptr);
    return -1;
  }

  rez = func_p(&ptr[param_offset], (len - param_offset), resp->param, &resp->param_len);
  if (rez) {
    printf("\r\n[UnpackQueryMsg] called func (%s) FAILED (%d)\r\n", fe->name, rez);
    return -2;
  } else {
    resp->type = PROTO_MSG_TYPE_RESPONSE;
  }

  return 0;
}

/**
 * @brief Unpack payload message and call function with parameters
 * @param buf - input buffer with payload structure
 * @param len - length of input buffer
 * @param resp - empty response structure to fill in
 * @retval 0 - if OK
 */
int UnpackPayload(uint8_t *buf, uint16_t len, response_t *resp)
{
  uint8_t *ptr = buf;
  int rez = 0;

  /* At least first 2 bytes */
  if (len < 2) {
    printf("\r\n[UnpackPayload] msg too short: %u\r\n", len);
    return -1;
  }

  if (*ptr == PROTO_MSG_TYPE_QUERY) {
    rez = UnpackQueryMsg(buf, len, resp);
    if (rez != 0) {
      printf("\r\n[UnpackPayload] UnpackQueryMessage() failed with (%d)\r\n", rez);
    }
    return rez;
  } else if (*ptr == PROTO_MSG_TYPE_STREAM) {
    printf("\r\n[UnpackPayload] Not supported: PROTO_MSG_TYPE_STREAM\r\n");
    return -2;
  } else if (*ptr == PROTO_MSG_TYPE_RESPONSE) {
    printf("\r\n[UnpackPayload] Not supported: PROTO_MSG_TYPE_RESPONSE\r\n");
    return -3;
  } else if (*ptr == PROTO_MSG_TYPE_ERROR) {
    printf("\r\n[UnpackPayload] Not supported: PROTO_MSG_TYPE_ERROR\r\n");
    return -4;
  } else {
    printf("\r\n[UnpackPayload] Unknown message type (0x%02x)\r\n", *ptr);
    return -5;
  }
}

/**
 * @brief Create response message
 * @param resp - input data
 * @param total_len - output length
 * @retval txbuf - allocated buf with payload
 */
__attribute__((unused))
int MakeAppPayload(uint8_t *txbuf, response_t *resp, uint16_t *total_len)
{
  // add terminator (NULL)
  *total_len = PROTO_MSG_NAME_OFFSET + resp->name_len + resp->param_len + 1;
  if (*total_len > USER_PAYLOAD_SIZE_MAX) {
    printf("\r\n[MakeAppPayload] ERROR: total_len=%u\r\n", *total_len);
    (*total_len) = 0;
    return -1;
  }

  txbuf[0] = resp->type;
  txbuf[1] = resp->idx;    // idx from received message
  memcpy(&txbuf[PROTO_MSG_NAME_OFFSET], resp->name, resp->name_len);
  txbuf[PROTO_MSG_NAME_OFFSET + resp->name_len] = 0x00;
  if (resp->param_len) {
    memcpy(&txbuf[PROTO_MSG_NAME_OFFSET + resp->name_len + 1], resp->param, resp->param_len);
  }
  return 0;
}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of PayloadQueueRx */
  PayloadQueueRxHandle = osMessageQueueNew (3, sizeof(UartQueueData_t), &PayloadQueueRx_attributes);

  /* creation of PayloadQueueTx */
  PayloadQueueTxHandle = osMessageQueueNew (3, sizeof(UartQueueData_t), &PayloadQueueTx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  UartRxStream = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_TRIGGER_LEVEL);
  if (UartRxStream == NULL) {
    printf("\r\n[main] ERROR: xStreamBufferCreate\r\n");
  }

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Main */
  MainHandle = osThreadNew(MainTask, NULL, &Main_attributes);

  /* creation of UartRx */
  UartRxHandle = osThreadNew(UartRxTask, NULL, &UartRx_attributes);

  /* creation of UartTx */
  UartTxHandle = osThreadNew(UartTxTask, NULL, &UartTx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t received_sz)
{
  BaseType_t hptw = pdFALSE;
  size_t bytes_sent = 0;

  if (huart->Instance == USART1)  {
    /* Push received bytes into the stream buffer */
    bytes_sent = xStreamBufferSendFromISR(UartRxStream, uart1_dma_buffer_rx, received_sz, &hptw);
    if (bytes_sent != received_sz) {
      // TODO add error count
    }

    /* Restart DMA reception */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_buffer_rx, UART_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    if (hptw == pdTRUE) {
      portYIELD_FROM_ISR(hptw);
    }
  }
}

int UART1_Send(uint8_t *buf, uint16_t len)
{
  if (buf == NULL || len == 0) {
    printf("\r\nUART1_Send(): HAL_UART_Transmit_DMA() input par ERROR len=%u\r\n", len);
    return -1;
  }

  /* Check if UART1 TX is ready for new transfer */
  if (huart1.gState != HAL_UART_STATE_READY) {
    // TODO add timeout to check ready again (avoid returning fault)
    printf("\r\nUART1_Send(): Tx is NOT ready\r\n");
    return -2; /* Previous transfer is not finished */
  }

  if (HAL_UART_Transmit_DMA(&huart1, buf, len) != HAL_OK) {
    printf("\r\nUART1_Send(): HAL_UART_Transmit_DMA() failed\r\n");
    return -3;
  }
  return 0;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle.
  * @note   reports end of DMA Tx transfer
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    /* Set transmission flag: transfer complete */
    // TODO notify a task
  }
  return;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  char buf[32];
  uint32_t error = HAL_UART_GetError(UartHandle);

  switch(error) {
    case HAL_UART_ERROR_PE:
      strcpy(buf, "Parity Error");
      break;
    case HAL_UART_ERROR_NE:
      strcpy(buf, "Noise Error");
      break;
    case HAL_UART_ERROR_FE:
      strcpy(buf, "Frame Error");
      break;
    case HAL_UART_ERROR_ORE:
      strcpy(buf, "Overrun Error");
      break;
    case HAL_UART_ERROR_DMA:
      strcpy(buf, "DMA Transfer Error");
      break;
    case HAL_UART_ERROR_RTO:
      strcpy(buf, "Receiver Timeout Error");
      break;
    default:
      strcpy(buf, "Unknown Error");
      itoa(error, &buf[16], 10);
  }
  printf("HAL_UART_ErrorCallback(): %s\r\n", buf);
  Error_Handler();
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_MainTask */
/**
  * @brief  Function implementing the Main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MainTask */
void MainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  UartQueueData_t pm;
  int rez = 0;
  response_t resp;

  printf("\r\n[MainTask] Started\r\n");

  /* Register functions (3 examples) */
  register_func("sum2x", my_sum);
  register_func("power2", my_power);
  register_func("strlen", my_str);

  for (;;) {
    if (osMessageQueueGet(PayloadQueueRxHandle, &pm, NULL, osWaitForever) == osOK) {
      memset(&resp, 0, sizeof(resp));
      /* Unpacking client messages and executing function (if query) */
      rez = UnpackPayload(pm.data, pm.len, &resp);
      if (rez == 0) {
        printf("\r\n[MainTask] UnpackPayload(): OK\r\n");
      } else {
        printf("\r\n[MainTask] UnpackPayload: 0x%0x\r\n", rez);
      }

      pm.len = 0;
      //  Make Response Message
      if (MakeAppPayload(pm.data, &resp, &pm.len)) {
        pm.len = 0;
        continue;
      }

      // send it to Tx task
      if (osMessageQueuePut(PayloadQueueTxHandle, &pm, 0U, 0U) != osOK) {
        printf("\r\n[MainTask]  PayloadQueueTxHandle full, drop\r\n");
        pm.len = 0;
      } else {
      }
    }
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UartRxTask */
/**
* @brief Function implementing the UartRx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartRxTask */
void UartRxTask(void *argument)
{
  /* USER CODE BEGIN UartRxTask */
    size_t uart_buf_len = 0;
    uint32_t total;

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    // get DMA receive going
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_buffer_rx, UART_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    osDelay(50);
    printf("\r\n[UartRx] Started\r\n");

    for(;;) {
      /* read data from stream buffer */
      uart_buf_len = xStreamBufferReceive(UartRxStream, uart_task_buf_rx, USER_PAYLOAD_SIZE_MAX, portMAX_DELAY);
      if (uart_buf_len <= 0) {
        continue;
      }
      // debug indication
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

      printf("\r\n[UartRx] ISR buf received len=%u\r\n", uart_buf_len);
      print_in_hex(uart_task_buf_rx, uart_buf_len);

      /* was any part received in previous frame? */
      if (leftover_len) {
        total = leftover_len + uart_buf_len;

        if (total > USER_PAYLOAD_SIZE_MAX) {
          /* if too big, drop */
          leftover_len = 0;
          continue;
        }

        memcpy(assembled_buf, leftover, leftover_len);
        memcpy((uint8_t*)assembled_buf + leftover_len, uart_task_buf_rx, uart_buf_len);
        UnpackUartFrame(assembled_buf, (uint16_t)total);
      } else {
        UnpackUartFrame(uart_task_buf_rx, uart_buf_len);
      }
    }
  /* USER CODE END UartRxTask */
}

/* USER CODE BEGIN Header_UartTxTask */
/**
* @brief Function implementing the UartTx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartTxTask */
void UartTxTask(void *argument)
{
  /* USER CODE BEGIN UartTxTask */
  UartQueueData_t tx;
  uint16_t pkt_sz;
  uint8_t *pkt = NULL;

  (void) argument;

  osDelay(20);
  printf("\r\n[UartTx] Started\r\n");

  for (;;) {
    if (osMessageQueueGet(PayloadQueueTxHandle, &tx, NULL, osWaitForever) == osOK) {
      if (tx.len == 0) {
        printf("\r\n[UartTx]: ERROR zero length Msg from PayloadQueueTxHandle\r\n");
        continue;
      }

      /*** Pack UART frame ***/
      /* Header size + start data byte + payload len + checksum + end byte */
      pkt_sz = FRAME_PAYLOAD_OFFSET + tx.len + FRAME_TAIL_SIZE;
      pkt = uart_task_buf_tx;

      /* Header */
      pkt[0] = FRAME_START_BYTE;
      // length of the whole UART frame
      pkt[FRAME_HEADER_LSB_IDX] = (uint8_t) (pkt_sz & 0xFF);
      pkt[FRAME_HEADER_MSB_IDX] = (uint8_t) (pkt_sz >> 8);
      pkt[FRAME_HEADER_CRC_OFFSET] = crc8(pkt, FRAME_HEADER_CRC_OFFSET);
      pkt[FRAME_HEADER_SBD_OFFSET] = FRAME_PAYLOAD_START;

      // payload
      memcpy(&pkt[FRAME_PAYLOAD_OFFSET], tx.data, tx.len);

      /* Packet CRC over header+marker+payload */
      pkt[FRAME_PAYLOAD_OFFSET + tx.len] = crc8(pkt, (uint32_t) (FRAME_PAYLOAD_OFFSET + tx.len));
      pkt[FRAME_PAYLOAD_OFFSET + tx.len + 1] = FRAME_STOP_BYTE;

      printf("\r\n[UartTx]: UART Frame prepared len=%u\r\n", pkt_sz);
      print_in_hex(pkt, pkt_sz);

      if (UART1_Send(pkt, pkt_sz)) {
        // TODO implement resend
        printf("\r\n[UartTx]: UART1_Send() failed\r\n");
      } else {
        printf("\r\n[UartTx]: UART1_Send() OK\r\n");
      }
      tx.len = 0;
    }
  }

  /* USER CODE END UartTxTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
