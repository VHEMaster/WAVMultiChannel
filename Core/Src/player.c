/*
 * player.c
 *
 *  Created on: Aug 10, 2022
 *      Author: VHEMaster
 */

#include "player.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "bsp_driver_sd.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <limits.h>

osThreadId_t taskIdleHandle;

const osThreadAttr_t taskIdle_attributes = {
  .name = "taskIdle",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadAttr_t taskPlayer_attributes = {
  .name = "taskPlayer",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osMutexAttr_t mutexMp3_attributes = {
    .name= "mutexMp3",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

const osMutexAttr_t mutexFS_attributes = {
    .name= "mutexFS",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

const osMutexAttr_t mutexCommTx_attributes = {
    .name= "mutexCommTx",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

osMutexAttr_t mutexPlayer_attributes = {
    .name= "mutexPlayer",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockB1;
extern SAI_HandleTypeDef hsai_BlockA2;
extern SAI_HandleTypeDef hsai_BlockB2;


extern TIM_HandleTypeDef htim5;

extern UART_HandleTypeDef huart3;


#define UART_CMD_BUFFER 64
#define UART_TX_BUFFER 256
#define UART_RX_BUFFER 256

#define TDM_COUNT           (4)
#define CHANNELS_COUNT      (64)
#define PLAYERS_COUNT       (32)
#define CHANNELS_PER_TDM    (CHANNELS_COUNT / TDM_COUNT)
#define SAMPLE_BUFFER_SIZE  (2048)
#define FILE_BUFFER_SIZE    (16384)
#define MAX_FILE_PATH       (128)
#define TDM_TEMP_BUFFER_SIZE (TDM_BUFFER_SIZE / CHANNELS_PER_TDM)
#define TDM_TEMP_HALF_BUFFER_SIZE (TDM_TEMP_BUFFER_SIZE / 2)

uint8_t gMixer[CHANNELS_COUNT][PLAYERS_COUNT];

int16_t gSamplesBuffer[PLAYERS_COUNT][SAMPLE_BUFFER_SIZE];
uint8_t gFileBuffer[PLAYERS_COUNT][FILE_BUFFER_SIZE];
osMutexId_t gMp3Mutex;
osMutexId_t gFSMutex;
osThreadId_t gTaskIdle;

volatile float gCpuLoad = 0;
volatile float gCpuLoadMax = 0;
volatile uint32_t gIdleTick = 1000;
volatile uint32_t gMp3LedLast = 0;

#define TDM_BUFFER_SIZE (256 * CHANNELS_PER_TDM)
static SAI_HandleTypeDef  *const gSai[TDM_COUNT] = { &hsai_BlockA1, &hsai_BlockB1, &hsai_BlockA2, &hsai_BlockB2 };
static int16_t gTdmFinalBuffers[TDM_COUNT][TDM_BUFFER_SIZE] __attribute__((aligned(32)));
static int16_t gTdmTempHalfBuffers[PLAYERS_COUNT][TDM_TEMP_HALF_BUFFER_SIZE];

typedef struct {
    uint32_t index;
    const char *name;
    char file[MAX_FILE_PATH];
    volatile uint8_t enabled;
    volatile uint8_t playing;
    volatile uint8_t play_once;
    volatile uint8_t changed;
    uint8_t volume;
    osThreadId_t task;
    osMutexId_t mutex;
    uint32_t bufferSize;
    int16_t *buffer;
    uint8_t *fileBuffer;
    uint32_t fileBufferSize;
    uint32_t buffer_rd;
    uint32_t buffer_wr;
    uint32_t underflow_rd;
    uint32_t fs_errors;
    float file_percentage;
}sPlayerData;

typedef struct {
    uint32_t count;
    sPlayerData player[PLAYERS_COUNT];
}sPlayersData;

typedef struct {
    uint8_t rx_buffer[UART_RX_BUFFER];
    char tx_buffer[UART_TX_BUFFER];
    char rx_cmd[UART_CMD_BUFFER];
    UART_HandleTypeDef *huart;
    uint32_t rx_rd;
    uint32_t rx_wr;
    uint32_t rx_cmd_ptr;
    osSemaphoreId_t tx_sem;
    osMutexId_t mutex;
}sCommData __attribute__((aligned(32)));

sCommData gCommData = {0};
sPlayersData gPlayersData = { PLAYERS_COUNT };

static FATFS gFatFS;

static void StartIdleTask(void *args);
static void StartPlayerTask(void *args);

static void StartIdleTask(void *args)
{
  while(1) {
    gIdleTick++;
    osDelay(1);
  }
}


static inline uint32_t getavail(uint32_t wr, uint32_t rd, uint32_t size) {
  if(wr >= rd) return (wr - rd);
  else return (size - rd + wr);
}

static inline uint32_t getfree(uint32_t wr, uint32_t rd, uint32_t size) {
  return size - getavail(wr, rd, size);
}

static void HandleSaiDma(int sai_index, int16_t *buffer, uint32_t size)
{
  uint32_t samples_per_channel = size / CHANNELS_PER_TDM;
  int buffer_index;
  int channel_index;
  int32_t value;
  int16_t data;
  int available[PLAYERS_COUNT] = {0};

  for(int j = 0; j < samples_per_channel; j++)
  {
    for(int i = 0; i < CHANNELS_PER_TDM; i++)
    {
      channel_index = i + sai_index * CHANNELS_PER_TDM;
      buffer_index = j * CHANNELS_PER_TDM + i;
      value = 0;

      for(int player = 0; player < PLAYERS_COUNT; player++) {
        if(gMixer[channel_index][player]) {
          if(!available[player]) {
            available[player] = getavail(gPlayersData.player[player].buffer_wr, gPlayersData.player[player].buffer_rd, gPlayersData.player[player].bufferSize) >= samples_per_channel ? 1 : -1;
            if(available[player] == -1) {
              if(gPlayersData.player[player].playing) {
                gPlayersData.player[player].underflow_rd++;
              }
            }
          }
          if(available[player] > 0) {
            data = 0;
            if(available[player] == 1) {
              data = gPlayersData.player[player].buffer[gPlayersData.player[player].buffer_rd];
              if(gPlayersData.player[player].buffer_rd + 1 >= gPlayersData.player[player].bufferSize)
                gPlayersData.player[player].buffer_rd = 0;
              else gPlayersData.player[player].buffer_rd++;

              gTdmTempHalfBuffers[player][j] = data;
              available[player] = 2;
            } else if(available[player] == 2) {
              data = gTdmTempHalfBuffers[player][j];
            }

            value += data;
          }
        }
      }

      buffer[buffer_index] = value > SHRT_MAX ? SHRT_MAX : value < SHRT_MIN ? SHRT_MIN : value;

    }
  }

  SCB_CleanDCache_by_Addr((uint32_t *)buffer, size * sizeof(*buffer));
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  for(int i = 0; i < TDM_COUNT; i++) {
    if(gSai[i] == hsai) {
      HandleSaiDma(i, &gTdmFinalBuffers[i][TDM_BUFFER_SIZE / 2], TDM_BUFFER_SIZE / 2);
    }
  }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  for(int i = 0; i < TDM_COUNT; i++) {
    if(gSai[i] == hsai) {
      HandleSaiDma(i, &gTdmFinalBuffers[i][0], TDM_BUFFER_SIZE / 2);
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == gCommData.huart) {
    osSemaphoreRelease(gCommData.tx_sem);
  }
}

osStatus_t Comm_Transmit(sCommData *commdata, char *cmd, ...)
{
  osStatus_t sem_status;
  int strl;
  va_list va_args;

  sem_status = osMutexAcquire(commdata->mutex, osWaitForever);
  if(sem_status == osOK) {
    sem_status = osSemaphoreAcquire(commdata->tx_sem, osWaitForever);
    if(sem_status == osOK) {
      va_start(va_args, cmd);
      vsnprintf(commdata->tx_buffer, UART_TX_BUFFER - 4, cmd, va_args);
      va_end(va_args);
      strl = strlen(commdata->tx_buffer);
      commdata->tx_buffer[strl++] = '\r';
      commdata->tx_buffer[strl++] = '\n';

      SCB_CleanDCache_by_Addr((uint32_t *)commdata->tx_buffer, strl);
      if(HAL_UART_Transmit_DMA(commdata->huart, (uint8_t *)commdata->tx_buffer, strl) != HAL_OK) {
        osSemaphoreRelease(gCommData.tx_sem);
        sem_status = osError;
      }
    }
    osMutexRelease(commdata->mutex);
  }

  return sem_status;
}

int Comm_RxProcess(sCommData *commdata, char **command, uint32_t *size)
{
  char chr;

  *command = NULL;

  commdata->rx_wr = commdata->huart->RxXferSize - ((DMA_Stream_TypeDef *)commdata->huart->hdmarx->Instance)->NDTR;

  while(commdata->rx_wr != commdata->rx_rd) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)commdata->rx_buffer, commdata->huart->RxXferSize);
    chr = commdata->rx_buffer[commdata->rx_rd++];
    if(commdata->rx_rd >= UART_RX_BUFFER || commdata->rx_rd >= commdata->huart->RxXferSize) {
      commdata->rx_rd = 0;
    }

    if(commdata->rx_cmd_ptr + 1 >= UART_CMD_BUFFER || chr == '\r' || chr == '\n') {
      commdata->rx_cmd[commdata->rx_cmd_ptr] = '\0';
      if(commdata->rx_cmd_ptr > 0) {
        if(command) {
          memset(&commdata->rx_cmd[commdata->rx_cmd_ptr], 0, UART_CMD_BUFFER - commdata->rx_cmd_ptr);
          *command = commdata->rx_cmd;
        }
        if(size) {
          *size = commdata->rx_cmd_ptr;
        }
        commdata->rx_cmd_ptr = 0;
        return 1;
      }
    } else {
      commdata->rx_cmd[commdata->rx_cmd_ptr++] = chr;
    }
  }
  return 0;
}


void player_tick(void)
{

}

void player_start(void)
{
  uint64_t time = 0;
  sSdStats sdstats;
  sPlayersData *playersdata = &gPlayersData;
  char *str = NULL;
  char *args = NULL;
  uint32_t channel;
  uint32_t enabled;
  uint32_t play_once;
  uint32_t len = 0;
  uint32_t arg;
  uint32_t tick;
  uint32_t current;
  uint32_t arg_start, arg_end;
  const float msr_koff = 0.0166667f;
  float sd_avg = 0;
  float ram_avg = 0;
  float cpu_avg = 0;

  FRESULT res;

  HAL_GPIO_WritePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin, GPIO_PIN_SET);
  do {
    res = f_mount(&gFatFS, SDPath, 1);
    HAL_GPIO_WritePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(LED3_RED_GPIO_Port, LED3_RED_Pin);
  } while(res != FR_OK);

  HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_RED_GPIO_Port, LED3_RED_Pin, GPIO_PIN_RESET);


  memset(&gCommData, 0, sizeof(gCommData));
  gCommData.huart = &huart3;
  gCommData.tx_sem = osSemaphoreNew(1, 1, NULL);
  gCommData.mutex = osMutexNew(&mutexCommTx_attributes);
  HAL_UART_Receive_DMA(gCommData.huart, gCommData.rx_buffer, UART_RX_BUFFER);

  memset(gTdmFinalBuffers, 0, sizeof(gTdmFinalBuffers));
  memset(gMixer, 0, sizeof(gMixer));

  gFSMutex = osMutexNew(&mutexFS_attributes);
  for(int i = 0; i < PLAYERS_COUNT; i++) {
    memset(&playersdata->player[i], 0, sizeof(playersdata->player[i]));
    if(i * 2 < CHANNELS_COUNT)
      gMixer[i * 2][i] = 1;

    str = (char *)pvPortMalloc(12); sprintf(str, "player%d", i+1); playersdata->player[i].name = str;
    str = (char *)pvPortMalloc(16); sprintf(str, "mutexPlayer%d", i+1); mutexPlayer_attributes.name = str;
    str = (char *)pvPortMalloc(16); sprintf(str, "taskPlayer%d", i+1); taskPlayer_attributes.name = str;

    sprintf(playersdata->player[i].file, "music/%d.wav", i+1);

    playersdata->player[i].index = i;
    playersdata->player[i].enabled = 0;
    playersdata->player[i].playing = 0;
    playersdata->player[i].volume = 100;

    playersdata->player[i].bufferSize = SAMPLE_BUFFER_SIZE;
    playersdata->player[i].buffer = gSamplesBuffer[i];
    playersdata->player[i].fileBufferSize = FILE_BUFFER_SIZE;
    playersdata->player[i].fileBuffer = gFileBuffer[i];

    playersdata->player[i].task = osThreadNew(StartPlayerTask, &playersdata->player[i], &taskPlayer_attributes);
  }

  for(int i = 0; i < TDM_COUNT; i++) {
    HandleSaiDma(i, gTdmFinalBuffers[i], TDM_BUFFER_SIZE);
    HAL_SAI_Transmit_DMA(gSai[i], (uint8_t *)gTdmFinalBuffers[i], TDM_BUFFER_SIZE);
  }

  taskIdleHandle = osThreadNew(StartIdleTask, NULL, &taskIdle_attributes);
  HAL_TIM_Base_Start_IT(&htim5);

  current = xTaskGetTickCount();
  tick = 0;

  for(;;)
  {
    if(Comm_RxProcess(&gCommData, &str, &len)) {
      for(int i = 0; i < gPlayersData.count; i++) {
        if(strstr(str, gPlayersData.player[i].name) == str && str[strlen(gPlayersData.player[i].name)] == '.') {
          str += strlen(gPlayersData.player[i].name) + 1;
          arg_start = -1;
          arg_end = -1;
          args = strchr(str, '(');
          if(args == NULL)
            break;
          arg_start = args - str;
          while(args != NULL) {
            args = strchr(args + 1, ')');
            if(args == NULL)
              break;
            arg_end = args - str;
          }
          if(arg_end == -1)
            break;
          if(arg_start < arg_end) {
            args = &str[arg_start + 1];
            str[arg_start] = '\0';
            str[arg_end] = '\0';
            if((play_once = (strcmp(str, "play_once") == 0)) || (strcmp(str, "play") == 0)) {
              osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
              strcpy(gPlayersData.player[i].file, args);
              gPlayersData.player[i].enabled = 1;
              gPlayersData.player[i].play_once = play_once;
              gPlayersData.player[i].changed = 1;
              osMutexRelease(gPlayersData.player[i].mutex);
              Comm_Transmit(&gCommData, "OK");
            } else if(strcmp(str, "mixer") == 0) {
              osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
              if(sscanf(args, "%lu,%lu", &channel, &enabled) == 2) {
                channel -= 1;
                if(channel < CHANNELS_COUNT && (enabled == 1 || enabled == 0)) {
                  gMixer[channel][i] = enabled;
                  Comm_Transmit(&gCommData, "OK");
                }
              }
              osMutexRelease(gPlayersData.player[i].mutex);
            } else if(strcmp(str, "mixer_disable") == 0) {
              osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
              for(channel = 0; channel < CHANNELS_COUNT; channel++)
                gMixer[channel][i] = 0;
              osMutexRelease(gPlayersData.player[i].mutex);
              Comm_Transmit(&gCommData, "OK");
            } else if(strcmp(str, "stop") == 0) {
              osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
              gPlayersData.player[i].enabled = 0;
              gPlayersData.player[i].changed = 1;
              osMutexRelease(gPlayersData.player[i].mutex);
              Comm_Transmit(&gCommData, "OK");
            } else if(strcmp(str, "vol") == 0) {
              if(sscanf(args, "%ld", &arg) == 1) {
                osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
                gPlayersData.player[i].volume = arg;
                osMutexRelease(gPlayersData.player[i].mutex);
                Comm_Transmit(&gCommData, "OK");
              }
            }
          }
          break;
        }
      }
    }

    taskENTER_CRITICAL();
    if(xTaskGetTickCount() - gMp3LedLast > 1000) {
      gMp3LedLast = xTaskGetTickCount();
      HAL_GPIO_WritePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin, GPIO_PIN_RESET);
    }
    taskEXIT_CRITICAL();

    current += 30;
    osDelayUntil(current);
    if(++tick >= 1000/30) {
      tick = 0;
      time++;

      sdstats = BSP_SD_GetStats();

      if(gIdleTick > 1000)
        gIdleTick = 1000;

      gCpuLoad = (float)(1000 - gIdleTick) * 0.1f;
      gIdleTick = 0;

      if(gCpuLoadMax < gCpuLoad)
        gCpuLoadMax = gCpuLoad;

      if(cpu_avg == 0.0f)
        cpu_avg = gCpuLoad;
      else cpu_avg = cpu_avg * (1.0f - msr_koff) + gCpuLoad * msr_koff;

      if(sd_avg == 0.0f)
        sd_avg = sdstats.readLast;
      else sd_avg = sd_avg * (1.0f - msr_koff) + sdstats.readLast * msr_koff;

      if(ram_avg == 0.0f)
        ram_avg =  xPortGetFreeHeapSize();
      else ram_avg = ram_avg * (1.0f - msr_koff) + xPortGetFreeHeapSize() * msr_koff;

      Comm_Transmit(&gCommData, "SD: %.1fKB/s Max: %.1fKB/s Avg: %.1fKB/s\r\n"
          "CPU: %.1f%% Max: %.1f%% Avg: %.1f%%\r\n"
          "RAM: %.2fKB Min: %.1fKB Avg: %.2fKB",

          (float)sdstats.readLast / 1024.0f,
          (float)sdstats.readMax / 1024.0f,
          sd_avg / 1024.0f,

          gCpuLoad,
          gCpuLoadMax,
          cpu_avg,

          (float)xPortGetFreeHeapSize() / 1024.0f,
          (float)xPortGetMinimumEverFreeHeapSize() / 1024.0f,
          ram_avg / 1024.0f);

    }
  }
}

static volatile uint32_t gOpened = 0;
static volatile uint32_t gWriten = 0;
static volatile uint32_t times[PLAYERS_COUNT];
static volatile float speeds[PLAYERS_COUNT];

void StartPlayerTask(void *args)
{
  sPlayerData *playerdata = (sPlayerData *)args;
  FRESULT res;
  FIL file;
  uint32_t size;
  uint32_t block;
  uint32_t left;
  uint32_t time;
  UINT read = 0;


  osMutexAcquire(gFSMutex, osWaitForever);
  res = f_open(&file, playerdata->file, FA_READ);
  osMutexRelease(gFSMutex);
  if(res != FR_OK) {
    vTaskSuspendAll();
    while(1) {
      HAL_Delay(500);
      HAL_GPIO_TogglePin(LED3_RED_GPIO_Port, LED3_RED_Pin);
    }
  }

  taskENTER_CRITICAL();
  gOpened++;
  taskEXIT_CRITICAL();

  time = xTaskGetTickCount();

  size = f_size(&file);
  left = size;

  while(1) {
    time = xTaskGetTickCount();
    gMp3LedLast = time;

    block = left;
    if(block > playerdata->fileBufferSize)
      block = playerdata->fileBufferSize;

    osMutexAcquire(gFSMutex, osWaitForever);
    res = f_read(&file, playerdata->fileBuffer, block, &read);
    osMutexRelease(gFSMutex);

    if(res != FR_OK) {
      vTaskSuspendAll();
      while(1) {
        HAL_Delay(500);
        HAL_GPIO_TogglePin(LED3_RED_GPIO_Port, LED3_RED_Pin);
      }
    }

    left -= block;
    if(!left) {
      osMutexAcquire(gFSMutex, osWaitForever);
      res = f_rewind(&file);
      osMutexRelease(gFSMutex);

      left = size;

      times[playerdata->index] = xTaskGetTickCount() - time;
      speeds[playerdata->index] = ((float)size / ((float)times[playerdata->index] / 1000.0f)) / 1024.0f / 1024.0f;
      if(playerdata->index == 0)
        HAL_GPIO_TogglePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin);
    }
    //osDelay(1);
  }
}

