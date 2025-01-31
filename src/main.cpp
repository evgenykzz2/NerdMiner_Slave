#include <Arduino.h>
#include <driver/i2c.h>
#include <mutex>
#include <list>
#include <memory>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include "nerdSHA256plus.h"

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#define HARDWARE_SHA265
#endif

#ifdef HARDWARE_SHA265
#include <sha/sha_dma.h>
#include <hal/sha_hal.h>
#include <hal/sha_ll.h>
#endif

#define I2C_PORT 0
#define I2C_ADDRESS 0x11

#ifdef CONFIG_IDF_TARGET_ESP32S2
#define PIN_I2C_SDA 33
#define PIN_I2C_SCL 35
#endif

#ifdef CONFIG_IDF_TARGET_ESP32C3
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 10
#endif

#define I2C_SLAVE_TX_BUF_LEN 1024
#define I2C_SLAVE_RX_BUF_LEN 1024

static i2c_config_t s_i2c_config;
static intr_handle_t s_i2c_slave_intr_handle;
static TaskHandle_t s_miner_task_0 = NULL;
static TaskHandle_t s_miner_task_1 = NULL;

void minerWorkerSw(void * task_id);
#ifdef HARDWARE_SHA265
void minerWorkerHw(void * task_id);
#endif

void setup()
{
  Serial.begin(115200);
  Serial.println("NerdMiner Slave module");

  for (uint32_t n = 0; ;++n)
  {
    if (n != 0)
      delay(1000);

    memset(&s_i2c_config, 0, sizeof(s_i2c_config));
    s_i2c_config.mode = I2C_MODE_SLAVE;
    s_i2c_config.sda_io_num = PIN_I2C_SDA;
    s_i2c_config.scl_io_num = PIN_I2C_SCL;
    s_i2c_config.sda_pullup_en = GPIO_PULLUP_DISABLE;
    s_i2c_config.scl_pullup_en = GPIO_PULLUP_DISABLE;
    s_i2c_config.slave.addr_10bit_en = 0;
    s_i2c_config.slave.slave_addr = I2C_ADDRESS;
    s_i2c_config.slave.maximum_speed = 100000;
    s_i2c_config.clk_flags = 0;

    if (i2c_param_config(I2C_PORT, &s_i2c_config)!= ESP_OK)
    {
      Serial.println("NerdMinerSlave: i2c_param_config error");
      continue;
    }
    
    if (i2c_driver_install(I2C_PORT, s_i2c_config.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0) != ESP_OK )
    {
      Serial.println("NerdMinerSlave: i2c_driver_install error");
      continue;
    }

    //if (i2c_isr_register(I2C_PORT, i2c_callback, 0, 0, &s_i2c_slave_intr_handle) != ESP_OK)
    //{
      //Serial.println("i2c_isr_register: i2c_driver_install error");
      //continue;
    //}
    
    break;
  }

  #ifdef HARDWARE_SHA265
  xTaskCreate(minerWorkerHw, "MinerHw-0", 6000, (void*)0, 1, &s_miner_task_0);
  #else
  xTaskCreate(minerWorkerSw, "MinerSw-0", 6000, (void*)0, 1, &s_miner_task_0);
  #endif
  esp_task_wdt_add(s_miner_task_0);

  #if (SOC_CPU_CORES_NUM >= 2)
  xTaskCreate(minerWorkerSw, "MinerSw-1", 6000, (void*)1, 1, &s_miner_task_1);
  esp_task_wdt_add(s_miner_task_1);
  #endif
}

const uint8_t s_crc8_table[256] =
{
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

uint8_t CommandCrc8(const void* data, size_t len)
{
  const uint8_t* ptr = (const uint8_t*)data;
  uint8_t crc = 0xFF;
  crc = s_crc8_table[crc ^ ptr[0]];
  for (size_t n = 2; n < len; ++n)
      crc = s_crc8_table[crc ^ ptr[n]];
  return crc;
}

#define I2C_CMD_FEED 0xA1
#define I2C_CMD_REQUEST_RESULT 0xA9
#define I2C_CMD_SLAVE_RESULT 0xAA

struct __attribute__((__packed__)) JobI2cRequest
{
  //84 bytes
  uint8_t cmd;
  uint8_t crc;
  uint8_t id;
  uint8_t nonce_start;
  float difficulty;
  uint8_t buffer[76];
};

struct __attribute__((__packed__)) JobI2cResult
{
  //11 bytes
  uint8_t cmd;
  uint8_t crc;
  uint8_t id;
  uint32_t nonce;
  uint32_t processed_nonce;
};

struct JobRequest
{
  uint32_t id;
  uint32_t nonce_start;
  uint32_t nonce_count;
  double difficulty;
  uint8_t buffer_upper[64];
  uint32_t midstate[8];
  uint32_t bake[16];
};

struct JobResult
{
  uint32_t id;
  uint32_t nonce;
  uint32_t nonce_count;
  double difficulty;
  uint8_t hash[32];
};

#define NONCE_PER_JOB_SW 4096
#define NONCE_PER_JOB_HW 16*1024

static std::mutex s_job_mutex;
std::list<std::shared_ptr<JobRequest>> s_job_request_list_sw;
#ifdef HARDWARE_SHA265
std::list<std::shared_ptr<JobRequest>> s_job_request_list_hw;
#endif
std::list<std::shared_ptr<JobResult>> s_job_result_list;

static uint32_t s_nonce_pool = 0xFFFFFFFF;
static uint8_t s_job_id = 0;
static double s_pool_difficulty = 0.001;
static uint8_t s_sha_buffer[128];

uint32_t s_hw_midstate[8];
uint32_t s_diget_mid[8];
uint32_t s_bake[16];

static uint32_t s_best_job_id = 0xFFFFFFFF;
static uint32_t s_best_nonce = 0xFFFFFFFF;
static double s_best_diff = 0.0;
static uint32_t s_slave_nonces = 0;

static void JobPush(std::list<std::shared_ptr<JobRequest>> &job_list,  uint32_t id, uint32_t nonce_start, uint32_t nonce_count, double difficulty,
                    const uint8_t* buffer_upper, const uint32_t* midstate, const uint32_t* bake)
{
  std::shared_ptr<JobRequest> job = std::make_shared<JobRequest>();
  job->id = id;
  job->nonce_start = nonce_start;
  job->nonce_count = nonce_count;
  job->difficulty = difficulty;
  memcpy(job->buffer_upper, buffer_upper, sizeof(job->buffer_upper));
  memcpy(job->midstate, midstate, sizeof(job->midstate));
  memcpy(job->bake, bake, sizeof(job->bake));
  job_list.push_back(job);
}

static void HostCommand_Request()
{
  //Serial.printf("Request: diff=%.4f, nonce=0x%X id=0x%X\n", s_best_diff, s_best_nonce, s_best_job_id);
  //if (s_best_nonce == 0xFFFFFFFF)
    //return;

  JobI2cResult result;
  result.cmd = I2C_CMD_SLAVE_RESULT;
  result.id = s_best_job_id & 0xFF;
  result.nonce = s_best_nonce;
  result.processed_nonce = s_slave_nonces;
  result.crc = CommandCrc8(&result, sizeof(result));
  
  //if (s_best_nonce != 0xFFFFFFFF)
    //Serial.printf("Sending hash: diff=%.4f, nonce=0x%X, total=%d\n", s_best_diff, s_best_nonce, s_slave_nonces);

  if (i2c_slave_write_buffer(I2C_PORT, (const uint8_t*)&result, sizeof(result), 500 / portTICK_RATE_MS) > 0)
  {
    s_best_job_id = 0xFFFFFFFF;
    s_best_nonce = 0xFFFFFFFF;
    s_best_diff = 0.0;
    s_slave_nonces = 0;
  }
}

static void HostCommand_Feed(const JobI2cRequest* request)
{
  Serial.printf("Job Got id=0x%X, nonce=0x%X\n", (uint32_t)request->id, ((uint32_t)request->nonce_start) << 24);
  {
    std::lock_guard<std::mutex> lock(s_job_mutex);
    s_job_request_list_sw.clear();
    #ifdef HARDWARE_SHA265
    s_job_request_list_hw.clear();
    #endif
  }

  s_best_job_id = 0xFFFFFFFF;
  s_best_nonce = 0xFFFFFFFF;
  s_best_diff = 0.0;

  s_nonce_pool = ((uint32_t)request->nonce_start) << 24;
  s_job_id = request->id;
  s_pool_difficulty = request->difficulty;
  memcpy(s_sha_buffer, request->buffer, sizeof(request->buffer));

  memset(s_sha_buffer+76, 0, 128-76);
  s_sha_buffer[80] = 0x80;
  s_sha_buffer[126] = 0x02;
  s_sha_buffer[127] = 0x80;

  nerd_mids(s_diget_mid, s_sha_buffer);
  nerd_sha256_bake(s_diget_mid, s_sha_buffer+64, s_bake);

  #ifdef HARDWARE_SHA265
  esp_sha_acquire_hardware();
  sha_hal_hash_block(SHA2_256,  s_sha_buffer, 64/4, true);
  sha_hal_read_digest(SHA2_256, s_hw_midstate);
  esp_sha_release_hardware();
  #endif

  {
    std::lock_guard<std::mutex> lock(s_job_mutex);
    for (int i = 0; i < 4; ++ i)
    {
      JobPush( s_job_request_list_sw, s_job_id, s_nonce_pool, NONCE_PER_JOB_SW, s_pool_difficulty, s_sha_buffer+64, s_diget_mid, s_bake);
      s_nonce_pool += NONCE_PER_JOB_SW;
      #ifdef HARDWARE_SHA265
      JobPush( s_job_request_list_sw, s_job_id, s_nonce_pool, NONCE_PER_JOB_HW, s_pool_difficulty, s_sha_buffer+64, s_hw_midstate, s_bake);
      s_nonce_pool += NONCE_PER_JOB_HW;
      #endif
    }
  }
}

static uint8_t s_recieve_buffer[256];
static uint32_t s_recieve_buffer_pos = 0;
static int64_t s_time_statistick_us = esp_timer_get_time();
static uint32_t s_hashes_statistick = 0;

void DecreaseRecieveBuffer(size_t n)
{
  uint32_t rest = s_recieve_buffer_pos - n;
  if (rest > 0)
    memcpy(s_recieve_buffer, s_recieve_buffer + n, rest);
  s_recieve_buffer_pos = rest;
}

void loop()
{
  vTaskDelay(5 / portTICK_PERIOD_MS);
  int recieve_size = i2c_slave_read_buffer(I2C_PORT, s_recieve_buffer + s_recieve_buffer_pos, sizeof(s_recieve_buffer) - s_recieve_buffer_pos, 10 / portTICK_RATE_MS);
  if (recieve_size > 0)
  {
    s_recieve_buffer_pos += recieve_size;
    if (recieve_size > 2)
      Serial.printf("Recieve %d bytes, size=%d\n", recieve_size, s_recieve_buffer_pos);
    while (s_recieve_buffer_pos >= 2)
    {
      if (s_recieve_buffer[0] == I2C_CMD_REQUEST_RESULT)
      {
        uint8_t crc = CommandCrc8(s_recieve_buffer, 2);
        if (crc == s_recieve_buffer[1])
        {
          HostCommand_Request();
          DecreaseRecieveBuffer(2);
        } else
          DecreaseRecieveBuffer(1);
        continue;
      }

      if (s_recieve_buffer[0] == I2C_CMD_FEED)
      {
        if (s_recieve_buffer_pos < sizeof(JobI2cRequest))
          break;  //Need more data

        uint8_t crc = CommandCrc8(s_recieve_buffer, sizeof(JobI2cRequest));
        if (crc == s_recieve_buffer[1])
        {
          HostCommand_Feed((JobI2cRequest*)s_recieve_buffer);
          DecreaseRecieveBuffer(sizeof(JobI2cRequest));
        } else
        {
          Serial.println("CMD_FEED crc error");
          DecreaseRecieveBuffer(1);
        }
        continue;
      }

      if (s_recieve_buffer[0] != I2C_CMD_FEED && s_recieve_buffer[0] != I2C_CMD_REQUEST_RESULT)
      {
        size_t drop_size = s_recieve_buffer_pos;
        for (size_t n = 0; n < s_recieve_buffer_pos; ++n)
        {
          if (s_recieve_buffer[n] == I2C_CMD_FEED ||
              s_recieve_buffer[n] == I2C_CMD_REQUEST_RESULT)
          {
              drop_size = n;
              break;
          }
        }
        Serial.printf("Drop unknown data %d bytes\n", drop_size);
        DecreaseRecieveBuffer(drop_size);
        continue;
      }

      if (s_recieve_buffer_pos > sizeof(JobI2cRequest) * 2)
      {
        Serial.printf("Drop overflow data %d bytes\n", sizeof(JobI2cRequest));
        DecreaseRecieveBuffer(sizeof(JobI2cRequest));
      }
    }
  }

  //Work Task process
  std::list<std::shared_ptr<JobResult>> job_result_list;
  if (s_nonce_pool != 0xFFFFFFFF)
  {
    std::lock_guard<std::mutex> lock(s_job_mutex);
    job_result_list = s_job_result_list;
    s_job_result_list.clear();
    
    while (s_job_request_list_sw.size() < 4)
    {
      JobPush( s_job_request_list_sw, s_job_id, s_nonce_pool, NONCE_PER_JOB_SW, s_pool_difficulty, s_sha_buffer+64, s_diget_mid, s_bake);
      s_nonce_pool += NONCE_PER_JOB_SW;
    }
    
    #ifdef HARDWARE_SHA265
    while (s_job_request_list_hw.size() < 4)
    {
      JobPush( s_job_request_list_hw, s_job_id, s_nonce_pool, NONCE_PER_JOB_HW, s_pool_difficulty, s_sha_buffer+64, s_hw_midstate, s_bake);
      s_nonce_pool += NONCE_PER_JOB_HW;
    }
    #endif
  }

  while (!job_result_list.empty())
  {
    std::shared_ptr<JobResult> res = job_result_list.front();
    job_result_list.pop_front();

    s_hashes_statistick += res->nonce_count;
    s_slave_nonces += res->nonce_count;
    if (res->id != s_job_id || res->nonce == 0xFFFFFFFF)
      continue;

    if (res->difficulty > s_best_diff)
    {
      s_best_job_id = res->id;
      s_best_nonce = res->nonce;
      s_best_diff = res->difficulty;
      Serial.printf("Best hash: diff=%.4f, nonce=0x%X id=0x%X\n", s_best_diff, s_best_nonce, s_best_job_id);
    }
  }

  int64_t time = esp_timer_get_time();
  if (time > s_time_statistick_us + 5000000)
  {
	  double hash_rate = (double)s_hashes_statistick * 1000000.0 / double(time - s_time_statistick_us);
    Serial.printf("HashRate %.2fKH/s\n", hash_rate / 1000.0);
    s_hashes_statistick = 0;
    s_time_statistick_us = time;
  }
}

static const double truediffone = 26959535291011309493156476344723991336010898738574164086137773096960.0;
/* Converts a little endian 256 bit value to a double */
double le256todouble(const void *target)
{
	const uint64_t *data64;
	double dcut64;

	data64 = (const uint64_t *)((const uint8_t*)target + 24);
	dcut64 = *data64 * 6277101735386680763835789423207666416102355444464034512896.0;

	data64 = (const uint64_t *)((const uint8_t*)target + 16);
	dcut64 += *data64 * 340282366920938463463374607431768211456.0;

	data64 = (const uint64_t *)((const uint8_t*)target + 8);
	dcut64 += *data64 * 18446744073709551616.0;

	data64 = (const uint64_t *)(target);
	dcut64 += *data64;

	return dcut64;
}

double diff_from_target(void *target)
{
	double d64, dcut64;

	d64 = truediffone;
	dcut64 = le256todouble(target);
	if (unlikely(!dcut64))
		dcut64 = 1;
	return d64 / dcut64;
}


void minerWorkerSw(void * task_id)
{
  unsigned int miner_id = (uint32_t)task_id;
  Serial.printf("[MINER] %d Started minerWorkerSw Task!\n", miner_id);

  std::shared_ptr<JobRequest> job;
  std::shared_ptr<JobResult> result;
  uint8_t hash[32];
  uint32_t wdt_counter = 0;
  while (1)
  {
    {
      std::lock_guard<std::mutex> lock(s_job_mutex);
      if (result)
      {
        s_job_result_list.push_back(result);
        result.reset();
      }
      if (!s_job_request_list_sw.empty())
      {
        //Serial.printf("[MINER] %d got job queue size=%d\n", miner_id, s_job_request_list_sw.size());
        job = s_job_request_list_sw.front();
        s_job_request_list_sw.pop_front();
      } else
        job.reset();
    }
    if (job)
    {
      result = std::make_shared<JobResult>();
      result->difficulty = job->difficulty;
      result->nonce = 0xFFFFFFFF;
      result->id = job->id;
      result->nonce_count = job->nonce_count;
      for (uint32_t n = 0; n < job->nonce_count; ++n)
      {
        ((uint32_t*)(job->buffer_upper+12))[0] = job->nonce_start+n;
        nerd_sha256d_baked(job->midstate, job->buffer_upper, job->bake, hash);
        if(hash[31] == 0 && hash[30] == 0)
        {
          double diff_hash = diff_from_target(hash);
          if (diff_hash > result->difficulty)
          {
            result->difficulty = diff_hash;
            result->nonce = job->nonce_start+n;
            memcpy(result->hash, hash, 32);
          }
        }
      }
    } else
      vTaskDelay(2 / portTICK_PERIOD_MS);

    wdt_counter++;
    if (wdt_counter >= 8)
    {
      wdt_counter = 0;
      esp_task_wdt_reset();
    }
  }
}


#ifdef HARDWARE_SHA265

static inline void nerd_sha_ll_fill_text_block_sha256(const void *input_text)
{
    uint32_t *data_words = (uint32_t *)input_text;
    uint32_t *reg_addr_buf = (uint32_t *)(SHA_TEXT_BASE);

    REG_WRITE(&reg_addr_buf[0], data_words[0]);
    REG_WRITE(&reg_addr_buf[1], data_words[1]);
    REG_WRITE(&reg_addr_buf[2], data_words[2]);
    REG_WRITE(&reg_addr_buf[3], data_words[3]);
    REG_WRITE(&reg_addr_buf[4], data_words[4]);
    REG_WRITE(&reg_addr_buf[5], data_words[5]);
    REG_WRITE(&reg_addr_buf[6], data_words[6]);
    REG_WRITE(&reg_addr_buf[7], data_words[7]);
    REG_WRITE(&reg_addr_buf[8], data_words[8]);
    REG_WRITE(&reg_addr_buf[9], data_words[9]);
    REG_WRITE(&reg_addr_buf[10], data_words[10]);
    REG_WRITE(&reg_addr_buf[11], data_words[11]);
    REG_WRITE(&reg_addr_buf[12], data_words[12]);
    REG_WRITE(&reg_addr_buf[13], data_words[13]);
    REG_WRITE(&reg_addr_buf[14], data_words[14]);
    REG_WRITE(&reg_addr_buf[15], data_words[15]);
}

static inline void nerd_sha_hal_wait_idle()
{
    while (sha_ll_busy())
    {}
}

void minerWorkerHw(void * task_id)
{
  unsigned int miner_id = (uint32_t)task_id;
  Serial.printf("[MINER] %d Started minerWorkerHw Task!\n", miner_id);

  std::shared_ptr<JobRequest> job;
  std::shared_ptr<JobResult> result;
  uint8_t interResult[64];
  uint8_t hash[32];

  uint32_t wdt_counter = 0;

  memset(interResult, 0, sizeof(interResult));
  interResult[32] = 0x80;
  interResult[62] = 0x01;
  interResult[63] = 0x00;
  while (1)
  {
    {
      std::lock_guard<std::mutex> lock(s_job_mutex);
      if (result)
      {
        s_job_result_list.push_back(result);
        result.reset();
      }
      if (!s_job_request_list_hw.empty())
      {
        job = s_job_request_list_hw.front();
        s_job_request_list_hw.pop_front();
      } else
        job.reset();
    }
    if (job)
    {
      result = std::make_shared<JobResult>();
      result->id = job->id;
      result->nonce = 0xFFFFFFFF;
      result->nonce_count = job->nonce_count;
      result->difficulty = job->difficulty;

      uint8_t* sha_buffer = job->buffer_upper;
      esp_sha_acquire_hardware();
      for (uint32_t n = 0; n < job->nonce_count; ++n)
      {
        ((uint32_t*)(sha_buffer+12))[0] = job->nonce_start+n;
          
        sha_ll_write_digest(SHA2_256, job->midstate, 256 / 32);  //no need to unroll
        //sha_hal_wait_idle();
        nerd_sha_hal_wait_idle();
        //sha_ll_fill_text_block(header64, 64/4);
        nerd_sha_ll_fill_text_block_sha256(sha_buffer);
        sha_ll_continue_block(SHA2_256);
    
        sha_ll_load(SHA2_256);
        //sha_hal_wait_idle();
        nerd_sha_hal_wait_idle();
        sha_ll_read_digest(SHA2_256, interResult, 256 / 32);
    
        //sha_hal_wait_idle();
        nerd_sha_hal_wait_idle();
        //sha_ll_fill_text_block(interResult, 64/4);
        nerd_sha_ll_fill_text_block_sha256(interResult);
        sha_ll_start_block(SHA2_256);

        sha_ll_load(SHA2_256);
        //sha_hal_wait_idle();
        nerd_sha_hal_wait_idle();
        sha_ll_read_digest(SHA2_256, hash, 256 / 32);

        if(hash[31] == 0 && hash[30] == 0)
        {
          double diff_hash = diff_from_target(hash);
          if (diff_hash > result->difficulty)
          {
            result->difficulty = diff_hash;
            result->nonce = job->nonce_start+n;
            memcpy(result->hash, hash, sizeof(hash));
          }
        }
      }
      esp_sha_release_hardware();
    } else
      vTaskDelay(2 / portTICK_PERIOD_MS);

    wdt_counter++;
    if (wdt_counter >= 8)
    {
      wdt_counter = 0;
      esp_task_wdt_reset();
    }
  }
}
#endif