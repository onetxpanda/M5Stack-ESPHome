/**
   @copyright (C) 2017 Melexis N.V.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

*/
#include "MLX90640_I2C_Driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "MLX90640_I2C";
static i2c_port_t i2c_port = I2C_NUM_0;
static int i2c_sda_pin = GPIO_NUM_NC;
static int i2c_scl_pin = GPIO_NUM_NC;
static uint32_t i2c_frequency = 400000;

void MLX90640_I2CInit(i2c_port_t port, int sda, int scl, uint32_t frequency) {
  i2c_port = port;
  i2c_sda_pin = sda;
  i2c_scl_pin = scl;
  i2c_frequency = frequency;

  i2c_config_t conf{};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = static_cast<gpio_num_t>(sda);
  conf.scl_io_num = static_cast<gpio_num_t>(scl);
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = frequency;

  esp_err_t err = i2c_param_config(i2c_port, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(err));
    return;
  }

  err = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
  }
}

//Read a number of words from startAddress. Store into Data array.
//Returns 0 if successful, -1 if error
int MLX90640_I2CRead(uint8_t _deviceAddress, unsigned int startAddress, unsigned int nWordsRead,
                     uint16_t *data) {
  uint16_t bytesRemaining = nWordsRead * 2;
  uint16_t dataSpot = 0;

  while (bytesRemaining > 0) {
    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > I2C_BUFFER_LENGTH) {
      numberOfBytesToRead = I2C_BUFFER_LENGTH;
    }

    uint8_t reg[2] = {static_cast<uint8_t>(startAddress >> 8),
                      static_cast<uint8_t>(startAddress & 0xFF)};
    uint8_t buffer[I2C_BUFFER_LENGTH];
    esp_err_t err = i2c_master_write_read_device(i2c_port, _deviceAddress, reg, sizeof(reg), buffer,
                                                 numberOfBytesToRead, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
      return 0;
    }

    for (uint16_t x = 0; x < numberOfBytesToRead / 2; x++) {
      data[dataSpot] = static_cast<uint16_t>(buffer[x * 2]) << 8;
      data[dataSpot] |= buffer[x * 2 + 1];
      dataSpot++;
    }

    bytesRemaining -= numberOfBytesToRead;
    startAddress += numberOfBytesToRead / 2;
  }

  return 0;
}

//Set I2C Freq, in kHz
//MLX90640_I2CFreqSet(1000) sets frequency to 1MHz
void MLX90640_I2CFreqSet(int freq) {
  i2c_frequency = static_cast<uint32_t>(freq) * 1000U;
  i2c_config_t conf{};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = static_cast<gpio_num_t>(i2c_sda_pin);
  conf.scl_io_num = static_cast<gpio_num_t>(i2c_scl_pin);
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = i2c_frequency;
  i2c_param_config(i2c_port, &conf);
}

//Write two bytes to a two byte address
int MLX90640_I2CWrite(uint8_t _deviceAddress, unsigned int writeAddress, uint16_t data) {
  uint8_t buffer[4] = {static_cast<uint8_t>(writeAddress >> 8),
                       static_cast<uint8_t>(writeAddress & 0xFF),
                       static_cast<uint8_t>(data >> 8),
                       static_cast<uint8_t>(data & 0xFF)};
  esp_err_t err =
      i2c_master_write_to_device(i2c_port, _deviceAddress, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
    return -1;
  }

  uint16_t dataCheck;
  MLX90640_I2CRead(_deviceAddress, writeAddress, 1, &dataCheck);
  if (dataCheck != data) {
    return -2;
  }

  return 0;
}

bool MLX90640_isConnected(uint8_t addr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return err == ESP_OK;
}
