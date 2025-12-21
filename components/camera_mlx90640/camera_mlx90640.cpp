#include "camera_mlx90640.h"
#include "esphome/core/log.h"
#include <cmath>


uint8_t MLX90640_address = 0x33;  // Default 7-bit unshifted address of the
                                     // MLX90640.  MLX90640的默认7位未移位地址
#define TA_SHIFT \
    8  // Default shift for MLX90640 in open air.  MLX90640在户外的默认移位

#define COLS   32
#define ROWS   24
float pixels[COLS * ROWS];
uint8_t speed_setting = 2;  // High is 1 , Low is 2

static const char * TAG = "MLX90640" ;
paramsMLX90640 mlx90640;
bool dataValid = false ;
float medianTemp ;
float meanTemp ;



// low range of the sensor (this will be blue on the screen).
// 传感器的低量程(屏幕上显示为蓝色)
int MINTEMP   = 24;   // For color mapping.  颜色映射
float min_v     = 24;   // Value of current min temp.  当前最小温度的值
int min_cam_v = -40;  // Spec in datasheet.  规范的数据表

// high range of the sensor (this will be red on the screen).
// 传感器的高量程(屏幕上显示为红色)
int MAXTEMP      = 35;   // For color mapping.  颜色映射
float max_v        = 35;   // Value of current max temp.  当前最大温度值
int max_cam_v    = 300;  // Spec in datasheet.  规范的数据表

namespace esphome{
    namespace mlx90640_app{
        void MLX90640::setup(){
            // Initialize the the sensor data
                ESP_LOGCONFIG(TAG, "Setting up MLX90640...");
                ESP_LOGCONFIG(TAG, "Address 0x%02X", this->address_);
                MLX90640_address = this->address_ ;
                MINTEMP = this->mintemp_ ;
                MAXTEMP = this->maxtemp_ ;

                ESP_LOGCONFIG(TAG, "Color MinTemp %d ", MINTEMP);
                ESP_LOGCONFIG(TAG, "Color MaxTemp %d ", MAXTEMP);
                MLX90640_I2CInit(this);
                int status;
                uint16_t eeMLX90640[832];  // 32 * 24 = 768
                if(MLX90640_isConnected(MLX90640_address)){
                status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
                if (status != 0) 
                ESP_LOGE(TAG,"Failed to load system parameters");

                status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
                if (status != 0)  ESP_LOGE(TAG,"Parameter extraction failed");
                
                int SetRefreshRate;
                // Setting MLX90640 device at slave address 0x33 to work with 16Hz refresh
                // rate: 设置从地址0x33的MLX90640设备以16Hz刷新率工作:
                // 0x00 – 0.5Hz
                // 0x01 – 1Hz
                // 0x02 – 2Hz
                // 0x03 – 4Hz
                // 0x04 – 8Hz // OK
                // 0x05 – 16Hz // OK
                // 0x06 – 32Hz // Fail
                // 0x07 – 64Hz
                if(this->refresh_rate_){
                  SetRefreshRate = MLX90640_SetRefreshRate(MLX90640_address, this->refresh_rate_);
                  if(this->refresh_rate_==0x05){
                      ESP_LOGI(TAG, "Refresh rate set to 16Hz ");

                  }else if(this->refresh_rate_==0x04){
                    ESP_LOGI(TAG, "Refresh rate set to 8Hz ");
                  }else{
                    ESP_LOGI(TAG, "Refresh rate Not Valid ");
                    SetRefreshRate = MLX90640_SetRefreshRate(MLX90640_address, 0x05);
                  }
                  
                }else{
                  SetRefreshRate = MLX90640_SetRefreshRate(MLX90640_address, 0x05);
                  ESP_LOGI(TAG, "Refresh rate set to 16Hz ");
                }
                (void) SetRefreshRate;
                
                // Once params are extracted, we can release eeMLX90640 array.
                // 一旦提取了参数，我们就可以释放eeMLX90640数组
                }else{
                    ESP_LOGE(TAG, "The sensor is not connected");
                }
        }

        void MLX90640::dump_config() {
            ESP_LOGCONFIG(TAG, "MLX90640:");
            LOG_I2C_DEVICE(this);
            if (this->is_failed())
            {
                ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
            }
            ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
            ESP_LOGCONFIG(TAG, "  Color MinTemp: %d", static_cast<int>(this->mintemp_));
            ESP_LOGCONFIG(TAG, "  Color MaxTemp: %d", static_cast<int>(this->maxtemp_));
            ESP_LOGCONFIG(TAG, "  Filter level: %.2f", this->filter_level_);
            if (this->refresh_rate_ > 0) {
                ESP_LOGCONFIG(TAG, "  Refresh rate: 0x%02X", this->refresh_rate_);
            }
            LOG_UPDATE_INTERVAL(this);
            LOG_SENSOR("  ", "Min temperature", this->min_temperature_sensor_);
            LOG_SENSOR("  ", "Max temperature", this->max_temperature_sensor_);
            LOG_SENSOR("  ", "Mean temperature", this->mean_temperature_sensor_);
            LOG_SENSOR("  ", "Median temperature", this->median_temperature_sensor_);
        }

        void MLX90640::filter_outlier_pixel(float *pixels_ , int pixel_size , float level){
            for(int i=1 ; i<pixel_size -1 ; i++){
                if(std::fabs(pixels_[i]-pixels_[i-1])>= level && std::fabs((pixels_[i]-pixels_[i+1]))>= level ){
                    pixels_[i] = (pixels_[i-1] + pixels_[i+1])/2.0 ;
                }
            }
            // Check the zero index pixel
            if(std::fabs(pixels_[0]-pixels_[1])>=level && std::fabs(pixels_[0]-pixels_[2])>=level){
                pixels_[0] = (pixels_[1] +pixels_[2])/2.0 ;
            }
            // Check the zero index pixel
            if(std::fabs(pixels_[pixel_size-1]-pixels_[pixel_size-2])>=level &&
               std::fabs(pixels_[pixel_size-1]-pixels_[pixel_size-3])>=level){
                pixels_[pixel_size-1] = (pixels_[pixel_size-2] +pixels_[pixel_size-3])/2.0 ;
            }
        }

        void MLX90640::update()
        {
           //this->pixel_data_->publish_state(payload);
           if(dataValid)
           {
                this->min_temperature_sensor_->publish_state(min_v);
                this->max_temperature_sensor_->publish_state(max_v);
                this->mean_temperature_sensor_->publish_state(meanTemp);
                this->median_temperature_sensor_->publish_state(medianTemp);
           }
           
           if(MLX90640_isConnected(MLX90640_address)){
                   this->mlx_update();
           }else{
            ESP_LOGE(TAG, "The sensor is not connected");
            dataValid = false;
           }

        }


      void MLX90640::mlx_update(){
            for (uint8_t x = 0; x < speed_setting; x++)  // x < 2 Read both subpages
            {
                uint16_t mlx90640Frame[834];
                int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
                if (status < 0) {
                ESP_LOGE(TAG,"GetFrame Error: %d",status);
                }

                float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
                (void) vdd;
                float Ta  = MLX90640_GetTa(mlx90640Frame, &mlx90640);
                float tr = Ta - TA_SHIFT;  // Reflected temperature based on the sensor ambient
                                    // temperature.  根据传感器环境温度反射温度
                float emissivity = 0.95;
               MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, pixels);  // save pixels temp to array (pixels).
                                            // 保存像素temp到数组(像素)
                int mode_ = MLX90640_GetCurMode(MLX90640_address);
                // amendment.  修正案
                MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, pixels, mode_, &mlx90640);
            }

                filter_outlier_pixel(pixels,sizeof(pixels) / sizeof(pixels[0]), this->filter_level_ );
                medianTemp = (pixels[165]+pixels[180]+pixels[176]+pixels[192]) / 4.0;
                max_v      = MINTEMP;
                min_v      = MAXTEMP;
                // while(1);
                float total =0 ;
                for (int itemp = 0; itemp < sizeof(pixels) / sizeof(pixels[0]); itemp++) {
                    if (pixels[itemp] > max_v) {
                        max_v = pixels[itemp];
                    }
                    if (pixels[itemp] < min_v) {
                        min_v = pixels[itemp];
                    }
                    total += pixels[itemp] ;
                }
                meanTemp = total/((sizeof(pixels) / sizeof(pixels[0])));

                if (max_v > max_cam_v | max_v < min_cam_v) {
                    ESP_LOGE(TAG, "MLX READING VALUE ERRORS");
                    dataValid = false ;
                } else {
                    ESP_LOGI(TAG, "Min temperature : %.2f C ",min_v);
                    ESP_LOGI(TAG, "Max temperature : %.2f C ",max_v);
                    ESP_LOGI(TAG, "Mean temperature : %.2f C ",meanTemp);
                    ESP_LOGI(TAG, "Median temperature : %.2f C ",medianTemp);
                    dataValid = true ;
                }
      }
        
        

    }
}
