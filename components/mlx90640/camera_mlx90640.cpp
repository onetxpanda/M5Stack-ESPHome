#include "camera_mlx90640.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace mlx90640 {

static const char *const TAG = "MLX90640";
static constexpr int TA_SHIFT = 8;  // Default shift for MLX90640 in open air
static constexpr float MIN_CAM_V = -40.0f;   // Spec in datasheet
static constexpr float MAX_CAM_V = 300.0f;   // Spec in datasheet

static void iron_colormap(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b);

/* ---------------- MLX90640CameraImageReader ---------------- */
void MLX90640CameraImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::static_pointer_cast<MLX90640CameraImage>(image);
  this->offset_ = 0;
}

size_t MLX90640CameraImageReader::available() const {
  if (!this->image_)
    return 0;
  return this->image_->get_data_length() - this->offset_;
}

uint8_t *MLX90640CameraImageReader::peek_data_buffer() {
  if (!this->image_)
    return nullptr;
  return this->image_->get_data_buffer() + this->offset_;
}

/* ---------------- MLX90640 ---------------- */
void MLX90640::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MLX90640...");
  ESP_LOGCONFIG(TAG, "Address 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "Color MinTemp %d ", static_cast<int>(this->mintemp_));
  ESP_LOGCONFIG(TAG, "Color MaxTemp %d ", static_cast<int>(this->maxtemp_));

  this->encoder_output_.set_buffer_size(this->encoder_buffer_size_);
#ifdef USE_ESP32_CAMERA_JPEG_ENCODER
  this->encoder_ = std::make_unique<camera_encoder::ESP32CameraJPEGEncoder>(this->encoder_quality_,
                                                                            &this->encoder_output_);
  static_cast<camera_encoder::ESP32CameraJPEGEncoder *>(this->encoder_.get())
      ->set_buffer_expand_size(this->encoder_buffer_expand_size_);
#endif

  this->scaled_spec_ = {static_cast<uint16_t>(COLS * this->scale_),
                        static_cast<uint16_t>(ROWS * this->scale_),
                        camera::PIXEL_FORMAT_BGR888};
  this->scaled_buffer_ = std::make_unique<camera::BufferImpl>(
      static_cast<size_t>(this->scaled_spec_.width) * this->scaled_spec_.height * 3);
  const size_t rgb565_size = static_cast<size_t>(this->scaled_spec_.width) * this->scaled_spec_.height * 2;
  this->rgb565_buffer_.reset(
      static_cast<uint8_t *>(heap_caps_malloc(rgb565_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
  if (!this->rgb565_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate %u B RGB565 buffer in PSRAM", static_cast<unsigned>(rgb565_size));
    this->mark_failed();
    return;
  }

  MLX90640_I2CInit(this);
  int status = 0;
  uint16_t ee_data[832];

  status = MLX90640_DumpEE(this->address_, ee_data);
  if (status != 0) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->mark_failed();
    return;
  }

  status = MLX90640_ExtractParameters(ee_data, &this->mlx90640_params_);
  if (status != 0) {
    switch (status) {
      case MLX90640_I2C_NACK_ERROR:
        ESP_LOGE(TAG, "Bad EE data");
        break;
      case MLX90640_I2C_WRITE_ERROR:
        ESP_LOGE(TAG, "Bad split data");
        break;
      case MLX90640_BROKEN_PIXELS_NUM_ERROR:
        ESP_LOGE(TAG, "Too many broken pixels");
        break;
      case MLX90640_OUTLIER_PIXELS_NUM_ERROR:
        ESP_LOGE(TAG, "Too many outlier pixels");
        break;
      case MLX90640_BAD_PIXELS_NUM_ERROR:
        ESP_LOGE(TAG, "Too many bad pixels");
        break;
      case MLX90640_ADJACENT_BAD_PIXELS_ERROR:
        ESP_LOGE(TAG, "Adjacent bad pixels");
        break;
      case MLX90640_EEPROM_DATA_ERROR:
        ESP_LOGE(TAG, "EEPROM data error");
        break;
      case MLX90640_FRAME_DATA_ERROR:
        ESP_LOGE(TAG, "Frame data error");
        break;
      case MLX90640_MEAS_TRIGGER_ERROR:
        ESP_LOGE(TAG, "Measurement trigger error");
        break;
      default:
        ESP_LOGE(TAG, "Unknown error");
        break;
    }
    this->mark_failed();
    return;
  }

  uint8_t refresh_rate = (this->refresh_rate_ >= 0) ? static_cast<uint8_t>(this->refresh_rate_) : 0x05;
  MLX90640_SetRefreshRate(this->address_, refresh_rate);
  ESP_LOGI(TAG, "Refresh rate register set to 0x%02X", refresh_rate);

  this->interleaved_mode_ = MLX90640_GetCurMode(this->address_);

  for (int v = 0; v < 256; v++)
    iron_colormap(static_cast<uint8_t>(v), this->colormap_lut_[v].r, this->colormap_lut_[v].g, this->colormap_lut_[v].b);
}

void MLX90640::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90640:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    return;
  }
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Color MinTemp: %d", static_cast<int>(this->mintemp_));
  ESP_LOGCONFIG(TAG, "  Color MaxTemp: %d", static_cast<int>(this->maxtemp_));
  ESP_LOGCONFIG(TAG, "  Filter level: %.2f", this->filter_level_);
  ESP_LOGCONFIG(TAG, "  Refresh rate: 0x%02X", this->refresh_rate_ >= 0 ? this->refresh_rate_ : 0x05);
  ESP_LOGCONFIG(TAG, "  JPEG quality: %u", this->encoder_quality_);
  ESP_LOGCONFIG(TAG, "  JPEG scale: %ux (%ux%u)", this->scale_, COLS * this->scale_, ROWS * this->scale_);
  ESP_LOGCONFIG(TAG, "  JPEG buffer size: %u", static_cast<unsigned>(this->encoder_buffer_size_));
  ESP_LOGCONFIG(TAG, "  JPEG buffer expand size: %u", static_cast<unsigned>(this->encoder_buffer_expand_size_));
  LOG_SENSOR("  ", "Min temperature", this->min_temperature_sensor_);
  LOG_SENSOR("  ", "Max temperature", this->max_temperature_sensor_);
  LOG_SENSOR("  ", "Mean temperature", this->mean_temperature_sensor_);
  LOG_SENSOR("  ", "Median temperature", this->median_temperature_sensor_);
}

void MLX90640::loop() {
  if (this->current_image_ && this->current_image_.use_count() == 1)
    this->current_image_.reset();

  switch (this->phase_) {
    case CapturePhase::IDLE:
      if (!this->is_data_ready_() || this->current_image_)
        return;
      if (this->stream_requesters_)
        this->single_requesters_ |= this->stream_requesters_;
      // Phase 1: I2C read + FP math — yields here, colours next tick.
      if (!this->fetch_and_calc_()) {
        this->single_requesters_ = 0;
        return;
      }
      this->phase_ = CapturePhase::COLOR;
      return;

    case CapturePhase::COLOR:
      // Phase 2: filter + colormap + upscale + on_frame callbacks.
      if (!this->colormap_and_upscale_()) {
        this->single_requesters_ = 0;
        this->phase_ = CapturePhase::IDLE;
        return;
      }
      this->publish_sensors_();
      if (this->has_requested_image_()) {
        this->phase_ = CapturePhase::ENCODE;
      } else {
        this->single_requesters_ = 0;
        this->phase_ = CapturePhase::IDLE;
      }
      return;

    case CapturePhase::ENCODE:
      // Phase 3: JPEG encode — only reached when a requester is waiting.
      this->encode_frame_(this->single_requesters_);
      this->single_requesters_ = 0;
      this->phase_ = CapturePhase::IDLE;
      return;
  }
}

void MLX90640::start_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_) {
    listener->on_stream_start();
  }
  this->stream_requesters_ |= (1U << requester);
}

void MLX90640::stop_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_) {
    listener->on_stream_stop();
  }
  this->stream_requesters_ &= ~(1U << requester);
}

void MLX90640::filter_outlier_pixel_(float *pixels, int pixel_size, float level) {
  for (int i = 1; i < pixel_size - 1; i++) {
    if (std::fabs(pixels[i] - pixels[i - 1]) >= level && std::fabs((pixels[i] - pixels[i + 1])) >= level) {
      pixels[i] = (pixels[i - 1] + pixels[i + 1]) / 2.0f;
    }
  }
  if (std::fabs(pixels[0] - pixels[1]) >= level && std::fabs(pixels[0] - pixels[2]) >= level) {
    pixels[0] = (pixels[1] + pixels[2]) / 2.0f;
  }
  if (std::fabs(pixels[pixel_size - 1] - pixels[pixel_size - 2]) >= level &&
      std::fabs(pixels[pixel_size - 1] - pixels[pixel_size - 3]) >= level) {
    pixels[pixel_size - 1] = (pixels[pixel_size - 2] + pixels[pixel_size - 3]) / 2.0f;
  }
}

// Iron colormap: black → purple → red → orange → yellow → white
static void iron_colormap(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
  struct Point { uint8_t pos, r, g, b; };
  static constexpr Point kPoints[] = {
    {  0,   0,   0,   0},
    { 32,  74,   0,  85},
    { 64, 148,   0, 170},
    { 96, 196,   0,  85},
    {128, 220,   0,   0},
    {160, 255, 110,   0},
    {192, 255, 210,   0},
    {224, 253, 252, 124},
    {255, 255, 255, 255},
  };
  static constexpr size_t N = sizeof(kPoints) / sizeof(kPoints[0]);
  for (size_t i = 0; i < N - 1; i++) {
    if (v <= kPoints[i + 1].pos) {
      int16_t dr = (int16_t) kPoints[i + 1].r - (int16_t) kPoints[i].r;
      int16_t dg = (int16_t) kPoints[i + 1].g - (int16_t) kPoints[i].g;
      int16_t db = (int16_t) kPoints[i + 1].b - (int16_t) kPoints[i].b;
      uint16_t frac = v - kPoints[i].pos;
      uint16_t dspan = kPoints[i + 1].pos - kPoints[i].pos;
      r = (uint8_t) ((int16_t) kPoints[i].r + dr * frac / dspan);
      g = (uint8_t) ((int16_t) kPoints[i].g + dg * frac / dspan);
      b = (uint8_t) ((int16_t) kPoints[i].b + db * frac / dspan);
      return;
    }
  }
  r = 255; g = 255; b = 255;
}

bool MLX90640::fetch_and_calc_() {
  int status = MLX90640_GetFrameData(this->address_, this->frame_buffer_.data());
  if (status < 0) {
    ESP_LOGE(TAG, "GetFrame Error: %d", status);
    this->data_valid_ = false;
    return false;
  }

  float vdd = MLX90640_GetVdd(this->frame_buffer_.data(), &this->mlx90640_params_);
  (void) vdd;
  float ta = MLX90640_GetTa(this->frame_buffer_.data(), &this->mlx90640_params_);
  float tr = ta - TA_SHIFT;
  MLX90640_CalculateTo(this->frame_buffer_.data(), &this->mlx90640_params_, 0.95f, tr, this->pixels_.data());
  MLX90640_BadPixelsCorrection(this->mlx90640_params_.brokenPixels, this->pixels_.data(), this->interleaved_mode_,
                               &this->mlx90640_params_);

  // Spatially interpolate pixels not captured in this sub-frame from their
  // current-subframe neighbours. In chess mode every 4-connected neighbour of
  // a missing pixel belongs to the current sub-frame, so the result is a fully
  // temporally-coherent frame (all data from the same capture instant) with no
  // checker pattern on movement. In interleaved mode the two vertical neighbours
  // are used. Either way the missing pixels are never stale.
  const int subpage = MLX90640_GetSubPageNumber(this->frame_buffer_.data());
  if (this->interleaved_mode_ == 1) {
    // Chess mode: pixel (row,col) is in subpage (row+col)&1.
    // All 4-connected neighbours of a missing pixel are in the current subpage.
    for (int row = 0; row < ROWS; row++) {
      for (int col = 0; col < COLS; col++) {
        if (((row + col) & 1) == subpage)
          continue;
        float sum = 0.0f;
        int cnt = 0;
        if (row > 0)      { sum += this->pixels_[(row - 1) * COLS + col]; cnt++; }
        if (row < ROWS-1) { sum += this->pixels_[(row + 1) * COLS + col]; cnt++; }
        if (col > 0)      { sum += this->pixels_[row * COLS + col - 1];   cnt++; }
        if (col < COLS-1) { sum += this->pixels_[row * COLS + col + 1];   cnt++; }
        if (cnt)
          this->pixels_[row * COLS + col] = sum / cnt;
      }
    }
  } else {
    // Interleaved mode: pixel (row,col) is in subpage row&1.
    // Only the vertical neighbours are in the current subpage.
    for (int row = 0; row < ROWS; row++) {
      if ((row & 1) == subpage)
        continue;
      for (int col = 0; col < COLS; col++) {
        float sum = 0.0f;
        int cnt = 0;
        if (row > 0)      { sum += this->pixels_[(row - 1) * COLS + col]; cnt++; }
        if (row < ROWS-1) { sum += this->pixels_[(row + 1) * COLS + col]; cnt++; }
        if (cnt)
          this->pixels_[row * COLS + col] = sum / cnt;
      }
    }
  }
  return true;
}

bool MLX90640::colormap_and_upscale_() {
  this->filter_outlier_pixel_(this->pixels_.data(), PIXEL_COUNT, this->filter_level_);
  this->median_temp_ = (this->pixels_[165] + this->pixels_[180] + this->pixels_[176] + this->pixels_[192]) / 4.0f;
  this->max_v_ = this->mintemp_;
  this->min_v_ = this->maxtemp_;
  float total = 0.0f;
  for (float temperature : this->pixels_) {
    if (temperature > this->max_v_)
      this->max_v_ = temperature;
    if (temperature < this->min_v_)
      this->min_v_ = temperature;
    total += temperature;
  }
  this->mean_temp_ = total / PIXEL_COUNT;

  if (this->max_v_ > MAX_CAM_V || this->max_v_ < MIN_CAM_V) {
    ESP_LOGE(TAG, "MLX READING VALUE ERRORS");
    this->data_valid_ = false;
    return false;
  }
  this->data_valid_ = true;

  const float span = std::max(this->maxtemp_ - this->mintemp_, 1.0f);
  uint8_t *pixel_data = this->pixel_buffer_.get_data_buffer();
  for (size_t idx = 0; idx < PIXEL_COUNT; idx++) {
    float clamped = std::clamp(this->pixels_[idx], this->mintemp_, this->maxtemp_);
    uint8_t v = static_cast<uint8_t>(std::roundf((clamped - this->mintemp_) / span * 255.0f));
    const IronColor &c = this->colormap_lut_[v];
    pixel_data[idx * 3 + 0] = c.b;  // PIXEL_FORMAT_BGR888: B, G, R
    pixel_data[idx * 3 + 1] = c.g;
    pixel_data[idx * 3 + 2] = c.r;
  }

  // Upscale pixel_buffer_ into scaled_buffer_ (BGR888 for JPEG) and
  // rgb565_buffer_ (big-endian RGB565 for direct display use) simultaneously.
  uint8_t *dst = this->scaled_buffer_->get_data_buffer();
  uint8_t *dst565 = this->rgb565_buffer_.get();
  const uint8_t *src = this->pixel_buffer_.get_data_buffer();
  const uint16_t scaled_w = this->scaled_spec_.width;
  const uint16_t scaled_h = this->scaled_spec_.height;
  for (uint16_t oy = 0; oy < scaled_h; oy++) {
    const uint8_t *src_row = src + (oy / this->scale_) * COLS * 3;
    for (uint16_t ox = 0; ox < scaled_w; ox++) {
      const uint8_t *p = src_row + (ox / this->scale_) * 3;
      *dst++ = p[0];
      *dst++ = p[1];
      *dst++ = p[2];
      // p is [B, G, R] → RGB565 big-endian
      uint16_t px = ((uint16_t)(p[2] & 0xF8) << 8) | ((uint16_t)(p[1] & 0xFC) << 3) | (p[0] >> 3);
      *dst565++ = px >> 8;
      *dst565++ = px & 0xFF;
    }
  }

  this->on_frame_callbacks_.call();
  return true;
}

bool MLX90640::encode_frame_(uint8_t requesters) {
#ifdef USE_ESP32_CAMERA_JPEG_ENCODER
  if (this->encoder_ == nullptr) {
    ESP_LOGE(TAG, "JPEG encoder not configured");
    return false;
  }

  camera::EncoderError error;
  do {
    error = this->encoder_->encode_pixels(&this->scaled_spec_, this->scaled_buffer_.get());
    if (error == camera::ENCODER_ERROR_SKIP_FRAME)
      return false;
    if (error == camera::ENCODER_ERROR_CONFIGURATION) {
      this->mark_failed(LOG_STR("Failed to encode frame."));
      return false;
    }
  } while (error == camera::ENCODER_ERROR_RETRY_FRAME);

  this->current_image_ = std::make_shared<MLX90640CameraImage>(&this->encoder_output_, requesters);
  for (auto *listener : this->listeners_) {
    listener->on_camera_image(this->current_image_);
  }
  return true;
#else
  (void) requesters;
  ESP_LOGE(TAG, "ESP32 camera JPEG encoder is not enabled");
  return false;
#endif
}

void MLX90640::publish_sensors_() {
  if (!this->data_valid_)
    return;

  if (this->min_temperature_sensor_ != nullptr)
    this->min_temperature_sensor_->publish_state(this->min_v_);
  if (this->max_temperature_sensor_ != nullptr)
    this->max_temperature_sensor_->publish_state(this->max_v_);
  if (this->mean_temperature_sensor_ != nullptr)
    this->mean_temperature_sensor_->publish_state(this->mean_temp_);
  if (this->median_temperature_sensor_ != nullptr)
    this->median_temperature_sensor_->publish_state(this->median_temp_);
}

bool MLX90640::is_data_ready_() {
  uint16_t status;
  if (MLX90640_I2CRead(this->address_, MLX90640_STATUS_REG, 1, &status) != 0)
    return false;
  return MLX90640_GET_DATA_READY(status) != 0;
}

esphome::Color MLX90640::get_pixel_color(uint8_t col, uint8_t row) {
  if (col >= COLS || row >= ROWS || !this->data_valid_)
    return esphome::Color(0, 0, 0);
  size_t idx = row * COLS + col;
  const uint8_t *p = this->pixel_buffer_.get_data_buffer() + idx * 3;
  // pixel_buffer_ is BGR888: byte order B, G, R
  return esphome::Color(p[2], p[1], p[0]);
}

}  // namespace mlx90640
}  // namespace esphome
