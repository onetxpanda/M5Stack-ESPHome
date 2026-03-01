#pragma once

#include <array>
#include <memory>
#include <vector>

#include "esphome/components/camera/buffer_impl.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/camera_encoder/encoder_buffer_impl.h"
#include "esphome/components/camera_encoder/esp32_camera_jpeg_encoder.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/color.h"
#include "esphome/core/component.h"
#ifdef USE_DISPLAY
#include "esphome/components/display/display_color_utils.h"
#endif
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

namespace esphome {
namespace mlx90640 {

static const uint8_t MLX90640_ADDRESS_DEFAULT = 0x33;

class MLX90640CameraImage : public camera::CameraImage {
 public:
  MLX90640CameraImage(camera_encoder::EncoderBufferImpl *buffer, uint8_t requesters)
      : buffer_(buffer), requesters_(requesters) {}

  uint8_t *get_data_buffer() override { return this->buffer_->get_data(); }
  size_t get_data_length() override { return this->buffer_->get_size(); }
  bool was_requested_by(camera::CameraRequester requester) const override {
    return (this->requesters_ & (1U << requester)) != 0;
  }

 protected:
  camera_encoder::EncoderBufferImpl *buffer_;
  uint8_t requesters_;
};

class MLX90640CameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override { this->offset_ += consumed; }
  void return_image() override { this->image_.reset(); }

 protected:
  std::shared_ptr<MLX90640CameraImage> image_;
  size_t offset_{0};
};

class MLX90640FrameTrigger : public Trigger<> {};

class MLX90640 : public i2c::I2CDevice, public camera::Camera {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void dump_config() override;
  void loop() override;

  void set_min_temperature_sensor(sensor::Sensor *ts) { this->min_temperature_sensor_ = ts; }
  void set_max_temperature_sensor(sensor::Sensor *ts) { this->max_temperature_sensor_ = ts; }
  void set_mean_temperature_sensor(sensor::Sensor *ts) { this->mean_temperature_sensor_ = ts; }
  void set_median_temperature_sensor(sensor::Sensor *ts) { this->median_temperature_sensor_ = ts; }
  void set_mintemp(float min) { this->mintemp_ = min; }
  void set_maxtemp(float max) { this->maxtemp_ = max; }
  void set_refresh_rate(int refresh) { this->refresh_rate_ = refresh; }
  void set_filter_level(float level) { this->filter_level_ = level; }
  void set_encoder_quality(uint8_t quality) { this->encoder_quality_ = quality; }
  void set_encoder_buffer_size(size_t size) { this->encoder_buffer_size_ = size; }
  void set_encoder_buffer_expand_size(size_t size) { this->encoder_buffer_expand_size_ = size; }
  void set_jpeg_scale(uint8_t scale) { this->scale_ = scale; }
  void set_iron_palette(bool v) { this->iron_palette_ = v; }

  // Camera interface
  void add_listener(camera::CameraListener *listener) override { this->listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override { return new MLX90640CameraImageReader; }
  void request_image(camera::CameraRequester requester) override { this->single_requesters_ |= (1U << requester); }
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;

  // Display accessor API
  static constexpr uint8_t COLS = 32;
  static constexpr uint8_t ROWS = 24;
  bool is_data_valid() const { return this->data_valid_; }
  bool is_iron_palette() const { return this->iron_palette_; }
  /// Returns the colour for pixel (col, row). Safe to call from a display lambda.
  /// In iron palette mode returns the mapped colour; in grayscale mode returns a gray Color.
  esphome::Color get_pixel_color(uint8_t col, uint8_t row);
  uint16_t get_upscaled_width() const { return this->scaled_spec_.width; }
  uint16_t get_upscaled_height() const { return this->scaled_spec_.height; }
  /// camera::PixelFormat of the display buffer (for camera pipeline use, not draw_pixels_at).
  camera::PixelFormat get_display_pixel_format() const {
    return this->iron_palette_ ? camera::PIXEL_FORMAT_RGB565 : camera::PIXEL_FORMAT_GRAYSCALE;
  }
#ifdef USE_DISPLAY
  /// Color order for draw_pixels_at(). Always COLOR_ORDER_RGB for both modes.
  display::ColorOrder get_display_color_order() const { return display::COLOR_ORDER_RGB; }
  /// Color bitness for draw_pixels_at(). COLOR_BITNESS_565 (iron palette) or COLOR_BITNESS_332 (grayscale).
  display::ColorBitness get_display_color_bitness() const {
    return this->iron_palette_ ? display::COLOR_BITNESS_565 : display::COLOR_BITNESS_332;
  }
#endif
  /// Whether the display buffer is stored big-endian. Always true for iron palette (RGB565),
  /// always false for grayscale (Y8). Pass directly as the big_endian argument of draw_pixels_at().
  bool get_display_big_endian() const { return this->iron_palette_; }
  /// Upscaled pixel buffer ready for draw_pixels_at(). Valid after the first frame.
  const uint8_t *get_display_buffer() const {
    return this->scaled_buffer_ ? this->scaled_buffer_->get_data_buffer() : nullptr;
  }
  void register_on_frame_trigger(MLX90640FrameTrigger *trigger) {
    this->on_frame_callbacks_.add([trigger]() { trigger->trigger(); });
  }

 protected:
  static constexpr size_t PIXEL_COUNT = COLS * ROWS;

  void filter_outlier_pixel_(float *pixels, int size, float level);
  bool is_data_ready_();
  bool fetch_and_calc_();       // Phase 1: I2C read + MLX90640_CalculateTo + interpolation
  bool colormap_and_upscale_(); // Phase 2: filter + colormap + upscale + on_frame callbacks
  bool encode_frame_(uint8_t requesters);
  void publish_sensors_();
  bool has_requested_image_() const { return this->single_requesters_ || this->stream_requesters_; }

  enum class CapturePhase : uint8_t { IDLE = 0, COLOR, ENCODE };
  CapturePhase phase_{CapturePhase::IDLE};

  float mintemp_{24.0f};
  float maxtemp_{35.0f};
  int refresh_rate_{-1};
  float filter_level_{10.0f};

  sensor::Sensor *min_temperature_sensor_{nullptr};
  sensor::Sensor *max_temperature_sensor_{nullptr};
  sensor::Sensor *mean_temperature_sensor_{nullptr};
  sensor::Sensor *median_temperature_sensor_{nullptr};

  struct IronColor { uint8_t r, g, b; };
  std::array<IronColor, 256> colormap_lut_{};

  int interleaved_mode_{0};
  int last_subpage_{-1};  // throttle on_frame to one callback per subpage pair
  paramsMLX90640 mlx90640_params_{};
  std::array<float, PIXEL_COUNT> pixels_{};
  std::array<uint16_t, 834> frame_buffer_{};

  float min_v_{24.0f};
  float max_v_{35.0f};
  float mean_temp_{0.0f};
  float median_temp_{0.0f};
  bool data_valid_{false};
  bool iron_palette_{true};

  camera_encoder::EncoderBufferImpl encoder_output_{};
  std::unique_ptr<camera::Encoder> encoder_{};
  uint8_t encoder_quality_{80};
  uint8_t scale_{4};
  size_t encoder_buffer_size_{4096};
  size_t encoder_buffer_expand_size_{1024};

  camera::CameraImageSpec scaled_spec_{0, 0, camera::PIXEL_FORMAT_BGR888};
  std::unique_ptr<camera::BufferImpl> scaled_buffer_{};

  CallbackManager<void()> on_frame_callbacks_;
  std::vector<camera::CameraListener *> listeners_;
  std::shared_ptr<MLX90640CameraImage> current_image_{};
  uint8_t stream_requesters_{0};
  uint8_t single_requesters_{0};
};

}  // namespace mlx90640
}  // namespace esphome

