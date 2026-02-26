### Thermal Camera Unit (MLX90640)

https://shop.m5stack.com/products/thermal-camera

Add the external component to your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/onetxpanda/M5Stack-ESPHome/
      ref: native
    components: [mlx90640]
```

Then configure the component:

```yaml
mlx90640:
  name: "Thermal Camera"
  id: thermal_cam
  update_interval: 5s
  i2c_id: bus_a
  address: 0x33
  mintemp: 15   # Lower bound for colour mapping (°C)
  maxtemp: 40   # Upper bound for colour mapping (°C)
  refresh_rate: 4  # 0=0.5Hz 1=1Hz 2=2Hz 3=4Hz 4=8Hz 5=16Hz 6=32Hz 7=64Hz
  min_temperature:
    name: "MLX90640 Min Temp"
  max_temperature:
    name: "MLX90640 Max Temp"
  mean_temperature:
    name: "MLX90640 Mean Temp"
  median_temperature:
    name: "MLX90640 Median Temp"
```

The component integrates with ESPHome's native camera API and is accessible through the ESPHome dashboard and Home Assistant. To also expose a JPEG snapshot or MJPEG stream over HTTP, add the `esp32_camera_web_server` component:

```yaml
esp32_camera_web_server:
  - port: 8080
    mode: stream
  - port: 8081
    mode: snapshot
```
