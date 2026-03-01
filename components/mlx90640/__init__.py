import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import i2c, sensor, socket
from esphome.components.esp32 import add_idf_component
from esphome.core import coroutine_with_priority
from esphome.const import (
    CONF_BUFFER_SIZE,
    CONF_ID,
    CONF_MAX_TEMPERATURE,
    CONF_MIN_TEMPERATURE,
    CONF_NAME,
    CONF_TRIGGER_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)
from esphome.core.entity_helpers import setup_entity

CONF_ON_FRAME = "on_frame"
CONF_REFRESH_RATE = "refresh_rate"
CONF_MEAN_TEMPERATURE = "mean_temperature"
CONF_MEDIAN_TEMPERATURE = "median_temperature"
CONF_MINTEMP = "mintemp"
CONF_MAXTEMP = "maxtemp"
CONF_FILTER_LEVEL = "filter_level"
CONF_BUFFER_EXPAND_SIZE = "buffer_expand_size"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_JPEG_SCALE = "jpeg_scale"
CONF_IRON_PALETTE = "iron_palette"

DEPENDENCIES = ["i2c", "esp32"]
AUTO_LOAD = ["sensor", "camera", "camera_encoder", "socket"]

mlx90640_ns = cg.esphome_ns.namespace("mlx90640")
MLX90640 = mlx90640_ns.class_("MLX90640", i2c.I2CDevice, cg.Component, cg.EntityBase)
MLX90640FrameTrigger = mlx90640_ns.class_("MLX90640FrameTrigger", automation.Trigger.template())
CONFIG_SCHEMA = (
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(MLX90640),
            cv.Required(CONF_NAME): cv.string_strict,
            cv.Required(CONF_MAXTEMP): cv.float_,
            cv.Required(CONF_MINTEMP): cv.float_,
            cv.Optional(CONF_REFRESH_RATE): cv.int_range(min=0, max=7),
            cv.Optional(CONF_FILTER_LEVEL): cv.float_,
            cv.Optional(CONF_IRON_PALETTE, default=True): cv.boolean,
            cv.Optional(CONF_JPEG_QUALITY, default=80): cv.int_range(min=1, max=100),
            cv.Optional(CONF_JPEG_SCALE, default=4): cv.int_range(min=1, max=10),
            cv.Optional(CONF_BUFFER_SIZE): cv.int_range(min=1024, max=2 * 1024 * 1024),
            cv.Optional(CONF_BUFFER_EXPAND_SIZE, default=1024): cv.int_range(
                min=0, max=2 * 1024 * 1024
            ),
            cv.Optional(CONF_MIN_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MAX_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MEAN_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ON_FRAME): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(MLX90640FrameTrigger)}
            ),
            cv.Optional(CONF_MEDIAN_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(i2c.i2c_device_schema(0x33))
)


@coroutine_with_priority(45.0)
async def to_code(config):
    cg.add_define("USE_CAMERA")
    socket.require_wake_loop_threadsafe()
    cg.add_define("USE_ESP32_CAMERA_JPEG_ENCODER")
    add_idf_component(name="espressif/esp32-camera", ref="2.1.1")

    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    scale = config[CONF_JPEG_SCALE]
    iron_palette = config[CONF_IRON_PALETTE]
    cg.add(var.set_iron_palette(iron_palette))
    # JPEG output can never exceed the uncompressed pixel size, so use that as
    # the automatic upper bound. Users can still override with buffer_size.
    bpp = 3 if iron_palette else 1
    auto_buffer_size = 32 * scale * 24 * scale * bpp
    buffer_size = config.get(CONF_BUFFER_SIZE, auto_buffer_size)
    cg.add(var.set_encoder_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_encoder_buffer_size(buffer_size))
    cg.add(var.set_encoder_buffer_expand_size(config[CONF_BUFFER_EXPAND_SIZE]))
    cg.add(var.set_jpeg_scale(scale))

    for conf in config.get(CONF_ON_FRAME, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [], conf)
        cg.add(var.register_on_frame_trigger(trigger))

    if CONF_MIN_TEMPERATURE in config:
        conf = config[CONF_MIN_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_min_temperature_sensor(sens))

    if CONF_MAX_TEMPERATURE in config:
        conf = config[CONF_MAX_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_max_temperature_sensor(sens))
    
    if CONF_MEAN_TEMPERATURE in config:
        conf = config[CONF_MEAN_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_mean_temperature_sensor(sens))
    
    if CONF_MEDIAN_TEMPERATURE in config:
        conf = config[CONF_MEDIAN_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_median_temperature_sensor(sens))
        
    if CONF_MINTEMP in config:
        min = config[CONF_MINTEMP]
        cg.add(var.set_mintemp(min))

    if CONF_MAXTEMP in config:
        max = config[CONF_MAXTEMP]
        cg.add(var.set_maxtemp(max))

    if CONF_REFRESH_RATE in config:
        refresh = config[CONF_REFRESH_RATE]
        cg.add(var.set_refresh_rate(refresh))

    if CONF_FILTER_LEVEL in config:
        level = config[CONF_FILTER_LEVEL]
        cg.add(var.set_filter_level(level))
