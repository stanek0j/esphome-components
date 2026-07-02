"""
ESPHome external component — Rubicson 433 MHz thermometer decoder.

Decodes the same 36-bit OOK-PWM protocol as rtl_433's rubicson device.
Requires: CC1101 (or any OOK receiver) driving a remote_receiver pin.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

# ── Dependencies ──────────────────────────────────────────────────────────────
DEPENDENCIES = ["remote_receiver"]
AUTO_LOAD = ["sensor", "binary_sensor"]

# ── Namespaces ────────────────────────────────────────────────────────────────
rubicson_ns     = cg.esphome_ns.namespace("rubicson")
remote_base_ns  = cg.esphome_ns.namespace("remote_base")
remote_recv_ns  = cg.esphome_ns.namespace("remote_receiver")

RubicsonComponent = rubicson_ns.class_(
    "RubicsonComponent",
    cg.Component,
    remote_base_ns.class_("RemoteReceiverListener"),
)
RemoteReceiverComponent = remote_recv_ns.class_("RemoteReceiverComponent")

# ── Config keys ───────────────────────────────────────────────────────────────
CONF_RECEIVER_ID = "receiver_id"
CONF_TEMPERATURE = "temperature"
CONF_BATTERY_LOW = "battery_low"
CONF_SENSOR_ID   = "sensor_id"
CONF_CHANNEL     = "channel"

# ── Schema ────────────────────────────────────────────────────────────────────
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RubicsonComponent),
            # Reference the remote_receiver component that feeds us raw pulses
            cv.GenerateID(CONF_RECEIVER_ID): cv.use_id(RemoteReceiverComponent),

            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_LOW): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_BATTERY,
            ),

            # -1 means "accept any value" (no filter)
            cv.Optional(CONF_SENSOR_ID, default=-1): cv.int_range(min=-1, max=255),
            cv.Optional(CONF_CHANNEL,   default=-1): cv.int_range(min=-1, max=3),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    # At least one output must be configured
    cv.has_at_least_one_key(CONF_TEMPERATURE, CONF_BATTERY_LOW),
)


# ── Code generation ───────────────────────────────────────────────────────────
async def to_code(config: dict) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Register as a listener on the remote_receiver component
    recv = await cg.get_variable(config[CONF_RECEIVER_ID])
    cg.add(recv.register_listener(var))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_BATTERY_LOW in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_BATTERY_LOW])
        cg.add(var.set_battery_low_sensor(bs))

    cg.add(var.set_sensor_id(config[CONF_SENSOR_ID]))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
