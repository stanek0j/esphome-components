from esphome import config_validation as cv
import esphome.codegen as cg
from esphome.components import sensor, binary_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["remote_receiver"]

rubicson_ns = cg.esphome_ns.namespace("rubicson")
RubicsonComponent = rubicson_ns.class_("RubicsonComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RubicsonComponent),

        cv.Optional("temperature"): sensor.sensor_schema(
            unit_of_measurement="°C",
            accuracy_decimals=1,
            device_class="temperature",
            state_class="measurement",
        ),

        cv.Optional("battery"): binary_sensor.binary_sensor_schema(
            device_class="battery"
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Temperature sensor
    if "temperature" in config:
        sens = await sensor.new_sensor(config["temperature"])
        cg.add(var.set_temperature_sensor(sens))

    # Battery binary sensor
    if "battery" in config:
        batt = await binary_sensor.new_binary_sensor(config["battery"])
        cg.add(var.set_default_battery_sensor(batt))
