import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from . import rubicson_ns, RubicsonComponent

DEPENDENCIES = ["rubicson"]

CONF_BATTERY = "battery"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(RubicsonComponent),
    cv.Required(CONF_BATTERY): binary_sensor.binary_sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[cv.Required(cv.GenerateID())])
    sens = await binary_sensor.new_binary_sensor(config[CONF_BATTERY])
    cg.add(parent.set_battery_sensor(sens))
