import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from . import rubicson_ns, RubicsonComponent

DEPENDENCIES = ["rubicson"]

CONF_TEMPERATURE = "temperature"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(RubicsonComponent),
    cv.Required(CONF_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
    ),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[cv.Required(cv.GenerateID())])
    sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
    cg.add(parent.set_temperature_sensor(sens))
