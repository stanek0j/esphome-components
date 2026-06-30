import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor, remote_receiver

DEPENDENCIES = ["remote_receiver"]

rubicson_ns = cg.esphome_ns.namespace("rubicson")
RubicsonComponent = rubicson_ns.class_("RubicsonComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(RubicsonComponent),
}).extend(cv.COMPONENT_SCHEMA)
