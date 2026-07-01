import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor, remote_receiver

AUTO_LOAD = ["remote_receiver"]
DEPENDENCIES = ["remote_receiver"]

rubicson_ns = cg.esphome_ns.namespace("rubicson")
RubicsonComponent = rubicson_ns.class_("RubicsonComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(RubicsonComponent),
    cv.Required("remote_receiver_id"): cv.use_id(remote_receiver.RemoteReceiverComponent),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    rr = await cg.get_variable(config["remote_receiver_id"])
    cg.add(var.set_remote_receiver(rr))
    await cg.register_component(var, config)
