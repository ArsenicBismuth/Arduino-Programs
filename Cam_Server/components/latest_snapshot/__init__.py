import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import web_server_base
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
from esphome.const import CONF_ID

AUTO_LOAD = ["web_server_base"]

CONF_URL = "url"
CONF_ROOT = "root"
CONF_EXTENSION = "extension"

latest_snapshot_ns = cg.esphome_ns.namespace("latest_snapshot")
LatestSnapshot = latest_snapshot_ns.class_("LatestSnapshot", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LatestSnapshot),
        cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(
            web_server_base.WebServerBase
        ),
        cv.Optional(CONF_URL, default="/latest"): cv.string_strict,
        cv.Optional(CONF_ROOT, default="/sdcard"): cv.string_strict,
        cv.Optional(CONF_EXTENSION, default=".jpg"): cv.string_strict,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_WEB_SERVER_BASE_ID])
    var = cg.new_Pvariable(config[CONF_ID], paren)
    await cg.register_component(var, config)
    cg.add(var.set_url(config[CONF_URL]))
    cg.add(var.set_root(config[CONF_ROOT]))
    cg.add(var.set_extension(config[CONF_EXTENSION]))
