import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, automation
from esphome.automation import maybe_simple_id
from esphome.components import spi
from esphome.components import display
from esphome.const import (
    CONF_HEIGHT,
    CONF_WIDTH,
    CONF_PIN_A,
    CONF_PIN_B,
    CONF_PIN_C,
    CONF_PIN_D,
    CONF_STB_PIN,
    CONF_ENABLE_PIN,
    CONF_SCAN,
    CONF_ID,
    CONF_LAMBDA,
    CONF_BRIGHTNESS,
)
hub75_ns = cg.esphome_ns.namespace("hub75")

CODEOWNERS = ["@buxtronix"]

DEPENDENCIES = ["spi"]

CONF_ROW_SCAN = "row_scan"
CONF_MUX_PATTERN = "mux_pattern"
CONF_BLOCK_PATTERN = "block_pattern"
CONF_COLOR_ORDER = "color_order"
CONF_PIN_E = "pin_e"
CONF_PANELS_WIDE = "panels_wide"
CONF_FAST_UPDATE = "fastupdate"

Hub75 = hub75_ns.class_(
    "Hub75", cg.PollingComponent, display.DisplayBuffer, spi.SPIDevice
)

TurnOnAction = hub75_ns.class_("TurnOnAction", automation.Action)
TurnOffAction = hub75_ns.class_("TurnOffAction", automation.Action)
SetBrightnessAction = hub75_ns.class_("SetBrightnessAction", automation.Action)

validate_brightness = cv.All(cv.int_range(min=0, max=100))

ScanPattern = hub75_ns.enum("ScanPattern")
SCAN_PATTERNS = {
    "LINE": ScanPattern.SCANPATTERN_LINE,
    "ZIGZAG": ScanPattern.SCANPATTERN_ZIGZAG,
    "ZZAGG": ScanPattern.SCANPATTERN_ZZAGG,
    "ZAGGIZ": ScanPattern.SCANPATTERN_ZAGGIZ,
    "WZAGZIG": ScanPattern.SCANPATTERN_WZAGZIG,
    "VZAG": ScanPattern.SCANPATTERN_VZAG,
    "ZAGZIG": ScanPattern.SCANPATTERN_ZAGZIG,
    "WZAGZIG2": ScanPattern.SCANPATTERN_WZAGZIG2,
}
HUB75_SCAN_PATTERN = cv.enum(SCAN_PATTERNS, upper=True)

MuxPattern = hub75_ns.enum("MuxPattern")
MUX_PATTERNS = {
    "BINARY": MuxPattern.MUXPATTERN_BINARY,
    "STRAIGHT": MuxPattern.MUXPATTERN_STRAIGHT,
    "SHIFTREG_ABC": MuxPattern.MUXPATTERN_SHIFTREG_ABC,
    "SHIFTREG_SPI_SE": MuxPattern.MUXPATTERN_SHIFTREG_SPI_SE,
}
HUB75_MUX_PATTERN = cv.enum(MUX_PATTERNS, upper=True)

BlockPattern = hub75_ns.enum("BlockPattern")
BLOCK_PATTERNS = {
    "ABCD": MuxPattern.BLOCKPATTERN_ABCD,
    "DBCA": MuxPattern.BLOCKPATTERN_DBCA,
}
HUB75_BLOCK_PATTERN = cv.enum(BLOCK_PATTERNS, upper=True)

ColorOrder = hub75_ns.enum("ColorOrder")
COLOR_ORDERS = {
    "RRGGBB": ColorOrder.COLORORDER_RRGGBB,
    "RRBBGG": ColorOrder.COLORORDER_RRBBGG,
    "GGRRBB": ColorOrder.COLORORDER_GGRRBB,
    "GGBBRR": ColorOrder.COLORORDER_GGBBRR,
    "BBRRGG": ColorOrder.COLORORDER_BBRRGG,
    "BBGGRR": ColorOrder.COLORORDER_BBGGRR,
}
HUB75_COLOR_ORDER = cv.enum(COLOR_ORDERS, upper=True)

HUB75_SCHEMA = display.FULL_DISPLAY_SCHEMA.extend(
    {
        cv.Required(CONF_ROW_SCAN): cv.one_of(4, 8, 16, 32, int=True),
        cv.Optional(CONF_SCAN, default="LINE"): HUB75_SCAN_PATTERN,
        cv.Optional(CONF_MUX_PATTERN, default="BINARY"): HUB75_MUX_PATTERN,
        cv.Optional(CONF_BLOCK_PATTERN, default="ABCD"): HUB75_BLOCK_PATTERN,
        cv.Optional(CONF_COLOR_ORDER, default="RRGGBB"): HUB75_COLOR_ORDER,
        cv.Optional(CONF_PANELS_WIDE, default=1): cv.int_,
        cv.Required(CONF_PIN_A): pins.gpio_output_pin_schema,
        cv.Required(CONF_PIN_B): pins.gpio_output_pin_schema,
        cv.Optional(CONF_PIN_C): pins.gpio_output_pin_schema,
        cv.Optional(CONF_PIN_D): pins.gpio_output_pin_schema,
        cv.Optional(CONF_PIN_E): pins.gpio_output_pin_schema,
        cv.Required(CONF_STB_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_FAST_UPDATE, default=False): cv.boolean,
    }
).extend(cv.polling_component_schema("1s"))

CONFIG_SCHEMA = cv.All(
    HUB75_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(Hub75),
            cv.Required(CONF_WIDTH): cv.int_,
            cv.Required(CONF_HEIGHT): cv.int_,
            cv.Optional(CONF_BRIGHTNESS, default='100'): validate_brightness,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=False)),
)


async def setup_hub75(var, config):
    await cg.register_component(var, config)
    await display.register_display(var, config)

    pin_a = await cg.gpio_pin_expression(config[CONF_PIN_A])
    cg.add(var.set_pin_a(pin_a))
    pin_b = await cg.gpio_pin_expression(config[CONF_PIN_B])
    cg.add(var.set_pin_b(pin_b))

    if CONF_PIN_C in config:
        pin_c = await cg.gpio_pin_expression(config[CONF_PIN_C])
        cg.add(var.set_pin_c(pin_c))
    if CONF_PIN_D in config:
        pin_d = await cg.gpio_pin_expression(config[CONF_PIN_D])
        cg.add(var.set_pin_d(pin_d))
    if CONF_PIN_E in config:
        pin_e = await cg.gpio_pin_expression(config[CONF_PIN_E])
        cg.add(var.set_pin_e(pin_e))

    pin_stb = await cg.gpio_pin_expression(config[CONF_STB_PIN])
    pin_enable = await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])
    cg.add(var.set_ctrl_pins(pin_stb, pin_enable))
    cg.add(var.set_brightness(config[CONF_BRIGHTNESS]))
    cg.add(var.set_fastupdate(config[CONF_FAST_UPDATE]))

    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayBufferRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))


async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_ROW_SCAN],
        config[CONF_SCAN],
        config[CONF_WIDTH],
        config[CONF_HEIGHT],
        config[CONF_MUX_PATTERN],
        config[CONF_BLOCK_PATTERN],
        config[CONF_COLOR_ORDER],
        config[CONF_PANELS_WIDE],
    )
    await setup_hub75(var, config)
    await spi.register_spi_device(var, config)

BINARY_OUTPUT_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(Hub75),
    }
)


@automation.register_action("hub75.turn_on", TurnOnAction, BINARY_OUTPUT_ACTION_SCHEMA)
async def output_turn_on_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "hub75.turn_off", TurnOffAction, BINARY_OUTPUT_ACTION_SCHEMA
)
async def output_turn_off_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "hub75.set_brightness",
    SetBrightnessAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Hub75),
            cv.Required(CONF_BRIGHTNESS): cv.templatable(validate_brightness),
        },
        key=CONF_BRIGHTNESS,
    ),
)
async def set_brightness_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_BRIGHTNESS], args, cg.uint8)
    cg.add(var.set_brightness(template_))
    return var

