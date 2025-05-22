from esphome import core, pins
import esphome.codegen as cg
from esphome.components import display
import esphome.config_validation as cv
from esphome.const import (
    CONF_BUSY_PIN,
    CONF_DC_PIN,
    CONF_CS_PIN,
    CONF_ID,
    CONF_FULL_UPDATE_EVERY,
    CONF_LAMBDA,
    CONF_MODEL,
    CONF_PAGES,
    CONF_RESET_DURATION,
    CONF_RESET_PIN,
    CONF_CLK_PIN,
    CONF_MOSI_PIN,
    CONF_ROTATION,
)

crowpanel_epaper_ns = cg.esphome_ns.namespace("crowpanel_epaper")
CrowPanelEPaperBase = crowpanel_epaper_ns.class_(
    "CrowPanelEPaperBase", display.DisplayBuffer
)
CrowPanelEPaper = crowpanel_epaper_ns.class_("CrowPanelEPaper", CrowPanelEPaperBase)

CrowPanelEPaper4P2In = crowpanel_epaper_ns.class_(
    "CrowPanelEPaper4P2In", CrowPanelEPaper
)
CrowPanelEPaper5P79In = crowpanel_epaper_ns.class_(
    "CrowPanelEPaper5P79In", CrowPanelEPaper
)

MODELS = {
    "4.20in": CrowPanelEPaper4P2In,
    "5.79in": CrowPanelEPaper5P79In,
}

CONFIG_SCHEMA = cv.All(
    display.FULL_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(CrowPanelEPaperBase),
            cv.Required(CONF_CLK_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_MOSI_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_CS_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DC_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_BUSY_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_MODEL): cv.one_of(*MODELS, lower=True),
            cv.Optional(CONF_RESET_DURATION): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(max=core.TimePeriod(milliseconds=500)),
            ),
            cv.Optional(CONF_FULL_UPDATE_EVERY): cv.positive_int,
        }
    ),
    cv.has_at_most_one_key(CONF_PAGES, CONF_LAMBDA),
)

async def to_code(config):
    model_class = MODELS[config[CONF_MODEL]]

    rhs = model_class.new()
    var = cg.Pvariable(config[CONF_ID], rhs, model_class)
    
    # Configure pins
    clk_pin_expr = await cg.gpio_pin_expression(config[CONF_CLK_PIN])
    mosi_pin_expr = await cg.gpio_pin_expression(config[CONF_MOSI_PIN])
    cs_pin_expr = await cg.gpio_pin_expression(config[CONF_CS_PIN])
    dc_pin_expr = await cg.gpio_pin_expression(config[CONF_DC_PIN])
    reset_pin_expr = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    busy_pin_expr = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    
    cg.add(var.set_clk_pin(clk_pin_expr))
    cg.add(var.set_mosi_pin(mosi_pin_expr))
    cg.add(var.set_cs_pin(cs_pin_expr))
    cg.add(var.set_dc_pin(dc_pin_expr))
    cg.add(var.set_reset_pin(reset_pin_expr))
    cg.add(var.set_busy_pin(busy_pin_expr))

    await display.register_display(var, config)

    # Set full update frequency if specified
    if CONF_FULL_UPDATE_EVERY in config:
        cg.add(var.set_full_update_every(config[CONF_FULL_UPDATE_EVERY]))
        
    # Set rotation if specified
    if CONF_ROTATION in config:
        rotation_val = config[CONF_ROTATION]
        
        display_rotations = {
            0: cg.RawExpression("esphome::display::DISPLAY_ROTATION_0_DEGREES"),
            90: cg.RawExpression("esphome::display::DISPLAY_ROTATION_90_DEGREES"),
            180: cg.RawExpression("esphome::display::DISPLAY_ROTATION_180_DEGREES"),
            270: cg.RawExpression("esphome::display::DISPLAY_ROTATION_270_DEGREES"),
        }
        
        if rotation_val in display_rotations:
            cg.add(var.set_rotation(display_rotations[rotation_val]))

    # Process lambda for drawing
    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))
