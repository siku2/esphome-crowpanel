#pragma once

#include "esphome/core/component.h"
#include "esphome/components/display/display_buffer.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace crowpanel_epaper {

static const uint8_t COMMAND_END_MARKER = 0xFF;
static const uint8_t DELAY_FLAG = 0x80;
static const uint8_t ARG_COUNT_MASK = 0x7F;

static const uint16_t NATIVE_WIDTH_4P2IN = 400; 
static const uint16_t NATIVE_HEIGHT_4P2IN = 300;

static const uint16_t NATIVE_WIDTH_5P79IN = 792; 
static const uint16_t NATIVE_HEIGHT_5P79IN = 272;

enum class EpdState {
  IDLE,
  INIT_START,
  INIT_RESET,
  INIT_WAIT_RESET,
  INIT_WAIT_RESET_LOW,
  INIT_WAIT_RESET_HIGH,
  INIT_SEND_COMMANDS,
  INIT_WAIT_BUSY,
  INIT_DONE,
  UPDATE_START,
  UPDATE_WAIT_BUSY,
  UPDATE_PREPARE,
  UPDATE_SENDING_DATA,
  UPDATE_REFRESH,
  UPDATE_WAIT_REFRESH,
  UPDATE_DONE,
  DEEP_SLEEP,
};

enum class UpdateMode {
  FULL,
  PARTIAL
};

class CrowPanelEPaperBase : public display::DisplayBuffer {
 public:
  void set_dc_pin(GPIOPin *dc_pin) { dc_pin_ = dc_pin; }
  void set_cs_pin(GPIOPin *cs_pin) { cs_pin_ = cs_pin; }
  void set_clk_pin(GPIOPin *clk_pin) { clk_pin_ = clk_pin; }
  void set_mosi_pin(GPIOPin *mosi_pin) { mosi_pin_ = mosi_pin; }
  void set_reset_pin(GPIOPin *reset) { this->reset_pin_ = reset; }
  void set_busy_pin(GPIOPin *busy) { this->busy_pin_ = busy; }
  
  void set_full_update_every(uint32_t full_update_every) { this->full_update_every_ = full_update_every; }
  void set_rotation(display::DisplayRotation rotation) { this->rotation_ = rotation; }
  void set_update_mode(UpdateMode mode) { 
    this->force_update_mode_ = mode; 
    this->has_forced_update_mode_ = true;
  }

  float get_setup_priority() const override { return setup_priority::HARDWARE; } 
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  void on_safe_shutdown() override;
  
  void command(uint8_t value);
  void data(uint8_t value);
  
 protected:
  void setup_pins_();
  uint32_t get_buffer_length_();
  
  virtual void initialize() = 0;
  virtual void display() = 0;
  virtual void deep_sleep() = 0;
  
  void start_command_();
  void end_command_();
  void start_data_();
  void end_data_();
  void write_byte_soft_spi(uint8_t data);
  void send_command_sequence_(const uint8_t* sequence);
  
  bool is_idle_();
  bool check_busy_pin_();
  virtual void do_update_();
  
  virtual int get_width_controller() { return this->get_width_internal(); }
  virtual uint32_t idle_timeout_() { return 1000u; }
  
  bool calculate_rotated_coords_(int x, int y, int width, int height, int *out_x, int *out_y);

  GPIOPin *dc_pin_{nullptr};
  GPIOPin *cs_pin_{nullptr};
  GPIOPin *clk_pin_{nullptr};
  GPIOPin *mosi_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *busy_pin_{nullptr};

  uint32_t full_update_every_{10};
  uint32_t update_count_{0};
  display::DisplayRotation rotation_{display::DISPLAY_ROTATION_0_DEGREES};
  
  EpdState state_{EpdState::IDLE};
  uint32_t state_start_time_{0};
  uint32_t data_send_index_{0};
  bool is_full_update_{false};
  bool needs_update_{false};
  
  bool has_forced_update_mode_{false};
  UpdateMode force_update_mode_{UpdateMode::FULL};
};

class CrowPanelEPaper : public CrowPanelEPaperBase {
 public:
  void fill(Color color) override;
  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_BINARY; }

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override;
  
  virtual int get_native_width_() = 0;
  virtual int get_native_height_() = 0;
  
  int get_width_internal() override;
  int get_height_internal() override;
};

class CrowPanelEPaper4P2In : public CrowPanelEPaper {
 public:
  void initialize() override;
  void display() override;
  void dump_config() override;
  void deep_sleep() override;
  
 protected:
  uint32_t idle_timeout_() override { return 60000u; }
  int get_width_controller() override { return NATIVE_WIDTH_4P2IN; }
  int get_native_width_() override { return NATIVE_WIDTH_4P2IN; }
  int get_native_height_() override { return NATIVE_HEIGHT_4P2IN; }
  
  void prepare_for_update_(UpdateMode mode);
};

class CrowPanelEPaper5P79In : public CrowPanelEPaper {
 public:
  void initialize() override;
  void display() override;
  void dump_config() override;
  void deep_sleep() override;
  
 protected:
  uint32_t idle_timeout_() override { return 60000u; }
  int get_width_controller() override { return NATIVE_WIDTH_5P79IN; }
  int get_native_width_() override { return NATIVE_WIDTH_5P79IN; }
  int get_native_height_() override { return NATIVE_HEIGHT_5P79IN; }
  
  void prepare_for_update_(UpdateMode mode);
};

}  // namespace crowpanel_epaper
}  // namespace esphome
