#include "crowpanel_epaper.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace crowpanel_epaper {

static const char *const TAG = "crowpanel_epaper";

// SSD1683 EPD Driver chip command definitions
static const uint8_t CMD_SOFT_RESET = 0x12;
static const uint8_t CMD_DISPLAY_UPDATE_CONTROL = 0x21;
static const uint8_t CMD_DISPLAY_UPDATE = 0x20;
static const uint8_t CMD_DEEP_SLEEP = 0x10;
static const uint8_t CMD_DATA_ENTRY_MODE = 0x11;
static const uint8_t CMD_BORDER_WAVEFORM = 0x3C;
static const uint8_t CMD_WRITE_RAM = 0x24;
static const uint8_t CMD_UPDATE_SEQUENCE = 0x22;
static const uint8_t CMD_SET_X_ADDR = 0x44;
static const uint8_t CMD_SET_Y_ADDR = 0x45;
static const uint8_t CMD_SET_X_COUNTER = 0x4E;
static const uint8_t CMD_SET_Y_COUNTER = 0x4F;
static const uint8_t CMD_SET_MUX = 0x01;
static const uint8_t CMD_TEMP_SENSOR_CTL = 0x18;
static const uint8_t CMD_SET_TEMP = 0x1A;

// Explicit target selection when the SSD1683 is used in cascade mode.
static const uint8_t CMD_TARGET_PRIMARY = 0x00;
static const uint8_t CMD_TARGET_SECONDARY = 0x80;

// SSD1683 EPD Driver chip command parameters
static const uint8_t PARAM_BORDER_FULL = 0x05;
static const uint8_t PARAM_BORDER_PARTIAL = 0x80;
static const uint8_t PARAM_FULL_UPDATE = 0xF7;
static const uint8_t PARAM_PARTIAL_UPDATE = 0xFF;
static const uint8_t PARAM_DEEP_SLEEP_MODE = 0x01;
static const uint8_t PARAM_INTERNAL_TEMP_SENSOR = 0x80;
static const uint8_t PARAM_X_INC_Y_INC = 0x03; // left-right, top-down
static const uint8_t PARAM_X_DEC_Y_INC = 0x02; // right-left, top-down

// SSD1683 EPD Driver chip command sequences
// Format: command, num_args, arg1, arg2...
// Special case: if num_args has bit 7 set (0x80), it indicates a delay command
// End marker is two 0xFF bytes

const uint8_t display_start_sequence[] = {
  CMD_SOFT_RESET, DELAY_FLAG, 10,                    // Soft reset and 10ms delay
  CMD_SET_MUX, 0x03, 0x2b, 0x01, 0x00,               // Set MUX as 300
  CMD_DISPLAY_UPDATE_CONTROL, 0x02, 0x40, 0x00,      // Display update control
  CMD_BORDER_WAVEFORM, 0x01, PARAM_BORDER_FULL,      // Border waveform for full refresh
  CMD_DATA_ENTRY_MODE, 0x01, PARAM_X_INC_Y_INC,      // Data entry mode (X+ Y+)
  CMD_SET_X_ADDR, 0x02, 0x00, 0x31,                  // Set RAM X Address Start/End Pos (0 to 49 -> 400 pixels)
  CMD_SET_Y_ADDR, 0x04, 0x00, 0x00, 0x2b, 0x01,      // Set RAM Y Address Start/End Pos (0 to 299 -> 300 pixels)
  CMD_SET_X_COUNTER, 0x01, 0x00,                     // Set RAM X Address counter
  CMD_SET_Y_COUNTER, 0x02, 0x00, 0x00,               // Set RAM Y Address counter
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

const uint8_t display_start_sequence_5p79in[] = {
  CMD_SOFT_RESET, DELAY_FLAG, 10,                        // Soft reset and 10ms delay
  // CMD_SET_MUX, 0x03, 0x2b, 0x01, 0x00,                   // Set MUX as 300
  CMD_TEMP_SENSOR_CTL, 0x01, PARAM_INTERNAL_TEMP_SENSOR, // Enable internal temperature sensor
  // CMD_DISPLAY_UPDATE_CONTROL, 0x02, 0x40, 0x10,          // Display update control
  CMD_UPDATE_SEQUENCE, 0x01, 0xB1,                       // Enable clock signal, Load temperature value, Load LUT (black/white mode), Disable clock signal
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                    // Master activation with 10ms delay
  CMD_SET_TEMP, 0x02, 0x64, 0x00,                        // Write to temperature register
  CMD_UPDATE_SEQUENCE, 0x01, 0x91,                       // Load temperature value, Load LUT (black/white mode), Disable clock signal
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                    // Master activation with 10ms delay
  COMMAND_END_MARKER, COMMAND_END_MARKER                 // End marker
  // CMD_BORDER_WAVEFORM, 0x01, PARAM_BORDER_FULL,          // Border waveform for full refresh
  // Primary panel
  // CMD_DATA_ENTRY_MODE, 0x01, PARAM_X_INC_Y_INC,      // Data entry mode (X+ Y+)
  // CMD_SET_X_ADDR,      0x02, 0x00, 0x31,             // Set RAM X Address Start/End Pos (0 to 49 -> 400 pixels)
  // CMD_SET_Y_ADDR,      0x04, 0x00, 0x00, 0x0f, 0x01, // Set RAM Y Address Start/End Pos (0 to 271 -> 272 pixels)
  // CMD_SET_X_COUNTER,   0x01, 0x00,                   // Set RAM X Address counter
  // CMD_SET_Y_COUNTER,   0x02, 0x00, 0x00,             // Set RAM Y Address counter
  // // Secondary panel
  // CMD_DATA_ENTRY_MODE | CMD_TARGET_SECONDARY, 0x01, PARAM_X_INC_Y_INC,      // Data entry mode (X+ Y+)
  // CMD_SET_X_ADDR | CMD_TARGET_SECONDARY,      0x02, 0x00, 0x31,             // Set RAM X Address Start/End Pos (0 to 49 -> 400 pixels)
  // CMD_SET_Y_ADDR | CMD_TARGET_SECONDARY,      0x04, 0x00, 0x00, 0x0f, 0x01, // Set RAM Y Address Start/End Pos (0 to 271 -> 272 pixels)
  // CMD_SET_X_COUNTER | CMD_TARGET_SECONDARY,   0x01, 0x00,                   // Set RAM X Address counter
  // CMD_SET_Y_COUNTER | CMD_TARGET_SECONDARY,   0x02, 0x00, 0x00,             // Set RAM Y Address counter
};

const uint8_t display_stop_sequence[] = {
  CMD_DEEP_SLEEP, 0x01, PARAM_DEEP_SLEEP_MODE,       // Deep sleep mode
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

const uint8_t full_refresh_sequence[] = {
  CMD_UPDATE_SEQUENCE, 0x01, PARAM_FULL_UPDATE,      // Display update sequence option (full)
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                // Master activation with 10ms delay
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

const uint8_t fast_full_refresh_sequence[] = {
  CMD_DISPLAY_UPDATE_CONTROL, 0x02, 0x40, 0x10,      // Display Update Control (bypass RED, cascade application)
  CMD_SET_TEMP, 0x02, 0x64, 0x00,                    // Set temperature register
  CMD_UPDATE_SEQUENCE, 0x01, 0xd7,                   // Display update control 2
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                // Master activation with 10ms delay
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

const uint8_t partial_refresh_sequence[] = {
  CMD_UPDATE_SEQUENCE, 0x01, PARAM_PARTIAL_UPDATE,   // Display update sequence option (partial)
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                // Master activation with 10ms delay
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

// ========================================================
// CrowPanelEPaperBase Implementation - SPI Communication
// ========================================================

void CrowPanelEPaperBase::setup_pins_() {
  // Configure all the GPIO pins
  this->dc_pin_->setup();
  this->dc_pin_->digital_write(true);
  this->cs_pin_->setup();
  this->cs_pin_->digital_write(true); // Initialize CS high (inactive)
  this->clk_pin_->setup();
  this->clk_pin_->digital_write(false); // Clock low initially
  this->mosi_pin_->setup();

  if (this->reset_pin_ != nullptr)
    this->reset_pin_->setup();
  if (this->busy_pin_ != nullptr) {
    this->busy_pin_->pin_mode(gpio::FLAG_INPUT);
  }
}

void CrowPanelEPaperBase::write_byte_soft_spi(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    this->clk_pin_->digital_write(false); // SCK Low
    if (data & 0x80) {
      this->mosi_pin_->digital_write(true); // MOSI High
    } else {
      this->mosi_pin_->digital_write(false); // MOSI Low
    }
    this->clk_pin_->digital_write(true); // SCK High (Clock in data)
    data <<= 1; // Shift next bit into position
  }
}

void CrowPanelEPaperBase::command(uint8_t value) {
  this->start_command_();
  this->write_byte_soft_spi(value);
  this->end_command_();
}

void CrowPanelEPaperBase::data(uint8_t value) {
  this->start_data_();
  this->write_byte_soft_spi(value);
  this->end_data_();
}

void CrowPanelEPaperBase::start_command_() {
  this->dc_pin_->digital_write(false); // DC Low for command
  this->cs_pin_->digital_write(false); // CS Low (Enable chip)
}

void CrowPanelEPaperBase::end_command_() {
  this->cs_pin_->digital_write(true); // CS High (Disable chip)
  this->dc_pin_->digital_write(true); // Set DC back high (safer default?)
}

void CrowPanelEPaperBase::start_data_() {
  this->dc_pin_->digital_write(true); // DC High for data
  this->cs_pin_->digital_write(false); // CS Low (Enable chip)
}

void CrowPanelEPaperBase::end_data_() {
  this->cs_pin_->digital_write(true); // CS High (Disable chip)
}

void CrowPanelEPaperBase::send_command_sequence_(const uint8_t* sequence) {
  if (sequence == nullptr)
    return;
    
  uint32_t i = 0;
  while (true) {
    uint8_t cmd = sequence[i++];
    uint8_t num_args = sequence[i++];
    
    // End marker check
    if (cmd == COMMAND_END_MARKER && num_args == COMMAND_END_MARKER)
      break;
      
    this->command(cmd);
    
    // Check if this command has a delay parameter
    if (num_args & DELAY_FLAG) {
      // This is a delay command, skip the delay value
      // The actual delay is handled by the state machine
      // TODO: no it isn't?
      uint8_t delay_ms = sequence[i++];
      delay(delay_ms); // Delay for the specified time
      // Continue sending commands (don't break)
      num_args &= ARG_COUNT_MASK; // Clear delay bit for arg count
    }
    
    // Send all args
    for (uint8_t j = 0; j < num_args; j++) {
      this->data(sequence[i++]);
    }
  }
}

// ========================================================
// CrowPanelEPaperBase Implementation - State Machine
// ========================================================

bool CrowPanelEPaperBase::check_busy_pin_() {
  if (this->busy_pin_ == nullptr)
    return false;
    
  return this->busy_pin_->digital_read();
}

bool CrowPanelEPaperBase::is_idle_() {
  if (this->busy_pin_ == nullptr)
    return true;
    
  // Low means idle, high means busy
  return !this->check_busy_pin_();
}

bool CrowPanelEPaperBase::calculate_rotated_coords_(int x, int y, int width, int height, int *out_x, int *out_y) {
  switch (this->rotation_) {
    case display::DISPLAY_ROTATION_0_DEGREES:
      *out_x = x;
      *out_y = y;
      break;
    case display::DISPLAY_ROTATION_90_DEGREES:
      *out_x = y;
      *out_y = width - 1 - x;
      break;
    case display::DISPLAY_ROTATION_180_DEGREES:
      *out_x = width - 1 - x;
      *out_y = height - 1 - y;
      break;
    case display::DISPLAY_ROTATION_270_DEGREES:
      *out_x = height - 1 - y;
      *out_y = x;
      break;
    default:
      // Invalid rotation
      return false;
  }
  
  // Check if the rotated coordinates are within the display bounds
  if (*out_x >= width || *out_y >= height || *out_x < 0 || *out_y < 0)
    return false;
    
  return true;
}

void CrowPanelEPaperBase::setup() {
  ESP_LOGD(TAG, "Setting up CrowPanel E-Paper");
  
  uint32_t buffer_size = this->get_buffer_length_();
  this->init_internal_(buffer_size);
  
  this->fill(display::COLOR_OFF);
  this->setup_pins_();
  
  // Start initialization state machine
  this->state_ = EpdState::INIT_START;
  this->state_start_time_ = millis();
}

void CrowPanelEPaperBase::loop() {
  // Main state machine to handle non-blocking operations
  uint32_t now = millis();
  
  switch (this->state_) {
    case EpdState::IDLE:
      if (this->needs_update_) {
        this->needs_update_ = false;
        this->state_ = EpdState::UPDATE_START;
        this->state_start_time_ = now;
        ESP_LOGD(TAG, "Starting display update");
      }
      break;
      
    case EpdState::INIT_START:
      // Begin initialization sequence
      ESP_LOGD(TAG, "Initializing display");
      this->state_ = EpdState::INIT_RESET;
      this->state_start_time_ = now;
      break;
      
    case EpdState::INIT_RESET:
      if (this->reset_pin_ != nullptr) {
        this->reset_pin_->digital_write(true);
        this->state_ = EpdState::INIT_WAIT_RESET;
        this->state_start_time_ = now;
      } else {
        this->state_ = EpdState::INIT_SEND_COMMANDS;
      }
      break;
      
    case EpdState::INIT_WAIT_RESET:
      if (now - this->state_start_time_ >= 100) {
        // Reset pulse sequence
        this->reset_pin_->digital_write(false);
        this->state_ = EpdState::INIT_WAIT_RESET_LOW;
        this->state_start_time_ = now;
      }
      break;
      
    case EpdState::INIT_WAIT_RESET_LOW:
      if (now - this->state_start_time_ >= 10) {
        this->reset_pin_->digital_write(true);
        this->state_ = EpdState::INIT_WAIT_RESET_HIGH;
        this->state_start_time_ = now;
      }
      break;
    
    case EpdState::INIT_WAIT_RESET_HIGH:
      if (now - this->state_start_time_ >= 10) {
        this->state_ = EpdState::INIT_SEND_COMMANDS;
      }
      break;
      
    case EpdState::INIT_SEND_COMMANDS:
      this->initialize();
      this->state_ = EpdState::INIT_WAIT_BUSY;
      this->state_start_time_ = now;
      break;
      
    case EpdState::INIT_WAIT_BUSY:
      if (this->is_idle_() || now - this->state_start_time_ > this->idle_timeout_()) {
        this->state_ = EpdState::INIT_DONE;
        this->update_count_ = 0;
        ESP_LOGD(TAG, "Display initialization complete");
      }
      break;
      
    case EpdState::INIT_DONE:
      this->state_ = EpdState::IDLE;
      break;
      
    case EpdState::UPDATE_START:
      this->update_count_++;
      
      // Determine update mode (forced or automatic)
      if (this->has_forced_update_mode_) {
        this->is_full_update_ = (this->force_update_mode_ == UpdateMode::FULL);
      } else {
        // Ensure the very first update is always full
        this->is_full_update_ = (this->update_count_ == 1 || (this->update_count_ % this->full_update_every_ == 0));
      }
      
      ESP_LOGD(TAG, "Performing %s display update (%u)", 
                 this->is_full_update_ ? "FULL" : "PARTIAL", this->update_count_);
      
      // Clear buffer to white first
      this->fill(display::COLOR_OFF);
      
      // Execute the lambda (if set) - this draws text, shapes, etc.
      if (this->writer_.has_value()) {
        (*this->writer_)(*this);
      }
      
      this->state_ = EpdState::UPDATE_WAIT_BUSY;
      this->state_start_time_ = now;
      break;
      
    case EpdState::UPDATE_WAIT_BUSY:
      if (this->is_idle_() || now - this->state_start_time_ > this->idle_timeout_()) {
        this->state_ = EpdState::UPDATE_PREPARE;
      }
      break;
      
    case EpdState::UPDATE_PREPARE:
      this->display(); // Set up for data transfer
      this->state_ = EpdState::UPDATE_SENDING_DATA;
      this->state_start_time_ = now;
      break;
    case EpdState::UPDATE_SENDING_DATA: {
      this->update_sending_data_(now);
      break;
    }
    case EpdState::UPDATE_WAIT_REFRESH:
      if (this->is_idle_() || now - this->state_start_time_ > this->idle_timeout_()) {
        this->state_ = EpdState::UPDATE_DONE;
        ESP_LOGD(TAG, "Display update complete");
      }
      break;
      
    case EpdState::UPDATE_DONE:
      this->state_ = EpdState::IDLE;
      break;
      
    case EpdState::DEEP_SLEEP:
      // Stay in deep sleep state
      break;
  }
}

void CrowPanelEPaperBase::update_sending_data_(uint32_t now) {
  // Send a chunk of data per loop
  const size_t CHUNK_SIZE = 32;
  size_t buffer_len = this->get_buffer_length_();
  size_t i = this->data_send_index_;
  size_t end = (i + CHUNK_SIZE < buffer_len) ? (i + CHUNK_SIZE) : buffer_len;
  for (; i < end; ++i) {
    this->write_byte_soft_spi(this->buffer_[i]);
  }
  this->data_send_index_ = end;
  if (this->data_send_index_ >= buffer_len) {
    this->end_data_();
    // Send refresh command based on update mode
    UpdateMode mode = this->is_full_update_ ? UpdateMode::FULL : UpdateMode::PARTIAL;
    if (mode == UpdateMode::FULL) {
      this->send_command_sequence_(full_refresh_sequence);
    } else {
      this->send_command_sequence_(partial_refresh_sequence);
    }
    this->state_ = EpdState::UPDATE_WAIT_REFRESH;
    this->state_start_time_ = now;
  }
}

void CrowPanelEPaperBase::update() {
  this->do_update_();
}

void CrowPanelEPaperBase::do_update_() {
  // Just set the flag - actual update will happen in loop()
  this->needs_update_ = true;
}

void CrowPanelEPaperBase::on_safe_shutdown() { 
  this->state_ = EpdState::DEEP_SLEEP;
  this->deep_sleep(); 
}

void CrowPanelEPaperBase::dump_config() {
    LOG_DISPLAY("", "CrowPanel E-Paper (Software SPI)", this);
    LOG_PIN("  CS Pin: ", this->cs_pin_);
    LOG_PIN("  DC Pin: ", this->dc_pin_);
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
    LOG_PIN("  Busy Pin: ", this->busy_pin_);
    LOG_PIN("  CLK Pin: ", this->clk_pin_);
    LOG_PIN("  MOSI Pin: ", this->mosi_pin_);
    ESP_LOGCONFIG(TAG, "  Full Update Every: %u", this->full_update_every_);
    
    const char *rotation_str;
    switch (this->rotation_) {
      case display::DISPLAY_ROTATION_0_DEGREES:
        rotation_str = "0°";
        break;
      case display::DISPLAY_ROTATION_90_DEGREES:
        rotation_str = "90°";
        break;
      case display::DISPLAY_ROTATION_180_DEGREES:
        rotation_str = "180°";
        break;
      case display::DISPLAY_ROTATION_270_DEGREES:
        rotation_str = "270°";
        break;
      default:
        rotation_str = "UNKNOWN";
    }
    ESP_LOGCONFIG(TAG, "  Rotation: %s", rotation_str);
    
    if (this->has_forced_update_mode_) {
      ESP_LOGCONFIG(TAG, "  Forced Update Mode: %s", 
        this->force_update_mode_ == UpdateMode::FULL ? "FULL" : "PARTIAL");
    }
}

uint32_t CrowPanelEPaperBase::get_buffer_length_() {
  return this->get_width_internal() * this->get_height_internal() / 8u;
}

// ========================================================
// CrowPanelEPaper Implementation (Basic B/W display)
// ========================================================

int CrowPanelEPaper::get_width_internal() {
  switch (this->rotation_) {
    case display::DISPLAY_ROTATION_90_DEGREES:
    case display::DISPLAY_ROTATION_270_DEGREES:
      return this->get_native_height_();
    case display::DISPLAY_ROTATION_0_DEGREES:
    case display::DISPLAY_ROTATION_180_DEGREES:
    default:
      return this->get_native_width_();
  }
}

int CrowPanelEPaper::get_height_internal() {
  switch (this->rotation_) {
    case display::DISPLAY_ROTATION_90_DEGREES:
    case display::DISPLAY_ROTATION_270_DEGREES:
      return this->get_native_width_();
    case display::DISPLAY_ROTATION_0_DEGREES:
    case display::DISPLAY_ROTATION_180_DEGREES:
    default:
      return this->get_native_height_();
  }
}

void CrowPanelEPaper::fill(Color color) {
  const uint8_t fill = !color.is_on() ? 0x00 : 0xFF;
  ESP_LOGD(TAG, "Filling buffer with %s", !color.is_on() ? "BLACK" : "WHITE");
  
  if (this->get_buffer_length_() == 0 || this->buffer_ == nullptr) {
    ESP_LOGE(TAG, "ERROR: Buffer not initialized");
    return;
  }
  
  for (uint32_t i = 0; i < this->get_buffer_length_(); i++)
    this->buffer_[i] = fill;
}

void CrowPanelEPaper::draw_absolute_pixel_internal(int x, int y, Color color) {
  int rotated_x, rotated_y;
  int native_w = this->get_native_width_();
  int native_h = this->get_native_height_();

  // Apply rotation.
  if (!this->calculate_rotated_coords_(x, y, native_w, native_h, &rotated_x, &rotated_y)) {
    return; // Coordinates out of bounds after rotation
  }

  rotated_x = native_w - 1 - rotated_x;

  // Use the directly rotated coordinates for buffer calculation
  // Check bounds using the correctly rotated coordinates
  if (rotated_x < 0 || rotated_x >= native_w || rotated_y < 0 || rotated_y >= native_h) {
     // Should not happen if calculate_rotated_coords_ includes a check, but good practice.
     return;
  }

  // Calculate buffer offset using rotated coordinates
  const uint32_t byte_offset = (rotated_y * native_w + rotated_x) / 8u;
  const uint8_t bit_offset = 7 - (rotated_x % 8); // MSB is leftmost pixel

  // Check buffer bounds
  if (byte_offset >= this->get_buffer_length_()) {
    // ESP_LOGE(TAG, "ERROR: Attempt to write outside buffer (%d, %d) -> rotated (%d, %d) -> byte %u >= %u", x, y, rotated_x, rotated_y, byte_offset, this->get_buffer_length_());
    return; // Error: write outside buffer
  }

  // Write pixel
  // Since this is an EPD, on is black and off is white.
  if (color.is_on()) {
    this->buffer_[byte_offset] &= ~(1 << bit_offset);
  } else {
    this->buffer_[byte_offset] |= (1 << bit_offset);
  }
}

// ========================================================
// CrowPanelEPaper4P2In Implementation (4.2" B/W display)
// ========================================================

void CrowPanelEPaper4P2In::initialize() {
  ESP_LOGD(TAG, "Initializing CrowPanel 4.2in display");

  // Send initialization sequence using the predefined commands
  this->send_command_sequence_(display_start_sequence);
}

void CrowPanelEPaper4P2In::prepare_for_update_(UpdateMode mode) {
  if (mode == UpdateMode::FULL) {
    ESP_LOGD(TAG, "Preparing for FULL update mode");
    
    // Set BorderWavefrom for full refresh
    this->command(CMD_BORDER_WAVEFORM);
    this->data(PARAM_BORDER_FULL);
    
    // Additional display update control settings 
    this->command(CMD_DISPLAY_UPDATE_CONTROL);
    this->data(0x40);
    this->data(0x00);
  } else {
    ESP_LOGD(TAG, "Preparing for PARTIAL update mode");
    
    // Set BorderWavefrom for partial refresh
    this->command(CMD_BORDER_WAVEFORM);
    this->data(PARAM_BORDER_PARTIAL);
    
    // Additional settings for partial update
    this->command(CMD_DISPLAY_UPDATE_CONTROL);
    this->data(0x00);
    this->data(0x00);
  }
}

void CrowPanelEPaper4P2In::display() {
  ESP_LOGD(TAG, "E-Paper display refresh starting");
  // Set the display mode based on update type
  UpdateMode mode = this->is_full_update_ ? UpdateMode::FULL : UpdateMode::PARTIAL;
  this->prepare_for_update_(mode);
  // Reset RAM address counters before writing data
  this->command(CMD_SET_X_COUNTER);
  this->data(0x00);
  this->command(CMD_SET_Y_COUNTER);
  this->data(0x00);
  this->data(0x00);
  // Send command to write to BLACK/WHITE RAM
  this->command(CMD_WRITE_RAM);
  // Start non-blocking data transfer (handled in state machine)
  this->start_data_();
  this->data_send_index_ = 0;
}

void CrowPanelEPaper4P2In::deep_sleep() {
  ESP_LOGD(TAG, "Entering deep sleep mode");
  
  // Send deep sleep sequence
  this->send_command_sequence_(display_stop_sequence);
}

void CrowPanelEPaper4P2In::dump_config() {
  LOG_DISPLAY("", "CrowPanel E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 4.2in");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

// ========================================================
// CrowPanelEPaper5P79In Implementation (5.79" B/W display)
//
// This model uses the cascade mode of the SSD1683 to chain
// two controllers together.
// The left half (closest to the connector) is the
// primary controller.
// ========================================================

void CrowPanelEPaper5P79In::initialize() {
  ESP_LOGD(TAG, "Initializing CrowPanel 5.79in display");

  // Send initialization sequence using the predefined commands
  this->send_command_sequence_(display_start_sequence_5p79in);
}

void CrowPanelEPaper5P79In::prepare_for_update_(UpdateMode mode) {
  // if (mode == UpdateMode::FULL) {
  //   ESP_LOGD(TAG, "Preparing for FULL update mode");
    
  //   // Set BorderWavefrom for full refresh
  //   this->command(CMD_BORDER_WAVEFORM);
  //   this->data(PARAM_BORDER_FULL);
    
  //   // Additional display update control settings 
  //   this->command(CMD_DISPLAY_UPDATE_CONTROL);
  //   this->data(0x40);
  //   this->data(0x00);
  // } else {
  //   ESP_LOGD(TAG, "Preparing for PARTIAL update mode");
    
  //   // Set BorderWavefrom for partial refresh
  //   this->command(CMD_BORDER_WAVEFORM);
  //   this->data(PARAM_BORDER_PARTIAL);
    
  //   // Additional settings for partial update
  //   this->command(CMD_DISPLAY_UPDATE_CONTROL);
  //   this->data(0x00);
  //   this->data(0x00);
  // }
}

void CrowPanelEPaper5P79In::display() {
  ESP_LOGD(TAG, "E-Paper display refresh starting");
  // Set the display mode based on update type
  UpdateMode mode = this->is_full_update_ ? UpdateMode::FULL : UpdateMode::PARTIAL;
  // this->prepare_for_update_(mode);
  // Reset RAM address counters before writing data
  // Primary controller
  // this->command(CMD_SET_X_COUNTER);
  // this->data(0x00);
  // this->command(CMD_SET_Y_COUNTER);
  // this->data(0x00);
  // this->data(0x00);
  // // Secondary controller
  // this->command(CMD_SET_X_COUNTER | CMD_TARGET_SECONDARY);
  // this->data(0x00);
  // this->command(CMD_SET_Y_COUNTER | CMD_TARGET_SECONDARY);
  // this->data(0x00);
  // this->data(0x00);
  // // Send command to write to BLACK/WHITE RAM
  // this->command(CMD_WRITE_RAM);
  // // Start non-blocking data transfer (handled in state machine)
  // this->start_data_();
  this->data_send_index_ = 0;
}

void CrowPanelEPaper5P79In::set_partial_ram_area_(uint8_t target, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
  ESP_LOGD(TAG, "Setting partial RAM area: target=%02X, x=%d, y=%d, width=%d, height=%d", 
           target, x, y, width, height);
  this->command(CMD_DATA_ENTRY_MODE | target);
  this->data((target != CMD_TARGET_PRIMARY) ? PARAM_X_DEC_Y_INC : PARAM_X_INC_Y_INC); // Set data entry mode to X+ Y+

  int x_end = x + width - 1;
  int y_end = y + height - 1;

  this->command(CMD_SET_X_ADDR | target);
  if (target != CMD_TARGET_PRIMARY) {
    this->data(x_end / 8u);
    this->data(x / 8u);
  } else {
    this->data(x / 8u);
    this->data(x_end / 8u);
  }

  this->command(CMD_SET_Y_ADDR | target);
  this->data(y & 0xFF); // Y start low byte
  this->data(y >> 8); // Y start high byte
  this->data(y_end & 0xFF); // Y end low byte
  this->data(y_end >> 8); // Y end high byte

  this->command(CMD_SET_X_COUNTER | target);
  if (target != CMD_TARGET_PRIMARY) {
    this->data(x_end / 8u); // Set X counter to end position
  } else {
    this->data(x / 8u); // Set X counter to start position
  }

  this->command(CMD_SET_Y_COUNTER | target);
  this->data(y & 0xFF); // Set Y counter to start position low byte
  this->data(y >> 8); // Set Y counter to start position high byte
}

void CrowPanelEPaper5P79In::write_data_from_buffer(uint16_t x_start, uint16_t y_start, uint16_t width, uint16_t height) {
  ESP_LOGD(TAG, "Writing data from buffer: x_start=%d, y_start=%d, width=%d, height=%d", x_start, y_start, width, height);
  this->start_data_();
  for (uint16_t y = y_start; y < (height + y_start); ++y) {
    for (uint16_t x = x_start; x < (width + x_start); x += 8u) {
      size_t index = (y * this->get_native_width_() + x) / 8u;
      assert(index < this->get_buffer_length_());

      this->write_byte_soft_spi(this->buffer_[index]);
      // if (this->buffer_[index] != 0xFF) {
      //   ESP_LOGD(TAG, "Writing byte %02X at index %u (x=%d, y=%d)", 
      //            this->buffer_[index], index, x, y);
      // }
    }
  }
  this->end_data_();
}

void CrowPanelEPaper5P79In::update_sending_data_(uint32_t now) {
  // Send a chunk of data per loop
  // const size_t CHUNK_SIZE = 32;

  uint16_t half_width = this->get_native_width_() / 2u;
  uint16_t half_height = this->get_native_height_() / 2u;

  // left half (primary controller)
  this->set_partial_ram_area_(CMD_TARGET_PRIMARY, 0, 0, half_width, this->get_native_height_());
  this->command(CMD_WRITE_RAM | CMD_TARGET_PRIMARY);
  this->write_data_from_buffer(0, 0, half_width, this->get_native_height_());
  // right half (secondary controller)
  this->set_partial_ram_area_(CMD_TARGET_SECONDARY, 0, 0, half_width, this->get_native_height_());
  this->command(CMD_WRITE_RAM | CMD_TARGET_SECONDARY);
  this->write_data_from_buffer(half_width, 0, half_width, this->get_native_height_());

  this->send_command_sequence_(fast_full_refresh_sequence);
  // this->send_command_sequence_(partial_refresh_sequence);
  this->state_ = EpdState::UPDATE_WAIT_REFRESH;
  this->state_start_time_ = now;

  // if (true) {
  //   // this->end_data_();
  //   // Send refresh command based on update mode
  //   UpdateMode mode = this->is_full_update_ ? UpdateMode::FULL : UpdateMode::PARTIAL;
  //   if (mode == UpdateMode::FULL) {
  //     this->send_command_sequence_(full_refresh_sequence);
  //   } else {
  //     this->send_command_sequence_(partial_refresh_sequence);
  //   }
  //   this->state_ = EpdState::UPDATE_WAIT_REFRESH;
  //   this->state_start_time_ = now;
  // }
}

void CrowPanelEPaper5P79In::deep_sleep() {
  ESP_LOGD(TAG, "Entering deep sleep mode");
  
  // Send deep sleep sequence
  this->send_command_sequence_(display_stop_sequence);
}

void CrowPanelEPaper5P79In::dump_config() {
  LOG_DISPLAY("", "CrowPanel E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 5.79in");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace crowpanel_epaper
}  // namespace esphome
