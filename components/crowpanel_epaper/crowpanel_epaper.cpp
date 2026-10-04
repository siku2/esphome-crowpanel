#include "crowpanel_epaper.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include <cmath>
#include <cstring>

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
static const uint8_t CMD_WRITE_RAM_PREVIOUS = 0x26;
static const uint8_t CMD_UPDATE_SEQUENCE = 0x22;
static const uint8_t CMD_SET_X_ADDR = 0x44;
static const uint8_t CMD_SET_Y_ADDR = 0x45;
static const uint8_t CMD_SET_X_COUNTER = 0x4E;
static const uint8_t CMD_SET_Y_COUNTER = 0x4F;
static const uint8_t CMD_SET_MUX = 0x01;
static const uint8_t CMD_TEMPERATURE_SENSOR = 0x18;
static const uint8_t CMD_WRITE_TEMPERATURE = 0x1A;

// Explicit target selection when the SSD1683 is used in cascade mode.
static const uint8_t CMD_TARGET_PRIMARY = 0x00;
static const uint8_t CMD_TARGET_SECONDARY = 0x80;

// SSD1683 EPD Driver chip command parameters
static const uint8_t PARAM_BORDER_FULL = 0x05;
static const uint8_t PARAM_BORDER_PARTIAL = 0x80;
static const uint8_t PARAM_FULL_UPDATE = 0xF7;
static const uint8_t PARAM_PARTIAL_UPDATE = 0xFF;
static const uint8_t PARAM_FULL_UPDATE_KEEP_TEMPERATURE = 0xD7;
static const uint8_t PARAM_PARTIAL_UPDATE_KEEP_TEMPERATURE = 0xDF;
static const uint8_t PARAM_TEMPERATURE_INTERNAL = 0x80;
static const int8_t FAST_FULL_UPDATE_TEMPERATURE = 100;
static const uint8_t PARAM_DEEP_SLEEP_MODE = 0x01;
static const uint8_t PARAM_X_INC_Y_INC = 0x03; // left-right, top-down
static const uint8_t PARAM_X_DEC_Y_INC = 0x02; // right-left, top-down
static const uint8_t PARAM_SEL_SINGLE_CHIP = 0x00;
static const uint8_t PARAM_SEL_CASCADE = 0x10;

// SSD1683 EPD Driver chip command sequences
// Format: command, num_args, arg1, arg2...
// Special case: if num_args has bit 7 set (0x80), it indicates a delay command
// End marker is two 0xFF bytes

const uint8_t display_start_sequence[] = {
  CMD_SOFT_RESET,                                                // Soft reset
  CMD_SET_MUX, 0x03, 0x2b, 0x01, 0x00,                           // Set MUX as 300
  CMD_DISPLAY_UPDATE_CONTROL, 0x02, 0x40, PARAM_SEL_SINGLE_CHIP, // Display update control
  CMD_BORDER_WAVEFORM, 0x01, PARAM_BORDER_FULL,                  // Border waveform for full refresh
  CMD_DATA_ENTRY_MODE, 0x01, PARAM_X_INC_Y_INC,                  // Data entry mode (X+ Y+)
  CMD_SET_X_ADDR, 0x02, 0x00, 0x31,                              // Set RAM X Address Start/End Pos (0 to 49 -> 400 pixels)
  CMD_SET_Y_ADDR, 0x04, 0x00, 0x00, 0x2b, 0x01,                  // Set RAM Y Address Start/End Pos (0 to 299 -> 300 pixels)
  CMD_SET_X_COUNTER, 0x01, 0x00,                                 // Set RAM X Address counter
  CMD_SET_Y_COUNTER, 0x02, 0x00, 0x00,                           // Set RAM Y Address counter
  COMMAND_END_MARKER, COMMAND_END_MARKER                         // End marker
};

const uint8_t display_start_sequence_5p79in[] = {
  CMD_SOFT_RESET, DELAY_FLAG, 10,                        // Soft reset and 10ms delay
  CMD_TEMPERATURE_SENSOR, 0x01, PARAM_TEMPERATURE_INTERNAL, // Use the internal temperature sensor
  // Do not set MUX. Not sure why, but it causes issues with the 5.79in display.
  // Set up the RAM area for the primary controller
  CMD_DATA_ENTRY_MODE | CMD_TARGET_PRIMARY, 0x01, PARAM_X_INC_Y_INC, // This panel goes from left to right.
  CMD_SET_X_ADDR | CMD_TARGET_PRIMARY, 0x02, 0x00, 0x31,             // Set RAM X Address Start/End Pos (0 to 49 -> 400 pixels)
  CMD_SET_Y_ADDR | CMD_TARGET_PRIMARY, 0x04, 0x00, 0x00, 0x0f, 0x01, // Set RAM Y Address Start/End Pos (0 to 271 -> 272 pixels)
  // Set up the RAM area for the secondary controller
  CMD_DATA_ENTRY_MODE | CMD_TARGET_SECONDARY, 0x01, PARAM_X_DEC_Y_INC, // This panel goes from right to left.
  CMD_SET_X_ADDR | CMD_TARGET_SECONDARY, 0x02, 0x31, 0x00,             // Set RAM X Address Start/End Pos (49 to 0 -> 400 pixels)
  CMD_SET_Y_ADDR | CMD_TARGET_SECONDARY, 0x04, 0x00, 0x00, 0x0f, 0x01, // Set RAM Y Address Start/End Pos (0 to 271 -> 272 pixels)
  COMMAND_END_MARKER, COMMAND_END_MARKER                 // End marker
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

const uint8_t full_refresh_keep_temperature_sequence[] = {
  CMD_UPDATE_SEQUENCE, 0x01, PARAM_FULL_UPDATE_KEEP_TEMPERATURE, // Display update sequence option (full, no temperature load)
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                // Master activation with 10ms delay
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

const uint8_t partial_refresh_sequence[] = {
  CMD_UPDATE_SEQUENCE, 0x01, PARAM_PARTIAL_UPDATE,   // Display update sequence option (partial)
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                // Master activation with 10ms delay
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

const uint8_t partial_refresh_keep_temperature_sequence[] = {
  CMD_UPDATE_SEQUENCE, 0x01, PARAM_PARTIAL_UPDATE_KEEP_TEMPERATURE, // Display update sequence option (partial, no temperature load)
  CMD_DISPLAY_UPDATE, DELAY_FLAG, 10,                // Master activation with 10ms delay
  COMMAND_END_MARKER, COMMAND_END_MARKER             // End marker
};

void CrowPanelEPaperBase::send_refresh_sequence_(bool full) {
  this->send_command_sequence_(full ? full_refresh_sequence : partial_refresh_sequence);
}

// ========================================================
// CrowPanelEPaperBase Implementation - SPI Communication
// ========================================================

void CrowPanelEPaperBase::setup_pins_() {
  this->dc_pin_->setup();
  this->dc_pin_->digital_write(true);

  if (this->reset_pin_ != nullptr)
    this->reset_pin_->setup();
  if (this->busy_pin_ != nullptr) {
    this->busy_pin_->pin_mode(gpio::FLAG_INPUT);
  }
}

void CrowPanelEPaperBase::command(uint8_t value) {
  this->dc_pin_->digital_write(false);
  this->enable();
  this->write_byte(value);
  this->disable();
  this->dc_pin_->digital_write(true);
}

void CrowPanelEPaperBase::data(uint8_t value) {
  this->dc_pin_->digital_write(true);
  this->enable();
  this->write_byte(value);
  this->disable();
}

void CrowPanelEPaperBase::write_data_(const uint8_t *data, size_t length) {
  this->dc_pin_->digital_write(true);
  this->enable();
  this->write_array(data, length);
  this->disable();
}

size_t CrowPanelEPaperBase::get_chunk_size_() {
  // Bytes that transfer in about 10 ms
  return std::max<size_t>(this->data_rate_ / 800u, 64u);
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
      // TODO: The delay should be handled by the state machine 
      uint8_t delay_ms = sequence[i++];
      delay(delay_ms);
      // i++;
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
  this->spi_setup();
  
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
        this->high_freq_.start();
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
      
      // Determine update mode: one-shot request, then forced mode, then cadence
      if (this->full_update_requested_) {
        this->is_full_update_ = true;
        this->full_update_requested_ = false;
      } else if (this->has_forced_update_mode_) {
        this->is_full_update_ = (this->force_update_mode_ == UpdateMode::FULL);
      } else {
        // Ensure the very first update is always full. A cadence of 0 disables automatic full updates.
        this->is_full_update_ = (this->update_count_ == 1 ||
                                 (this->full_update_every_ != 0 && this->update_count_ % this->full_update_every_ == 0));
      }
      
      ESP_LOGD(TAG, "Performing %s display update (%u)", 
                 this->is_full_update_ ? "FULL" : "PARTIAL", this->update_count_);

      if (this->auto_clear_enabled_) {
        // Clear buffer to white first
        this->fill(display::COLOR_OFF);
      }

      // Execute the lambda (if set) - this draws text, shapes, etc.
      if (this->page_ != nullptr) {
        this->page_->get_writer()(*this);
      } else if (this->writer_.has_value()) {
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
    case EpdState::UPDATE_SENDING_DATA:
      this->update_send_data_(now);
      break;
    case EpdState::UPDATE_REFRESH: {
      // Send refresh command based on update mode
      this->send_refresh_sequence_(this->is_full_update_);
      this->state_ = EpdState::UPDATE_WAIT_REFRESH;
      this->state_start_time_ = now;
      break;
    }
    case EpdState::UPDATE_WAIT_REFRESH:
      if (this->is_idle_() || now - this->state_start_time_ > this->idle_timeout_()) {
        if (this->has_post_refresh_sync_()) {
          this->state_ = EpdState::UPDATE_SYNC_PREPARE;
        } else {
          this->state_ = EpdState::UPDATE_DONE;
          ESP_LOGD(TAG, "Display update complete");
        }
        this->state_start_time_ = now;
      }
      break;
    case EpdState::UPDATE_SYNC_PREPARE:
      this->prepare_post_refresh_sync_();
      this->state_ = EpdState::UPDATE_SYNC_SENDING;
      this->state_start_time_ = now;
      break;
    case EpdState::UPDATE_SYNC_SENDING:
      this->update_send_data_(now);
      break;
      
    case EpdState::UPDATE_DONE:
      this->state_ = EpdState::IDLE;
      this->high_freq_.stop();
      break;
      
    case EpdState::DEEP_SLEEP:
      // Stay in deep sleep state
      break;
  }
}

void CrowPanelEPaperBase::update_send_data_(uint32_t now) {
  size_t buffer_len = this->get_buffer_length_();
  size_t i = this->data_send_index_;
  size_t end = std::min(i + this->get_chunk_size_(), buffer_len);
  this->write_data_(this->buffer_ + i, end - i);
  this->data_send_index_ = end;
  if (this->data_send_index_ >= buffer_len) {
    this->state_ = EpdState::UPDATE_REFRESH;
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
  this->high_freq_.stop();
  this->deep_sleep(); 
}

void CrowPanelEPaperBase::dump_config() {
    LOG_DISPLAY("", "CrowPanel E-Paper", this);
    LOG_SPI_DEVICE(this);
    LOG_PIN("  DC Pin: ", this->dc_pin_);
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
    LOG_PIN("  Busy Pin: ", this->busy_pin_);
    if (this->full_update_every_ == 0) {
      ESP_LOGCONFIG(TAG, "  Full Update Every: never (manual only)");
    } else {
      ESP_LOGCONFIG(TAG, "  Full Update Every: %u", this->full_update_every_);
    }
    
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
  const uint8_t fill = color.is_on() ? 0x00 : 0xFF;
  ESP_LOGD(TAG, "Filling buffer with %s", color.is_on() ? "BLACK" : "WHITE");
  
  if (this->get_buffer_length_() == 0 || this->buffer_ == nullptr) {
    ESP_LOGE(TAG, "ERROR: Buffer not initialized");
    return;
  }
  std::memset(this->buffer_, fill, this->get_buffer_length_());
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
    this->data(PARAM_SEL_SINGLE_CHIP);
  } else {
    ESP_LOGD(TAG, "Preparing for PARTIAL update mode");
    
    // Set BorderWavefrom for partial refresh
    this->command(CMD_BORDER_WAVEFORM);
    this->data(PARAM_BORDER_PARTIAL);
    
    // Additional settings for partial update
    this->command(CMD_DISPLAY_UPDATE_CONTROL);
    this->data(0x00);
    this->data(PARAM_SEL_SINGLE_CHIP);
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
  LOG_SPI_DEVICE(this);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

// ========================================================================
// CrowPanelEPaper5P79In Implementation (5.79" B/W display)
//
// This model uses the cascade mode of the SSD1683 to chain two controllers
// together. The left half (closest to the connector) is the primary
// controller.
// ========================================================================

void CrowPanelEPaper5P79In::initialize() {
  ESP_LOGD(TAG, "Initializing CrowPanel 5.79in display");

  // Send initialization sequence using the predefined commands
  this->send_command_sequence_(display_start_sequence_5p79in);
}

void CrowPanelEPaper5P79In::prepare_for_update_(UpdateMode mode) {
  if (mode == UpdateMode::FULL) {
    ESP_LOGD(TAG, "Preparing for FULL update mode");
    
    // Set BorderWavefrom for full refresh
    this->command(CMD_BORDER_WAVEFORM);
    this->data(PARAM_BORDER_FULL);
    
    // Additional display update control settings 
    this->command(CMD_DISPLAY_UPDATE_CONTROL);
    this->data(0x40);
    this->data(PARAM_SEL_CASCADE);
  } else {
    ESP_LOGD(TAG, "Preparing for PARTIAL update mode");
    
    // Set BorderWavefrom for partial refresh
    this->command(CMD_BORDER_WAVEFORM);
    this->data(PARAM_BORDER_PARTIAL);
    
    // Additional settings for partial update
    this->command(CMD_DISPLAY_UPDATE_CONTROL);
    this->data(0x00);
    this->data(PARAM_SEL_CASCADE);
  }
}

static const RamPass full_ram_passes[] = {
  {CMD_WRITE_RAM_PREVIOUS, EpdCascadeState::PRIMARY},
  {CMD_WRITE_RAM_PREVIOUS, EpdCascadeState::SECONDARY},
  {CMD_WRITE_RAM, EpdCascadeState::PRIMARY},
  {CMD_WRITE_RAM, EpdCascadeState::SECONDARY},
};

static const RamPass partial_ram_passes[] = {
  {CMD_WRITE_RAM, EpdCascadeState::PRIMARY},
  {CMD_WRITE_RAM, EpdCascadeState::SECONDARY},
};

static const RamPass sync_ram_passes[] = {
  {CMD_WRITE_RAM_PREVIOUS, EpdCascadeState::PRIMARY},
  {CMD_WRITE_RAM_PREVIOUS, EpdCascadeState::SECONDARY},
  {CMD_WRITE_RAM, EpdCascadeState::PRIMARY},
  {CMD_WRITE_RAM, EpdCascadeState::SECONDARY},
};

void CrowPanelEPaper5P79In::setup() {
  CrowPanelEPaperBase::setup();
  if (this->buffer_ == nullptr) {
    this->mark_failed();
    return;
  }
  RAMAllocator<uint8_t> allocator;
  this->snapshot_ = allocator.allocate(this->get_buffer_length_());
  if (this->snapshot_ == nullptr) {
    ESP_LOGE(TAG, "Could not allocate snapshot buffer for display!");
    this->mark_failed();
  }
}

void CrowPanelEPaper5P79In::start_pass_() {
  const RamPass &pass = this->passes_[this->pass_index_];
  // Reset the RAM address counters of the controller that receives this pass
  if (pass.target == EpdCascadeState::PRIMARY) {
    // Primary controller (start from top-left)
    this->command(CMD_SET_X_COUNTER | CMD_TARGET_PRIMARY);
    this->data(0x00);
    this->command(CMD_SET_Y_COUNTER | CMD_TARGET_PRIMARY);
  } else {
    // Secondary controller (start from top-right)
    this->command(CMD_SET_X_COUNTER | CMD_TARGET_SECONDARY);
    this->data(0x31); // 49b -> 400px
    this->command(CMD_SET_Y_COUNTER | CMD_TARGET_SECONDARY);
  }
  this->data(0x00);
  this->data(0x00);

  this->cascade_state_ = pass.target;
  this->data_send_index_ = 0;
  this->command(pass.command | (pass.target == EpdCascadeState::PRIMARY ? CMD_TARGET_PRIMARY : CMD_TARGET_SECONDARY));
}

void CrowPanelEPaper5P79In::display() {
  ESP_LOGD(TAG, "E-Paper display refresh starting");
  // Set the display mode based on update type
  UpdateMode mode = this->is_full_update_ ? UpdateMode::FULL : UpdateMode::PARTIAL;
  this->prepare_for_update_(mode);

  // The buffer keeps changing while the transfer runs, so every RAM write sends this copy.
  std::memcpy(this->snapshot_, this->buffer_, this->get_buffer_length_());

  if (mode == UpdateMode::FULL) {
    this->passes_ = full_ram_passes;
    this->pass_count_ = sizeof(full_ram_passes) / sizeof(full_ram_passes[0]);
  } else {
    this->passes_ = partial_ram_passes;
    this->pass_count_ = sizeof(partial_ram_passes) / sizeof(partial_ram_passes[0]);
  }
  this->pass_index_ = 0;
  this->start_pass_();
}

void CrowPanelEPaper5P79In::prepare_post_refresh_sync_() {
  this->passes_ = sync_ram_passes;
  this->pass_count_ = sizeof(sync_ram_passes) / sizeof(sync_ram_passes[0]);
  this->pass_index_ = 0;
  this->start_pass_();
}

void CrowPanelEPaper5P79In::update_send_data_(uint32_t now) {
  constexpr uint16_t width_bytes = NATIVE_WIDTH_5P79IN / 8u;
  // It's important to round up here!
  constexpr uint16_t x_offset_end = (width_bytes + 1u) / 2u;
  // And here it's important to round down.
  constexpr uint16_t x_offset_start = width_bytes / 2u;
  constexpr size_t max_rows = 64u;

  // The logic here is slightly more complex than for the 4.2in display because we have to deal
  // with two controllers, each with its own buffer. Worse, they even have an overlap in the middle.
  // Luckily for us, we can just write the 8-bit overlap data to both controllers and it will work
  // fine. That's why the rounding is important above.
  //
  // The buffer's layout would force us to switch controllers right in the middle of a row.
  // Instead we first write the left half of every row to the primary controller, then switch to
  // the secondary controller and write the right half of every row. Each half row is contiguous,
  // so we gather several of them into a stack buffer and send them with a single transfer.

  // For the secondary controller, read from the right half of the buffer.
  const uint16_t x_start = (this->cascade_state_ == EpdCascadeState::PRIMARY) ? 0 : x_offset_start;
  const size_t height = this->get_native_height_();
  const size_t rows_per_pass = std::min(std::max<size_t>(this->get_chunk_size_() / x_offset_end, 1u), max_rows);

  uint8_t rows[max_rows * x_offset_end];
  size_t row = this->data_send_index_;
  const size_t end_row = std::min(row + rows_per_pass, height);
  uint8_t *out = rows;
  for (; row < end_row; ++row) {
    size_t index = row * width_bytes + x_start;
    assert(index + x_offset_end <= this->get_buffer_length_());
    std::memcpy(out, this->snapshot_ + index, x_offset_end);
    out += x_offset_end;
  }
  this->write_data_(rows, out - rows);
  this->data_send_index_ = end_row;

  // Still writing data...
  if (this->data_send_index_ < height) return;

  // The current transfer is done.
  if (++this->pass_index_ < this->pass_count_) {
    this->start_pass_();
    return;
  }

  // All RAM passes are done.
  if (this->state_ == EpdState::UPDATE_SYNC_SENDING) {
    this->state_ = EpdState::UPDATE_DONE;
    ESP_LOGD(TAG, "Display update complete");
  } else {
    this->state_ = EpdState::UPDATE_REFRESH;
  }
  this->state_start_time_ = now;
}

void CrowPanelEPaper5P79In::deep_sleep() {
  ESP_LOGD(TAG, "Entering deep sleep mode");
  
  // Send deep sleep sequence
  this->send_command_sequence_(display_stop_sequence);
}

void CrowPanelEPaper5P79In::dump_config() {
  LOG_DISPLAY("", "CrowPanel E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: 5.79in");
  LOG_SPI_DEVICE(this);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  ESP_LOGCONFIG(TAG, "  Fast Full Update: %s", YESNO(this->fast_full_update_));
#ifdef USE_SENSOR
  if (this->temperature_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Temperature Sensor", this->temperature_sensor_);
  } else
#endif
  {
    ESP_LOGCONFIG(TAG, "  Temperature Source: internal");
  }
  LOG_UPDATE_INTERVAL(this);
}

void CrowPanelEPaper5P79In::write_temperature_(int8_t celsius) {
  this->command(CMD_WRITE_TEMPERATURE);
  this->data(static_cast<uint8_t>(celsius));
  this->data(0x00);
}

void CrowPanelEPaper5P79In::send_refresh_sequence_(bool full) {
  if (full && this->fast_full_update_) {
    this->write_temperature_(FAST_FULL_UPDATE_TEMPERATURE);
    this->send_command_sequence_(full_refresh_keep_temperature_sequence);
    return;
  }
#ifdef USE_SENSOR
  if (this->temperature_sensor_ != nullptr && this->temperature_sensor_->has_state() &&
      !std::isnan(this->temperature_sensor_->state)) {
    float celsius = std::min(std::max(std::round(this->temperature_sensor_->state), -128.0f), 127.0f);
    this->write_temperature_(static_cast<int8_t>(celsius));
    this->send_command_sequence_(full ? full_refresh_keep_temperature_sequence
                                      : partial_refresh_keep_temperature_sequence);
    return;
  }
#endif
  CrowPanelEPaperBase::send_refresh_sequence_(full);
}

}  // namespace crowpanel_epaper
}  // namespace esphome
