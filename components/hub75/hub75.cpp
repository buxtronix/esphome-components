#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "hub75.h"

#ifdef USE_ESP8266
#include <core_esp8266_waveform.h>
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include <esp32-hal-timer.h>
#define _BV(bit) (1 << (bit))
#endif

#define ONE_MUX_PER_INTERRUPT

#ifdef USE_ESP8266
// setup a new mutex
void ICACHE_FLASH_ATTR CreateMutux(mutex_t *mutex) {
	*mutex = 1;
}

// try to get a mutex
// returns true if successful, false if mutex not free
// as the esp8266 doesn't support the atomic S32C1I instruction
// we have to make the code uninterruptable to produce the
// same overall effect
bool ICACHE_FLASH_ATTR GetMutex(mutex_t *mutex) {

	int iOld = 1, iNew = 0;

	asm volatile (
		"rsil a15, 1\n"    // read and set interrupt level to 1
		"l32i %0, %1, 0\n" // load value of mutex
		"bne %0, %2, 1f\n" // compare with iOld, branch if not equal
		"s32i %3, %1, 0\n" // store iNew in mutex
		"1:\n"             // branch target
		"wsr.ps a15\n"     // restore program state
		"rsync\n"
		: "=&r" (iOld)
		: "r" (mutex), "r" (iOld), "r" (iNew)
		: "a15", "memory"
	);

	return (bool)iOld;
}

// release a mutex
void ICACHE_FLASH_ATTR ReleaseMutex(mutex_t *mutex) {
	*mutex = 1;
}

#endif

namespace esphome {
namespace hub75 {

static const char *TAG = "Hub75";

static Hub75 *global_panel;

// How often to perform a display update (in Hz).
static const int DISPLAY_UPDATE_INTERVAL = 1000;

#ifdef USE_ESP32
// Spinlock for interrupt handler below.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#endif

// Interrupt handler which is called every ~ms to push data to the display.
static uint32_t IRAM_ATTR HOT timer_interrupt() {
  if (global_panel != nullptr) {
#ifdef USE_ESP32
    portENTER_CRITICAL_ISR(&timerMux);
#endif
    global_panel->timer_intr(0);
#ifdef USE_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);
#endif
  }
  return F_CPU/DISPLAY_UPDATE_INTERVAL;
}

#ifdef USE_ESP32
// Timer which generates the above interrupts.
static hw_timer_t *row_timer = nullptr;
void IRAM_ATTR HOT s_timer_intr() { timer_interrupt(); }
#endif

static uint8_t setup_done;

Hub75::Hub75(int rowscan, ScanPattern scanpattern, int width, int height,        
           MuxPattern muxpattern, BlockPattern blockpattern, ColorOrder colororder,
           int panels_wide)
  : rowscan_(rowscan),
    scanpattern_(scanpattern),
    width_(width),
    height_(height),
    muxpattern_(muxpattern),
    blockpattern_(blockpattern),
    colororder_(colororder),
    panels_width_(panels_wide) {}

void Hub75::setup() {
  this->spi_setup();
  ESP_LOGCONFIG(TAG, "Setting up Hub75 LED Matrix");

  this->pin_en_->setup();
  this->pin_stb_->setup();

  this->pin_a_->setup();
  this->pin_b_->setup();
  if (this->pin_c_ != nullptr) this->pin_c_->setup();
  if (this->pin_d_ != nullptr) this->pin_d_->setup();
  if (this->pin_e_ != nullptr) this->pin_e_->setup();

  this->pin_a_->digital_write(false);
  this->pin_b_->digital_write(false);
  this->pin_en_->digital_write(true);

  if (this->rowscan_ >= 8) this->pin_c_->digital_write(false);
  if (this->rowscan_ >= 16) this->pin_d_->digital_write(false);
  if (this->rowscan_ >= 32) this->pin_e_->digital_write(false);

  this->display_color_ = 0;
  this->buffer_size_ = (this->width_ * this->height_ * 3 / 8);
  this->panel_width_bytes_ = (this->width_ / this->panels_width_)/8;
  this->rows_per_buffer_ = this->height_ / 2;
  this->pattern_color_bytes_ = (this->height_/this->rowscan_)*(this->width_/8);
  this->row_sets_per_buffer_ = this->rows_per_buffer_/this->rowscan_;
  this->send_buffer_size_ = this->pattern_color_bytes_*3;

  this->matrix_buffer_ = new uint8_t[COLOR_DEPTH*this->buffer_size_];
  memset(this->matrix_buffer_, 0, COLOR_DEPTH*this->buffer_size_);
  this->matrix_buffer2_ = new uint8_t[COLOR_DEPTH*this->buffer_size_];
  memset(this->matrix_buffer2_, 0, COLOR_DEPTH*this->buffer_size_);
  this->active_buffer_ = false;
  this->enabled_ = false;


  // Precompute row offset values
  this->row_offset_=new uint32_t[this->height_];
  for (uint8_t yy=0; yy<this->height_;yy++)
      this->row_offset_[yy]=((yy)%this->rowscan_)*this->send_buffer_size_+this->send_buffer_size_-1;

  global_panel = this;

#ifdef USE_ESP8266
  CreateMutux(&this->mutex);
#endif
#ifdef USE_ESP32
  row_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(row_timer, s_timer_intr, true);
#ifdef ONE_MUX_PER_INTERRUPT
  timerAlarmWrite(row_timer, DISPLAY_UPDATE_INTERVAL/this->rowscan_, true);
#else
  timerAlarmWrite(row_timer, DISPLAY_UPDATE_INTERVAL, true);
#endif // ONE_MUX_PER_INTERRUPT
#endif // USE_ESP32
}

void Hub75::dump_config(){
    LOG_DISPLAY("", "Hub75", this);
    LOG_PIN(" Pin EN: ", this->pin_en_);
    LOG_PIN(" Pin STB: ", this->pin_stb_);
    LOG_PIN(" Pin  A: ", this->pin_a_);
    LOG_PIN(" Pin  B: ", this->pin_b_);
    if (this->pin_c_ != nullptr) LOG_PIN(" Pin  C: ", this->pin_c_);
    if (this->pin_d_ != nullptr) LOG_PIN(" Pin  D: ", this->pin_d_);
    if (this->pin_e_ != nullptr) LOG_PIN(" Pin  E: ", this->pin_e_);
    ESP_LOGD(TAG, " Width: %d", this->width_);
    ESP_LOGD(TAG, " Height: %d", this->height_);
    LOG_UPDATE_INTERVAL(this);

}

void Hub75::update() {

#ifdef USE_ESP8266
  GetMutex(&this->mutex);
#endif
  this->do_update_();
#ifdef USE_ESP32
  portENTER_CRITICAL(&timerMux);
#endif // USE_ESP32
  this->active_buffer_ = !this->active_buffer_; // Swap buffer.
#ifdef USE_ESP32
  portEXIT_CRITICAL(&timerMux);
#endif // USE_ESP32
#ifdef USE_ESP8266
  ReleaseMutex(&this->mutex);
#endif
}

void Hub75::panel_enable(bool enable) {
  if (enable) {
    this->enabled_ = true;
    this->mux_current_ = 0;
#ifdef USE_ESP32
    timerAlarmEnable(row_timer);
#endif // USE_ESP32
#ifdef USE_ESP8266
  setTimer1Callback(&timer_interrupt);
#endif // USE_ESP8266
  } else {
    this->enabled_ = false;
#ifdef USE_ESP32
    timerAlarmDisable(row_timer);
#endif // USE_ESP32
#ifdef USE_ESP8266
  setTimer1Callback(NULL);
#endif // USE_ESP8266
    this->pin_en_->digital_write(true);
  }
}

void IRAM_ATTR HOT Hub75::timer_intr(uint32_t now) {
#ifdef USE_ESP8266
  GetMutex(&this->mutex);
#endif
  if (this->enabled_) {
    this->display(40);
  }
#ifdef USE_ESP8266
  ReleaseMutex(&this->mutex);
#endif
}

void Hub75::fillMatrixBuffer(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
  //TODO: Support colour offset.

  if (this->blockpattern_ == BLOCKPATTERN_DBCA) {
    if (this->panels_width_>1) {
      x += this->width_/4;
    } else if ((x > this->width_/2) && (x < this->width_*3/4)) {
      x -= this->width_/4;
    }

    uint16_t y_block = y*4/this->height_;
    uint16_t x_block = x*2*this->panels_width_/this->height_;

    if (!(y_block%2)) {
      if (!(x_block%2)) {
        x += this->width_/2/this->panels_width_;
        y += this->height_/4;
      }
    } else {
      if (!(x_block%2)) {
        x -= this->width_/2/this->panels_width_;
        y -= this->height_/4;
      }
    }
  }

  // Panels are naturally flipped.
  x = this->width_ - 1 - x;

  if ((x < 0) || (x >= this->width_) || (y < 0) || (y >= this->height_))
    return; 

  if (this->colororder_ != COLORORDER_RRGGBB) {
    uint8_t r_temp = r;
    uint8_t g_temp = g;
    uint8_t b_temp = b;

    switch (this->colororder_) {
      case (COLORORDER_RRGGBB): break;
      case (COLORORDER_RRBBGG): g=b_temp; b=g_temp; break;
      case (COLORORDER_GGRRBB): r=g_temp; g=r_temp; break;
      case (COLORORDER_GGBBRR): r=g_temp; g=b_temp; b=r_temp; break;
      case (COLORORDER_BBRRGG): r=b_temp; g=r_temp; b=g_temp; break;
      case (COLORORDER_BBGGRR): r=b_temp; g=g_temp; b=r_temp; break;
    }
  }

  uint32_t base_offset;
  uint32_t total_offset_r = 0;
  uint32_t total_offset_g = 0;
  uint32_t total_offset_b = 0;

  if (this->scanpattern_==SCANPATTERN_WZAGZIG || this->scanpattern_==SCANPATTERN_VZAG || this->scanpattern_==SCANPATTERN_WZAGZIG2) {

    uint8_t rows_per_buffer = this->height_/2;
    uint8_t rows_per_block = rows_per_buffer/2;

    uint8_t cols_per_block = 16;
    uint8_t panel_width = this->width_/this->panels_width_;
    uint8_t blocks_x_per_panel = panel_width/cols_per_block;
    uint8_t panel_index = x/panel_width;

    x = x%panel_width;
    uint8_t base_y_offset = y/rows_per_buffer;
    uint8_t buffer_y = y%rows_per_buffer;
    uint8_t block_x = x/cols_per_block;
    uint8_t block_x_mod = x%cols_per_block;
    uint8_t block_y = buffer_y/rows_per_block;
    uint8_t block_y_mod = buffer_y%rows_per_block;

    uint8_t block_y_inv = 1 - block_y;
    uint8_t block_x_inv = blocks_x_per_panel - block_x - 1;
    uint8_t block_linear_index = 0;
    if (this->scanpattern_ == SCANPATTERN_WZAGZIG2) {
      block_linear_index = block_x_inv * 2 + block_y;
    } else if (this->scanpattern_ == SCANPATTERN_WZAGZIG) {
      block_linear_index = block_x_inv * 2 + block_y_inv;
    } else if (this->scanpattern_ == SCANPATTERN_VZAG) {
      block_linear_index = block_x_inv * 3 * block_y + block_y_inv * (block_x_inv + 1);
    }
    uint8_t new_block_x = block_linear_index % blocks_x_per_panel;
    uint8_t new_block_y = 1 - block_linear_index/blocks_x_per_panel;
    x = new_block_x * cols_per_block + block_x_mod + panel_index * panel_width;
    y = new_block_y * rows_per_block + block_y_mod + base_y_offset * rows_per_buffer;
  }
  if (this->scanpattern_!= SCANPATTERN_LINE && this->scanpattern_ != SCANPATTERN_WZAGZIG && this->scanpattern_ != SCANPATTERN_VZAG && this->scanpattern_ != SCANPATTERN_WZAGZIG2) {
    // Precomputed row offset values
    base_offset=this->row_offset_[y]-(x/8)*2;
    uint8_t row_sector=0;
    uint16_t row_sector_offset=this->width_/4;
    for (uint8_t yy = 0; yy<this->height_; yy+=2*this->rowscan_)
    {
      if ((yy<=y) && (y<yy+this->rowscan_))
        total_offset_r=base_offset-row_sector_offset*row_sector;
      if ((yy+this->rowscan_<=y) && (y<yy+2*this->rowscan_))
        total_offset_r=base_offset-row_sector_offset*row_sector;

      row_sector++;
    }
  } else {
    // can only be non-zero when _height/(2 inputs per panel)/_row_pattern > 1
    // i.e.: 32x32 panel with 1/8 scan (A/B/C lines) -> 32/2/8 = 2
    uint8_t vert_index_in_buffer = (y%this->rows_per_buffer_)/this->rowscan_; // which set of rows per buffer

    // can only ever be 0/1 since there are only ever 2 separate input sets present for this variety of panels (R1G1B1/R2G2B2)
    uint8_t which_buffer = y/this->rows_per_buffer_;
    uint8_t x_byte = x/8;
    // assumes panels are only ever chained for more width
    uint8_t which_panel = x_byte/this->panel_width_bytes_;
    uint8_t in_row_byte_offset = x_byte%this->panel_width_bytes_;
    // this could be pretty easily extended to vertical stacking as well
    total_offset_r = this->row_offset_[y] - in_row_byte_offset - this->panel_width_bytes_*(this->row_sets_per_buffer_*(this->panels_width_*which_buffer + which_panel) + vert_index_in_buffer);
  }

  uint8_t bit_select = x%8;

  if ((y % (this->rowscan_*2)) < this->rowscan_) {
    if (this->scanpattern_ == SCANPATTERN_ZAGGIZ) {
      total_offset_r--;
      bit_select = 7-bit_select;
    }
    if (this->scanpattern_ == SCANPATTERN_ZAGZIG) {
      total_offset_r--;
    }
    if (this->scanpattern_ == SCANPATTERN_ZZIAGG) {
      if (bit_select>3)
        bit_select += 4;
      else
        total_offset_r--;
    }
    if (this->scanpattern_ == SCANPATTERN_ZZAGG) {
      if (bit_select>3) total_offset_r--;
    }
  } else {
    if (this->scanpattern_ == SCANPATTERN_ZIGZAG) {
      total_offset_r--;
    }
    if (this->scanpattern_ == SCANPATTERN_ZZIAGG) {
      if (bit_select > 3) {
        total_offset_r--;
        bit_select -= 4;
      }
    }
    if (this->scanpattern_ == SCANPATTERN_ZZAGG) {
      if (bit_select <= 3) {
        bit_select += 4;
      } else {
        bit_select -= 4;
        total_offset_r--;
      }
    }
  }

  total_offset_g=total_offset_r-this->pattern_color_bytes_;
  total_offset_b=total_offset_g-this->pattern_color_bytes_;
 
  uint8_t *matrix_bufferp = this->active_buffer_ ? this->matrix_buffer_ : this->matrix_buffer2_;

  r = r >> (8-COLOR_DEPTH);
  g = g >> (8-COLOR_DEPTH);
  b = b >> (8-COLOR_DEPTH);

  //Color interlacing
  for (int this_color_bit=0; this_color_bit<COLOR_DEPTH; this_color_bit++)
  {
    if ((r >> this_color_bit) & 0x01)
      matrix_bufferp[this_color_bit*this->buffer_size_+total_offset_r] |=_BV(bit_select);
    else
      matrix_bufferp[this_color_bit*this->buffer_size_+total_offset_r] &= ~_BV(bit_select);

    if ((g >> this_color_bit) & 0x01)
      matrix_bufferp[this_color_bit*this->buffer_size_+total_offset_g] |=_BV(bit_select);
    else
      matrix_bufferp[this_color_bit*this->buffer_size_+total_offset_g] &= ~_BV(bit_select);

    if ((b >> this_color_bit) & 0x01)
      matrix_bufferp[this_color_bit*this->buffer_size_+total_offset_b] |=_BV(bit_select);
    else
      matrix_bufferp[this_color_bit*this->buffer_size_+total_offset_b] &= ~_BV(bit_select);
  }
}

void Hub75::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || x < 0 || y >= this->get_height_internal() || y < 0)
    return;
  fillMatrixBuffer(x, y, color.r, color.g, color.b);
}

void IRAM_ATTR HOT Hub75::display(uint16_t showtime) {
// How long do we keep the pixels on
  uint16_t latch_time = ((showtime*(1<<this->display_color_)*this->brightness_)/255/2);

  unsigned long start_time = 0;

  uint8_t *matrix_bufferp = this->active_buffer_ ? this->matrix_buffer2_ : this->matrix_buffer_;

  this->enable();

#ifndef ONE_MUX_PER_INTERRUPT
  for (this->mux_current_ = 0; this->mux_current_ < this->rowscan_; this->mux_current_++)
#endif
  {
    this->set_mux(this->mux_current_);
    if (this->fastupdate_) {
      this->pin_stb_->digital_write(true);
      this->pin_stb_->digital_write(false);
      this->pin_en_->digital_write(false);
      start_time = micros();
      delayMicroseconds(1);

      if (this->mux_current_ < this->rowscan_-1) {
       this->write_array(
          &matrix_bufferp[this->display_color_*this->buffer_size_+(this->mux_current_+1)*this->send_buffer_size_],
          this->send_buffer_size_);
      } else {
       this->write_array(
          &matrix_bufferp[((this->display_color_+1)%COLOR_DEPTH)*this->buffer_size_],
          this->send_buffer_size_);
      }

      while((micros()-start_time)<latch_time)
        delayMicroseconds(2);

      this->pin_en_->digital_write(true);
    } else {
      this->write_array(&matrix_bufferp[this->display_color_*this->buffer_size_+this->mux_current_*this->send_buffer_size_],this->send_buffer_size_);
      latch(latch_time);
    }
  }

#ifdef ONE_MUX_PER_INTERRUPT
  if (++this->mux_current_ >= this->rowscan_) {
    this->mux_current_ = 0;
#endif
  if (++this->display_color_ >= COLOR_DEPTH)
    this->display_color_ = 0;
#ifdef ONE_MUX_PER_INTERRUPT
  }
#endif
  this->disable();
}

void IRAM_ATTR HOT Hub75::set_mux(uint8_t value, bool random_access) {
  if (this->muxpattern_ == MUXPATTERN_BINARY) {
     // TODO: implement delay.
     this->pin_a_->digital_write(value & 0x01);
     this->pin_b_->digital_write(value & 0x02);
     if (this->scanpattern_ >= 8)  this->pin_c_->digital_write(value & 0x04);
     if (this->scanpattern_ >= 16) this->pin_d_->digital_write(value & 0x08);
     if (this->scanpattern_ >= 32) this->pin_e_->digital_write(value & 0x10);
  }
  if (this->muxpattern_ == MUXPATTERN_STRAIGHT) {
    this->pin_a_->digital_write(value == 0);
    this->pin_b_->digital_write(value == 1);
    this->pin_c_->digital_write(value == 2);
    this->pin_d_->digital_write(value == 3);
  }
  // TODO: Implement shiftreg muxing.
}

void IRAM_ATTR HOT Hub75::latch(uint16_t showtime) {
  // TODO: Implement non-shift driver.
  this->pin_stb_->digital_write(true);
  this->pin_stb_->digital_write(false);
  this->pin_en_->digital_write(false);
  delayMicroseconds(showtime);
  this->pin_en_->digital_write(true);
}

}  // namespace hub75
}  // namespace esphome
