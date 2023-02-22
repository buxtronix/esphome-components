#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/display/display_buffer.h"

#ifndef COLOR_DEPTH
#define COLOR_DEPTH 4
#endif

#if COLOR_DEPTH > 8 || COLOR_DEPTH < 1
#error "COLOR_DEPTH must be between 1 and 8 bits"
#endif

#ifdef USE_ESP8266
#include <c_types.h>
typedef int32 mutex_t;

void ICACHE_FLASH_ATTR CreateMutux(mutex_t *mutex);
bool ICACHE_FLASH_ATTR GetMutex(mutex_t *mutex);
void ICACHE_FLASH_ATTR ReleaseMutex(mutex_t *mutex);
#endif

namespace esphome {
namespace hub75 {


static const uint8_t SCAN_PATTERN_LINE = 0x1;
static const uint8_t SCAN_PATTERN_ZIGZAG = 0x2;
static const uint8_t SCAN_PATTERN_ZZAGG = 0x3;
static const uint8_t SCAN_PATTERN_ZAGGIZ = 0x4;
static const uint8_t SCAN_PATTERN_WZAGZIG = 0x5;
static const uint8_t SCAN_PATTERN_VZAG = 0x6;
static const uint8_t SCAN_PATTERN_ZAGZIG = 0x7;
static const uint8_t SCAN_PATTERN_WZAGZIG2 = 0x8;
static const uint8_t SCAN_PATTERN_ZZIAGG = 0x9;

enum ScanPattern {
  SCANPATTERN_LINE = SCAN_PATTERN_LINE,
  SCANPATTERN_ZIGZAG = SCAN_PATTERN_ZIGZAG,
  SCANPATTERN_ZZAGG = SCAN_PATTERN_ZZAGG,
  SCANPATTERN_ZAGGIZ = SCAN_PATTERN_ZAGGIZ,
  SCANPATTERN_WZAGZIG = SCAN_PATTERN_WZAGZIG,
  SCANPATTERN_VZAG = SCAN_PATTERN_VZAG,
  SCANPATTERN_ZAGZIG = SCAN_PATTERN_ZAGZIG,
  SCANPATTERN_WZAGZIG2 = SCAN_PATTERN_WZAGZIG2,
  SCANPATTERN_ZZIAGG = SCAN_PATTERN_ZZIAGG,
};

static const uint8_t MUX_PATTERN_BINARY = 0x1;
static const uint8_t MUX_PATTERN_STRAIGHT = 0x2;
static const uint8_t MUX_PATTERN_SHIFTREG_ABC = 0x3;
static const uint8_t MUX_PATTERN_SHIFTREG_SPI_SE = 0x4;

enum MuxPattern {
  MUXPATTERN_BINARY = MUX_PATTERN_BINARY,
  MUXPATTERN_STRAIGHT = MUX_PATTERN_STRAIGHT,
  MUXPATTERN_SHIFTREG_ABC = MUX_PATTERN_SHIFTREG_ABC,
  MUXPATTERN_SHIFTREG_SPI_SE = MUX_PATTERN_SHIFTREG_SPI_SE,
};

static const uint8_t BLOCK_PATTERN_ABCD = 0x1;
static const uint8_t BLOCK_PATTERN_DBCA = 0x2;

enum BlockPattern {
  BLOCKPATTERN_ABCD = BLOCK_PATTERN_ABCD,
  BLOCKPATTERN_DBCA = BLOCK_PATTERN_DBCA,
};

static const uint8_t COLOR_ORDER_RRGGBB = 0x1;
static const uint8_t COLOR_ORDER_RRBBGG = 0x2;
static const uint8_t COLOR_ORDER_GGRRBB = 0x3;
static const uint8_t COLOR_ORDER_GGBBRR = 0x4;
static const uint8_t COLOR_ORDER_BBRRGG = 0x5;
static const uint8_t COLOR_ORDER_BBGGRR = 0x6;

enum ColorOrder {
  COLORORDER_RRGGBB = COLOR_ORDER_RRGGBB,
  COLORORDER_RRBBGG = COLOR_ORDER_RRBBGG,
  COLORORDER_GGRRBB = COLOR_ORDER_GGRRBB,
  COLORORDER_GGBBRR = COLOR_ORDER_GGBBRR,
  COLORORDER_BBRRGG = COLOR_ORDER_BBRRGG,
  COLORORDER_BBGGRR = COLOR_ORDER_BBGGRR,
};

class Hub75 : public PollingComponent,
 		public display::DisplayBuffer,
                public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_8MHZ>  {
 public:

  Hub75(int row_scan, ScanPattern scanpattern, int width, int height,
           MuxPattern muxpattern, BlockPattern blockpattern, ColorOrder colororder,
           int panels_wide);
  void setup() override;
  void dump_config() override;
  void update() override;

  void set_pin_a(GPIOPin *p) { pin_a_ = p;}
  void set_pin_b(GPIOPin *p) { pin_b_ = p;}
  void set_pin_c(GPIOPin *p) { pin_c_ = p;}
  void set_pin_d(GPIOPin *p) { pin_d_ = p;}
  void set_pin_e(GPIOPin *p) { pin_e_ = p;}
  void set_ctrl_pins(GPIOPin *stb, GPIOPin *en) { pin_stb_ = stb; pin_en_ = en; }

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_COLOR; }

  void draw_absolute_pixel_internal(int x, int y, Color color) override;
  int get_width_internal() override { return this->width_; }
  int get_height_internal() override { return this->height_; }

  void timer_intr(uint32_t now);

  void panel_enable(bool);
  void set_brightness(uint8_t brightness) { this->brightness_ = brightness*255/100; }
  void set_fastupdate(bool fastupdate) { this->fastupdate_ = fastupdate; }

 protected:
  void set_mux(uint8_t value, bool random_access = false);
  void latch(uint16_t);

  void fillMatrixBuffer(int16_t x, int16_t y,
                        uint8_t r, uint8_t g, uint8_t b);
  void display(uint16_t showtime);

  uint8_t *matrix_buffer_;
  uint8_t *matrix_buffer2_;
  // The currently displayed buffer. If true, it's buffer2.
  bool active_buffer_;
  uint8_t panel_width_bytes_;
  uint8_t rows_per_buffer_;
  uint8_t row_sets_per_buffer_;
  uint8_t pattern_color_bytes_;
  uint32_t *row_offset_;
  int mux_current_;
  bool fastupdate_;

#ifdef USE_ESP8266
  mutex_t mutex;
#endif

  int rowscan_;
  ScanPattern scanpattern_;
  uint16_t width_;
  uint16_t height_;
  MuxPattern muxpattern_;
  BlockPattern blockpattern_;
  ColorOrder colororder_;
  uint8_t panels_width_;

  GPIOPin *pin_a_{nullptr};
  GPIOPin *pin_b_{nullptr};
  GPIOPin *pin_c_{nullptr};
  GPIOPin *pin_d_{nullptr};
  GPIOPin *pin_e_{nullptr};
  GPIOPin *pin_en_{nullptr};
  GPIOPin *pin_stb_{nullptr};

  uint16_t buffer_size_;
  uint16_t send_buffer_size_;
  
  uint8_t brightness_;
  uint8_t display_color_;
  bool enabled_;
};

template<typename... Ts> class TurnOnAction : public Action<Ts...>, public Parented<Hub75> {
 public:
  void play(Ts... x) override { this->parent_->panel_enable(true); }
};

template<typename... Ts> class TurnOffAction : public Action<Ts...>, public Parented<Hub75> {
 public:
  void play(Ts... x) override { this->parent_->panel_enable(false); }
};


}  // namespace hub75
}  // namespace esphome

