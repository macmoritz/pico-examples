/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "ili9341_lcd.pio.h"
#include "raspberry_256x256_rgb565.h"

// Tested with the parts that have the height of 240 and 320
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define IMAGE_SIZE 256
#define LOG_IMAGE_SIZE 8

// #define PIN_DIN 0
// #define PIN_CLK 1
// #define PIN_CS 2
// #define PIN_DC 3
// #define PIN_RESET 4
// #define PIN_BL 5
#define PIN_DIN 19
#define PIN_CLK 18
#define PIN_CS 17
#define PIN_DC 20
#define PIN_RESET 16
#define PIN_BL 5

#define SERIAL_CLK_DIV 1.f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// clang-format off
// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
static const uint8_t ili9341_init_seq[] = {
        4, 0, 0xef, 0x03, 0x80, 0x02,
        4, 0, 0xcf, 0x00, 0xc1, 0x30, // Power control B
        5, 0, 0xed, 0x64, 0x03, 0x12, 0x81, // Power on sequence control
        4, 0, 0xE8, 0x85, 0x00, 0x78,
        6, 0, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
        2, 0, 0xF7, 0x20,
        3, 0, 0xEA, 0x00, 0x00,
        2, 0, 0xc0, 0x23, // Power control 1 VRH[5:0]
        2, 0, 0xc1, 0x10, // Power control 2 SAP[2:0];BT[3:0]
        3, 0, 0xc5, 0x3e, 0x28, // VCM control 1
        2, 0, 0xc7, 0x86, // VCM control 2
        2, 0, 0x36, 0x48, // Memory access control
        2, 0, 0x37, 0x00, // Vertical scroll zero
        2, 0, 0x3a, 0x55, // Pixel Format 16-Bit RGB
        3, 0, 0xb1, 0x00, 0x18, // Frame rate control
        4, 0, 0xb6, 0x08, 0x82, 0x27, // Display function control
        2, 0, 0xf2,  0x00, // Gamma function disable
        2, 0, 0x26, 0x01, // Gamma curve selected
        16, 0, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Positive gamma correction
        16, 0, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Negative gamma correction
        2, 0, 0x11, 0x80, // Exit Sleep
        2, 0, 0x29, 0x80, // Display on
        0                                   // Terminate list
};
// clang-format on

static inline void lcd_set_dc_cs(bool dc, bool cs) {
  sleep_us(1);
  gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS),
                  !!dc << PIN_DC | !!cs << PIN_CS);
  sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd,
                                 size_t count) {
  ili9341_lcd_wait_idle(pio, sm);
  lcd_set_dc_cs(0, 0);
  ili9341_lcd_put(pio, sm, *cmd++);
  if (count >= 2) {
    ili9341_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 0);
    for (size_t i = 0; i < count - 1; ++i)
      ili9341_lcd_put(pio, sm, *cmd++);
  }
  ili9341_lcd_wait_idle(pio, sm);
  lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq) {
  const uint8_t *cmd = init_seq;
  while (*cmd) {
    lcd_write_cmd(pio, sm, cmd + 2, *cmd);
    if (*cmd & 0x80) {
      sleep_ms(150);
    }
    // sleep_ms(*(cmd + 1));
    cmd += *cmd + 2;
  }
}

static inline void ili9341_start_pixels(PIO pio, uint sm) {
  uint8_t cmd = 0x2c; // RAMWR
  lcd_write_cmd(pio, sm, &cmd, 1);
  lcd_set_dc_cs(1, 0);
}

int main() {
  stdio_init_all();

  PIO pio = pio0;
  uint sm = 0;
  uint offset = pio_add_program(pio, &ili9341_lcd_program);
  ili9341_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

  gpio_init(PIN_CS);
  gpio_init(PIN_DC);
  gpio_init(PIN_RESET);
  gpio_init(PIN_BL);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_set_dir(PIN_DC, GPIO_OUT);
  gpio_set_dir(PIN_RESET, GPIO_OUT);
  gpio_set_dir(PIN_BL, GPIO_OUT);

  gpio_put(PIN_CS, 1);
  gpio_put(PIN_RESET, 1);
  gpio_put(PIN_DC, 1);

  uint8_t swreset = 0x01;
  lcd_write_cmd(pio, sm, &swreset, 1);
  sleep_ms(150);
  gpio_put(PIN_RESET, 0);
  sleep_ms(5);
  gpio_put(PIN_RESET, 1);
  sleep_ms(150);

  lcd_init(pio, sm, ili9341_init_seq);
  gpio_put(PIN_BL, 1);

  // Other SDKs: static image on screen, lame, boring
  // Raspberry Pi Pico SDK: spinning image on screen, bold, exciting

  // Lane 0 will be u coords (bits 8:1 of addr offset), lane 1 will be v
  // coords (bits 16:9 of addr offset), and we'll represent coords with
  // 16.16 fixed point. ACCUM0,1 will contain current coord, BASE0/1 will
  // contain increment vector, and BASE2 will contain image base pointer
#define UNIT_LSB 16
  interp_config lane0_cfg = interp_default_config();
  interp_config_set_shift(&lane0_cfg,
                          UNIT_LSB - 1); // -1 because 2 bytes per pixel
  interp_config_set_mask(&lane0_cfg, 1, 1 + (LOG_IMAGE_SIZE - 1));
  interp_config_set_add_raw(&lane0_cfg,
                            true); // Add full accumulator to base with each POP
  interp_config lane1_cfg = interp_default_config();
  interp_config_set_shift(&lane1_cfg, UNIT_LSB - (1 + LOG_IMAGE_SIZE));
  interp_config_set_mask(&lane1_cfg, 1 + LOG_IMAGE_SIZE,
                         1 + (2 * LOG_IMAGE_SIZE - 1));
  interp_config_set_add_raw(&lane1_cfg, true);

  interp_set_config(interp0, 0, &lane0_cfg);
  interp_set_config(interp0, 1, &lane1_cfg);
  interp0->base[2] = (uint32_t)raspberry_256x256;

  float theta = 0.f;
  float theta_max = 2.f * (float)M_PI;
  while (1) {
    const uint64_t time = to_us_since_boot(get_absolute_time());
    
    theta += 0.02f;
    if (theta > theta_max)
      theta -= theta_max;
    int32_t rotate[4] = {(int32_t)(cosf(theta) * (1 << UNIT_LSB)),
                         (int32_t)(-sinf(theta) * (1 << UNIT_LSB)),
                         (int32_t)(sinf(theta) * (1 << UNIT_LSB)),
                         (int32_t)(cosf(theta) * (1 << UNIT_LSB))};
    interp0->base[0] = rotate[0];
    interp0->base[1] = rotate[2];
    ili9341_start_pixels(pio, sm);
    for (int y = 0; y < SCREEN_HEIGHT; ++y) {
      interp0->accum[0] = rotate[1] * y;
      interp0->accum[1] = rotate[3] * y;
      for (int x = 0; x < SCREEN_WIDTH; ++x) {
        uint16_t colour = *(uint16_t *)(interp0->pop[2]);
        ili9341_lcd_put(pio, sm, colour >> 8);
        ili9341_lcd_put(pio, sm, colour & 0xff);
      }
    }
  
    const int64_t delta = absolute_time_diff_us(time, to_us_since_boot(get_absolute_time()));
    printf("vBlank stuff took %lld us\n", delta);
  }
}
