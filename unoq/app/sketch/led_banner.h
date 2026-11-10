#pragma once

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

// "ArduHumanoid" scrolling on the Uno Q LED matrix (13 x 8) while the sketch runs: the board
// shows at a glance that the firmware is alive. ledBannerBegin() once, ledBannerStep() from
// loop(); a step only copies 104 bytes into the frame buffer of the matrix driver.

static Arduino_LED_Matrix ledMatrix;
static const char LED_BANNER_TEXT[] = "ArduHumanoid";
static const uint8_t LED_COLS = 13, LED_ROWS = 8;
static const uint8_t LED_LEVEL = 4;           // of 7
static const uint32_t LED_STEP_MS = 90;       // one column per step

// 5x7 glyphs, one byte per column, bit 0 = top row
struct LedGlyph { char c; uint8_t col[5]; };
static const LedGlyph LED_FONT[] = {
  {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
  {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
  {'a', {0x20, 0x54, 0x54, 0x54, 0x78}},
  {'d', {0x38, 0x44, 0x44, 0x48, 0x7F}},
  {'i', {0x00, 0x44, 0x7D, 0x40, 0x00}},
  {'m', {0x7C, 0x04, 0x18, 0x04, 0x78}},
  {'n', {0x7C, 0x08, 0x04, 0x04, 0x78}},
  {'o', {0x38, 0x44, 0x44, 0x44, 0x38}},
  {'r', {0x7C, 0x08, 0x04, 0x04, 0x08}},
  {'u', {0x3C, 0x40, 0x40, 0x20, 0x7C}},
};

// the text as a strip of columns, with a blank screen before and after it
static uint8_t ledStrip[2 * LED_COLS + 6 * (sizeof(LED_BANNER_TEXT) - 1)];
static uint16_t ledStripLen = 0, ledOffset = 0;
static uint32_t ledNextMs = 0;

static void ledBannerBegin() {
  uint16_t n = LED_COLS;      // ledStrip is zero-initialised: leading blank
  for (const char *p = LED_BANNER_TEXT; *p; ++p) {
    for (const LedGlyph &g : LED_FONT) {
      if (g.c == *p) {
        memcpy(&ledStrip[n], g.col, 5);
        break;
      }
    }
    n += 6;                   // 5 columns and a gap; an unknown character is a blank
  }
  ledStripLen = n;            // the trailing blank screen follows
  ledMatrix.begin();
  ledMatrix.setGrayscaleBits(3);
  ledNextMs = millis();
}

static void ledBannerStep() {
  if ((int32_t)(millis() - ledNextMs) < 0) return;
  ledNextMs += LED_STEP_MS;
  uint8_t frame[LED_COLS * LED_ROWS] = {};
  for (uint8_t x = 0; x < LED_COLS; ++x) {
    const uint8_t col = ledStrip[ledOffset + x];
    for (uint8_t y = 0; y < 7; ++y) {
      if (col & (1u << y)) frame[y * LED_COLS + x] = LED_LEVEL;
    }
  }
  ledMatrix.draw(frame);
  if (++ledOffset > ledStripLen) ledOffset = 0;
}
