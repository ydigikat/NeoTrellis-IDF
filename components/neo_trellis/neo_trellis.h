/*
  ------------------------------------------------------------------------------
    ESP IDF driver for AdaFruit NeoTrellis 4x4 keypad.
    Author: YDigiKat
  ------------------------------------------------------------------------------
    MIT License
    Copyright (c) 2025 YDigiKat

    Permission to use, copy, modify, and/or distribute this code for any purpose
    with or without fee is hereby granted, provided the above copyright notice and
    this permission notice appear in all copies.
  ------------------------------------------------------------------------------
*/
#ifndef __NEO_TRELLIS_H__
#define __NEO_TRELLIS_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
#extern "C" {
#endif

#ifdef NDEBUG
#define ESP_DEBUG_ASSERT(cond) ((void)0)
#else
#include <assert.h>
#define ESP_DEBUG_ASSERT(cond) assert(cond)
#endif

/*
 * The AdaFruit SeeSaw is a board that provides a consolidated I2C access to a set
 * of configurable IO.  Configuartion and peripheral implementation is provided
 * by the onboard MCU, in this case an ATSAMD09 8-bit MCU.
 */
#define SS_ADDRESS (0x2E) /* The NeoTrellis default I2C address */

/*
 * The NeoTrellis is a 4x4 keypad with NeoPixel LED backlighting for
 * each key.  Keys raise events indicating their status and rising/falling
 * edges, these are FIFO queued. An interrupt can be generated.
 */
#define NT_ROWS (4)  /* Rows in grid */
#define NT_COLS (4)  /* Columns in grid */
#define NT_KEYS (16) /* Number of keys (and NeoPixels) */

/* Register base addresses */
enum base
{
  SeeSawBase = 0x00,
  PixelBase = 0x0E,
  KeyBase = 0x10
};

/* Commands (registers)*/
enum cmd
{
  SeeSawGetID = 0x01,
  SeeSawReset = 0x7F,
  PixelSetPin = 0x01,
  PixelSetSpeed = 0x02,
  PixelSetBufLen = 0x03,
  PixelSetColour = 0x04,
  PixelShow = 0x05,
  KeyEnableEvent = 0x01,
  KeySetInt = 0x02,
  KeyClearInt = 0x03,
  KeyGetEventCount = 0x04,
  KeyGetEvents = 0x10
};

/* Command params */
enum params
{
  SeeSawId = 0x55,
  PixelPin = 0x03,
  PixelType = 0x52,
  PixelSpeed = 0x01,
  KeyHigh = 0x00,
  KeyLow = 0x01,
  KeyFall = 0x02,
  KeyRise = 0x03
};

/* Key event type */
typedef union
{
  struct
  {
    uint8_t edge : 2; /* Edge? */
    uint8_t num : 6;  /* Which key */
  } bit;
  uint8_t reg;
} key_event_t;

/* 
* It looks like the internal seesaw key numbering is for a 16x4 device (AdaFruit M4?). This
* confused me for quite some time!
*
* Trellis Key | SeeSaw Key |
* 0-3         | 0-3        |
* 4-7         | 8-11       |
* 8-11        | 16-19      |
* 12-15       | 24-27      |
* 
* The 2 macros below handle this backwards and forwards mapping.
*/
#define TO_SEESAW_KEY(x) (((x) / 4) * 8 + ((x) % 4))
#define FROM_SEESAW_KEY(x) (((x) / 8) * 4 + ((x) % 8))

/* Device instance structure */
struct nt_dev
{
  i2c_master_dev_handle_t i2c_handle;
  uint8_t i2c_address;
};

/* API */
esp_err_t nt_init(struct nt_dev *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_address);
esp_err_t nt_refresh(struct nt_dev *dev);
esp_err_t nt_set_colour(struct nt_dev *dev, uint8_t button, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
esp_err_t nt_read_keys(struct nt_dev *dev, key_event_t *events, uint8_t *count);
esp_err_t nt_enable_key_event(struct nt_dev *dev, uint8_t key, enum params event_type);
esp_err_t nt_enable_all_event(struct nt_dev *dev, enum params event_type);

void nt_key_to_xy(uint8_t key, uint8_t *x, uint8_t *y);
uint8_t nt_xy_to_key(uint8_t x, uint8_t y);

#ifdef __cplusplus
}
#endif

#endif /* __NEO_TRELLIS_H__ */