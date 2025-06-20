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
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "neo_trellis.h"

static const char *TAG = "neo_trellis";

/* Retry values */
#define I2C_TIMEOUT (1000)
#define I2C_RETRIES (10)
#define I2C_DELAY_US (250)

/* private functions */
static esp_err_t init(struct nt_dev *dev, i2c_master_bus_handle_t bus_handle);
static esp_err_t probe(struct nt_dev *dev, i2c_master_bus_handle_t bus_handle);
static esp_err_t verify(struct nt_dev *dev);
static esp_err_t write(struct nt_dev *dev, enum base base, enum cmd cmd, uint8_t *data, size_t len);
static esp_err_t read(struct nt_dev *dev, enum base base, enum cmd cmd, uint8_t *data, size_t len, uint32_t delay_us);

/**
 * \brief initialises the AdaFruit NeoTrellis keypad
 * \param dev the device instance
 * \param bus_handle handle to the i2c bus
 * \param i2c_address 1 of 5 possible i2c addresses (solder bridges, default = 0x0E)
 * \return esp_err_t code
 */
esp_err_t nt_init(struct nt_dev *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_address)
{
    ESP_DEBUG_ASSERT(dev);

    esp_err_t ret = ESP_OK;

    dev->i2c_address = i2c_address;

    ESP_RETURN_ON_ERROR(init(dev, bus_handle), TAG, "Failed to initialise SeeSaw device.");

    return ret;
}

/**
 * \brief this refreshes the LED pixel string
 * \details this requires the buffer to be streamed followed by a >= 300ms delay which
 *          signals the data to be latched into the pixels.
 * \param dev the device instance
 */
esp_err_t nt_refresh(struct nt_dev *dev)
{
    ESP_DEBUG_ASSERT(dev);

    esp_err_t ret = write(dev, PixelBase, PixelShow, NULL, 0);
    if (ret == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    return ret;
}

/**
 * \brief sets the backlight colour for a button
 * \details note that this sets the colour but does not refresh the hardware, you must make
 *          a subsequent call to nt_refresh() for this.  The device uses GRB format, 3 bytes
 *          per LED.
 * \param dev the device instance
 * \param button the button number
 * \param red red hue
 * \param greem green hue
 * \param blue blue hue
 * \param brightness brightness from 0x00 (off) to 0xFF (maximum brightness), 0x32 is comfortable.
 * \return esp_err_t
 */
esp_err_t nt_set_colour(struct nt_dev *dev, uint8_t button, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
    /* 3 bytes per pixel, GRB format*/
    uint8_t buf_idx = button * 3;
    uint8_t data[5] = {(buf_idx >> 8), buf_idx, (green * brightness) >> 8, (red * brightness) >> 8, (blue * brightness) >> 8};

    return write(dev, PixelBase, PixelSetColour, data, 5);
}

esp_err_t nt_read_keys(struct nt_dev *dev, key_event_t *events, uint8_t *count)
{
    ESP_DEBUG_ASSERT(dev);
    ESP_DEBUG_ASSERT(count);
    ESP_DEBUG_ASSERT(events);

    uint8_t fifo_count;
    ESP_RETURN_ON_ERROR(read(dev, KeyBase, KeyGetEventCount, &fifo_count, 1, 500), TAG, "FAILED: read event count");

    /* Cap the read if there are too many events for buffer supplied */
    uint8_t read_count = fifo_count < *count ? fifo_count : *count;
    if (read_count > 0)
    {
        ESP_RETURN_ON_ERROR(read(dev, KeyBase, KeyGetEvents, (uint8_t *)events, read_count, 1000), TAG, "FAILED: reading %d events", read_count);
    }

    /* Return the number of events read */
    *count = read_count;

    return ESP_OK;
}

/**
 * \brief converts an x,y pair to a key number
 * \param x x coordinate
 * \param y y coordinate
 * \return key number
 */
uint8_t nt_xy_to_key(uint8_t x, uint8_t y)
{
    if (x >= NT_COLS || y >= NT_ROWS) 
    {
        return 0xFF; /* Bad key */
    }
    return y * NT_COLS + x;
}

/**
 * \brief converts a key number to x,y pair
 * \param key the key number
 * \param x pointer to hold x coordinate
 * \param y pointer to hold y coordinate 
 */
void nt_key_to_xy(uint8_t key, uint8_t *x, uint8_t *y) 
{
    if (key >= NT_KEYS || !x || !y) 
    {
        if (x) *x = 0xFF;
        if (y) *y = 0xFF;
        return;
    }
    *x = key % NT_COLS;
    *y = key / NT_COLS;
}



/**
 * \brief initialises the SeeSaw device.
 * \details This sets up the SeeSaw MCU on the device:
 *          1. Adds it to i2c and probes for presence.
 *          2. Software resets the MCU.
 *          3. Verifies the presence of the expected MCU.
 *          4. Configures the neopixel string (LEDs).
 *          5. Configures the keypad and key events.
 * \param dev the device instance
 * \return esp_err_t code
 */
static esp_err_t init(struct nt_dev *dev, i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t i2c_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev->i2c_address,
        .scl_speed_hz = 100000};

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_cfg, &dev->i2c_handle), "TAG", "Failed to add device.");

    /* Probe for an i2c response and issue a software reset */
    ESP_RETURN_ON_ERROR(probe(dev, bus_handle), TAG, "FAILED: pre-reset probe %X", dev->i2c_address);
    ESP_RETURN_ON_ERROR(write(dev, SeeSawBase, SeeSawReset, ((uint8_t[]){0xFF}), 1), TAG, "FAILED: reset");
    ESP_RETURN_ON_ERROR(probe(dev, bus_handle), TAG, "FAILED: post-reset probe %X", dev->i2c_address);
    ESP_LOGI(TAG, "PASSED: reset and probe sequence");

    /* Verify the MCU on the end of the wire */
    ESP_RETURN_ON_ERROR(verify(dev),TAG,"FAILED: verification");
    ESP_LOGI(TAG,"PASSED: verification");

    /* Configure the LEDs */
    uint8_t buf_bytes = 3 * NT_KEYS; /* 3 bytes for each key's neopixel (48) */

    ESP_RETURN_ON_ERROR(write(dev, PixelBase, PixelSetPin, ((uint8_t[]){PixelPin}), 1), TAG, "FAILED: setting neopixel output pin");
    ESP_RETURN_ON_ERROR(write(dev, PixelBase, PixelSetBufLen, ((uint8_t[]){(uint8_t)(buf_bytes >> 8), (uint8_t)(buf_bytes & 0xFF)}), 2), TAG, "FAILED: setting neopixel output pin");
    ESP_RETURN_ON_ERROR(write(dev, PixelBase, PixelSetSpeed, ((uint8_t[]){PixelSpeed}), 1), TAG, "FAILED: reset");
    ESP_LOGI(TAG, "PASSED: neo-pixel string configured");

    uint8_t state_reg = 0;
    state_reg |= 0x01 | 0x01 << (KeyRise + 1) | 0x01 << (KeyFall + 1);

    /* Set up keypad */
    for (int key = 0; key < 16; key++)
    {
        nt_set_colour(dev, key, 0x00, 0x00, 0x00, 0x00);
        ESP_RETURN_ON_ERROR(write(dev, KeyBase, KeyEnableEvent, ((uint8_t[]){TO_SEESAW_KEY(key), state_reg}), 2),TAG,"FAILED: enable event for key %d",key);                     
    }
    ESP_RETURN_ON_ERROR(write(dev,KeyBase,KeySetInt,((uint8_t[]){0x01}),1),TAG,"FAILED: setting interrupt enable");
    ESP_LOGI(TAG, "PASSED: keypad configured");

    /* refresh the hardware */
    nt_refresh(dev);

    return ESP_OK;
}

/**
 * \brief validates the ID of the SeeSaw microcontroller
 * \param dev the instance of the device
 * \return esp_err_t
 */
static esp_err_t verify(struct nt_dev *dev)
{
    /* Check that we're talking to the right hardware */
    uint8_t id = 0;

    for (int retries = 0; retries < 10; retries++)
    {
        esp_err_t ret = read(dev, SeeSawBase, SeeSawGetID, &id, 1, I2C_DELAY_US);
        if (ret == ESP_OK && id == SeeSawId)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "RETRY: verify microcontroller id");
    }

    ESP_RETURN_ON_FALSE(id == SeeSawId, ESP_ERR_INVALID_RESPONSE, TAG, "FAILED: verify microcontroller ID %x", id);

    return ESP_OK;
}

/**
 * \brief probes for an ACK from an i2c device at the specified address
 * \param dev the device instance
 * \param bus_handle handle to the i2c bus
 * \return true or false
 */
static esp_err_t probe(struct nt_dev *dev, i2c_master_bus_handle_t bus_handle)
{
    bool ret = false;

    for (int x = 0; x < I2C_RETRIES; x++)
    {
        ret = i2c_master_probe(bus_handle, dev->i2c_address, I2C_TIMEOUT);
        if (ret == ESP_OK)
        {
            ret = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ret ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/**
 * \brief writes a command and data to the SeeSaw
 * \param dev the device instance
 * \param base the register base
 * \param cmd the command
 * \param data command data
 * \param len length of the command data
 */
static esp_err_t write(struct nt_dev *dev, enum base base, enum cmd cmd, uint8_t data[], size_t len)
{
    ESP_DEBUG_ASSERT(dev);
    ESP_DEBUG_ASSERT(len <= 6);

    uint8_t buf[8] = {base, cmd};
    memcpy(&buf[2], data, len);

    return i2c_master_transmit(dev->i2c_handle, buf, len + 2, I2C_TIMEOUT);
}

/**
 * \brief receives data from the SeeSaw
 * \param dev the device instance
 * \param base the register base
 * \param cmd the command
 * \param data where to place the received data
 * \param len the size of the data buffer
 * \param delay delay to place between sending the command and receiving response.
 */
static esp_err_t read(struct nt_dev *dev, enum base base, enum cmd cmd, uint8_t *data, size_t len, uint32_t delay)
{
    ESP_DEBUG_ASSERT(dev);
    ESP_DEBUG_ASSERT(data);

    /* Transmit command */
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->i2c_handle, ((uint8_t[]){base, cmd}), 2, I2C_TIMEOUT), TAG, "Failed to send read cmd %d:%d", base, cmd);

    /* Allow time to respond */
    if (delay)
    {
        esp_rom_delay_us(delay);
    }

    /* Receive data */
    return i2c_master_receive(dev->i2c_handle, data, len, I2C_TIMEOUT);
}
