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
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "neo_trellis.h"

static const char *TAG = "neotrellis_example";

/* i2c peripheral configuration */
#define I2C_MASTER_SCL (9)
#define I2C_MASTER_SDA (8)
#define I2C_MASTER_PORT (I2C_NUM_0)

static struct nt_dev neotrellis_dev;
static i2c_master_bus_handle_t i2c_bus_handle;

/**
 * \brief initialises the i2c master peripheral
 * \return ESP_OK or error code.
 */
static esp_err_t init_i2c()
{
    i2c_master_bus_config_t i2c_mst_config =
        {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_MASTER_PORT,
            .scl_io_num = I2C_MASTER_SCL,
            .sda_io_num = I2C_MASTER_SDA,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true};

    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "FAIL: i2c bus created - %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PASSED: i2c bus created");
    return ESP_OK;
}

/**
 * \brief set button colours
 */
static void set_colours(void)
{
    for (uint8_t key = 0; key < NT_KEYS; key++)
    {
        uint8_t row = key / 4;
        switch (row)
        {
        case 0:
            nt_set_colour_rgb(&neotrellis_dev, key, RGB_RED, 0x20);
            break;
        case 1:
            nt_set_colour_rgb(&neotrellis_dev, key, RGB_GREEN, 0x20);
            break;
        case 2:
            nt_set_colour_rgb(&neotrellis_dev, key, RGB_BLUE, 0x20);
            break;
        case 3:
            nt_set_colour_rgb(&neotrellis_dev, key, RGB_YELLOW, 0x20);
            break;
        }
    }
    nt_display(&neotrellis_dev);
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Clear colours for button press testing */
    for (uint8_t key = 0; key < NT_KEYS; key++)
    {
        nt_set_colour_rgb(&neotrellis_dev, key, RGB_WHITE, 0x20);
    }
    nt_display(&neotrellis_dev);
}

/**
 * \brief program entry point
 */
void app_main(void)
{
    /* Initialise i2c */
    esp_err_t ret = init_i2c();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }

    /* Initialise the neo-trellis device */
    ret = nt_init(&neotrellis_dev, i2c_bus_handle, SS_ADDRESS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NeoTrellis: %s", esp_err_to_name(ret));
        return;
    }

    /* Register the key events we're interested in */
    ret = nt_enable_all_event(&neotrellis_dev, KeyReleased);
    ret = nt_enable_all_event(&neotrellis_dev, KeyPressed);

    /* Set the backlighting */
    set_colours();

    key_event_t events[16];

    while (1)
    {
        uint8_t event_count = sizeof(events);

        nt_read_keys(&neotrellis_dev, events, &event_count);

        if (event_count > 0)
        {
            ESP_LOGI(TAG, "Received %d events", event_count);
            for (int i = 0; i < event_count; i++)
            {
                uint8_t key = SEESAW_KEY_TO_TRELLIS(events[i].bit.key);

                ESP_LOGI(TAG, "Raw event[%d]: seesaw key:%d, trellis key: %d, edge: %d",
                         i, key, SEESAW_KEY_TO_TRELLIS(key), events[i].bit.edge);

                if (events[i].bit.edge == KeyReleased)
                {                    
                    nt_set_colour_rgb(&neotrellis_dev, key, RGB_PURPLE, 0x20);
                }
                else if (events[i].bit.edge == KeyPressed)
                {                 
                    nt_set_colour_rgb(&neotrellis_dev, key, RGB_WHITE, 0x20);
                }
            }

            nt_display(&neotrellis_dev);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
