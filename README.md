# AdaFruit NeoTrellis 4x4 RGB Keypad

This is an ESP IDF "C" driver for the NeoTrellis 4x4 RGB.  

The code is largely self-explanatory exporting an API trimmed to bare minimum to handle my own use cases. 

The driver is stateless meaning, for example,  that you cannot ask it what colour an LED is currently displaying - you need to manage that yourself.

The example targets the ESP32-S3 but will work with most other variants.

### API 

```c
/* Initialisation */
esp_err_t nt_init(struct nt_dev *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_address);

/* LED colours */
esp_err_t nt_set_colour(struct nt_dev *dev, uint8_t button, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
esp_err_t nt_refresh(struct nt_dev *dev);

/* Keypad events */
esp_err_t nt_enable_event(struct nt_dev *dev, key_event_t event);
esp_err_t nt_read_keys(struct nt_dev *dev, key_event_t *events, uint8_t *count);

/* Utility */
void nt_key_to_xy(uint8_t key, uint8_t *x, uint8_t *y);
uint8_t nt_xy_to_key(uint8_t x, uint8_t y);
```
### Example Code
See the example ```app_main()``` in the driver code repository for basic usage. 

### Interrupts
The hardware interrupt from the keypad is enabled in the driver although this is not used in the example.  

To use this you need to connect the interrupt line to a free GPIO pin on the ESP32 and optionally provide an ISR.

The interrupt line is active low, pulled high by the internal pullup, so we're looking for the falling edge.

The snippet below provides a skeletal interrupt handler using a simple flag to indicate a key press interrupt.

Note the point about the interrupt staying set until the FIFO is read which allows an alternative approach whereby the state of the interrupt pin is polled continually rather than triggering an ISR, this
approach is more correctly referred to as event based polling.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define NEOTRELLIS_INT_GPIO 4    /* This can be pretty much any pin, but avoid the strapping pins */
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "Interrupt Example";
static volatile bool keys_pending;

/* ISR: keep short, defer processing */
static void IRAM_ATTR trellis_int_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Signal the main that we've had key events*/
    keys_pending = true;

    /* You might also use a FreeRTOS queue or semaphore here to notify a task */
    ESP_EARLY_LOGI(TAG, "NeoTrellis interrupt triggered");

    /* Yield to any higher priority task */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup_neotrellis_int_gpio()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NEOTRELLIS_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     /* INT is active-low; enable pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE        /* Trigger on falling edge (HIGH → LOW) */
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(NEOTRELLIS_INT_GPIO, trellis_int_isr_handler, NULL);    
}

void app_main(void)
{  
    setup_neotrellis_int_gpio();

    keys_pending = false;

    /* Initialise the neo-trellis etc here (see example app_main() in repo)*/

    /* nt_init(...) etc */
                

    while (1)
    {
        /* Read the key FIFO here, this is important as the read causes the
           SeeSaw firmware to clear the interrupt which otherwise remains set */

        if(keys_pending)
        {
            /* nt_read_keys(...)  etc */
            
            keys_pending = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

