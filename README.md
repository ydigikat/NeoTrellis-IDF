# AdaFruit NeoTrellis 4x4 RGB Keypad

This is an ESP IDF "C" driver for the NeoTrellis 4x4 RGB.  

The code is largely self-explanatory exporting an API trimmed to bare minimum to handle my own use cases. 

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

