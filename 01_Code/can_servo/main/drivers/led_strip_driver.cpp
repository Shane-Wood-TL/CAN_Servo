/**
 * @file led_strip_drive.cpp
 * @brief This is the source file for the led strip class
 * @author Shane Wood
 * @date 14/12/2024
*/

#include "../include/drivers/led_strip_driver.h"


led_strip_driver::led_strip_driver(uint8_t strip_gpio_pin, uint8_t number_of_leds_in_strip){
    led_strip_config.strip_gpio_num = strip_gpio_pin;
    led_strip_config.max_leds=number_of_leds_in_strip;
    led_strip_config.led_model = LED_MODEL_WS2812;
    led_strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB;
    led_strip_config.flags = {
        .invert_out = false, // don't invert the output signal
    };

    esp_err_t ret = led_strip_new_rmt_device(&led_strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK || led_strip == nullptr) {
        ESP_LOGE("led_strip_driver", "Failed to initialize RMT LED strip: %s", esp_err_to_name(ret));
        return;
    }

    //(void)led_strip_new_rmt_device(&led_strip_config, &rmt_config, &led_strip);
    set_color(0,0,0,0,number_of_leds_in_strip-1);
}



void led_strip_driver::set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t led_start, uint8_t led_end){
    printf("LED updated: R:%d G:%d B:%d\n", r, g, b);
    for(uint8_t led_to_change = led_start; led_to_change <= led_end; led_to_change++){
        (void)led_strip_set_pixel(led_strip, led_to_change, r,g,b);
    }
    (void)led_strip_refresh(led_strip);
}