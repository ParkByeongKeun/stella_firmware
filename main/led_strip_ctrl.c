/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      42 //0-->38-->48 -->42(Stella) :  Schematic이 부정확하다.
						// http://124.222.62.86/yd-data/YD-ESP32-S3/YD-ESP32-S3-classics/
						// https://github.com/vcc-gnd/YD-ESP32-S3?tab=readme-ov-file

#define EXAMPLE_LED_NUMBERS         1 // 24 -->1
//  #define EXAMPLE_CHASE_SPEED_MS      10
#define EXAMPLE_CHASE_SPEED_MS      500

#define NUM_COLOR	(3) // RGB
#define NUM_COLOR	(4) // RGBW

static const char *TAG = "example";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * NUM_COLOR];

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

//  void app_main(void)
//  void app_main_ctrl(int red, int green, int blue, int white)
void app_main_led_strip_ctrl(void *arg) //나중에 R/G/B/W로 변경하자
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    while (1) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < EXAMPLE_LED_NUMBERS; j += 3) {
                // Build RGB pixels
                hue = j * 360 / EXAMPLE_LED_NUMBERS + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels[j * NUM_COLOR + 0] = green/2; // shcho add /2
                led_strip_pixels[j * NUM_COLOR + 1] = blue /2; // shcho add /2
                led_strip_pixels[j * NUM_COLOR + 2] = red  /2; // shcho add /2

	//--------------------------------------------------------------------
//  	// 한번만하고 break;
//  //  				// No-Color
//  //                  led_strip_pixels[j * NUM_COLOR + 0] = 0; // Green
//  //                  led_strip_pixels[j * NUM_COLOR + 1] = 0; //Red
//  //                  led_strip_pixels[j * NUM_COLOR + 2] = 0; //Blue
//  				// Pink
//                  led_strip_pixels[j * NUM_COLOR + 0] = 0; // Green
//                  led_strip_pixels[j * NUM_COLOR + 1] = 100; //Red
//                  led_strip_pixels[j * NUM_COLOR + 2] = 100; //Blue
//  //  				// Yellow
//  //                  led_strip_pixels[j * NUM_COLOR + 0] = 87; // Green
//  //                  led_strip_pixels[j * NUM_COLOR + 1] = 100; //Red
//  //                  led_strip_pixels[j * NUM_COLOR + 2] = 0; //Blue
//  				// Green
//                  led_strip_pixels[j * NUM_COLOR + 0] = 255; // Green
//                  led_strip_pixels[j * NUM_COLOR + 1] = 0; //Red
//                  led_strip_pixels[j * NUM_COLOR + 2] = 0; //Blue
//  				// Red
//                  led_strip_pixels[j * NUM_COLOR + 0] = 0; // Green
//                  led_strip_pixels[j * NUM_COLOR + 1] = 255; //Red
//                  led_strip_pixels[j * NUM_COLOR + 2] = 0; //Blue
//  				// Blue
//                  led_strip_pixels[j * NUM_COLOR + 0] = 0; // Green
//                  led_strip_pixels[j * NUM_COLOR + 1] = 0; //Red
//                  led_strip_pixels[j * NUM_COLOR + 2] = 255; //Blue
//  				// Blue+Green
//                  led_strip_pixels[j * NUM_COLOR + 0] = 128; // Green
//                  led_strip_pixels[j * NUM_COLOR + 1] = 0; //Red
//                  led_strip_pixels[j * NUM_COLOR + 2] = 128; //Blue
//  //  				// Orange
//  //                  led_strip_pixels[j * NUM_COLOR + 0] = 35; // Green
//  //                  led_strip_pixels[j * NUM_COLOR + 1] = 100; //Red
//  //                  led_strip_pixels[j * NUM_COLOR + 2] = 0; //Blue
//  				#if (NUM_COLOR == 4 ) 
//                  	led_strip_pixels[j * NUM_COLOR + 3] = 0; //White
//  				#endif
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

//  			//shcho test : 1번만 Write하고 중단해도 유지되는지 확인
//  			break;
//  			//---------------------------------------------------------------//한번만 하고 break;

//              //shcho 끄지 않고 바로 변경
//              vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
//              memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}
