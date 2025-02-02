#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

/*
 You have to set this config value with menuconfig
 CONFIG_INTERFACE

 for i2c
 CONFIG_MODEL
 CONFIG_SDA_GPIO
 CONFIG_SCL_GPIO
 CONFIG_RESET_GPIO

 for SPI
 CONFIG_CS_GPIO
 CONFIG_DC_GPIO
 CONFIG_RESET_GPIO
*/

#define tag "SSD1306"

extern MessageBufferHandle_t passkey_msg_handle;
char *Stella_Tag="STELLA OLED";

//  void app_main(void)
void app_main_task_oled(void *arg)
{
	SSD1306_t dev;
#if 0
	int center, top, bottom;
	char lineChar[20];
#endif

//----------------------------------------------------------------------
// 이 Example은 I2C를 계속 붙잡고 있어서 일단 한번만 수행해서
// Display만 하는 것으로 하고 나중에 수정한다.
#if CONFIG_I2C_INTERFACE
	ESP_LOGW(tag, "INTERFACE is i2c");
	ESP_LOGW(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGW(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGW(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE
//
//
//  //======================================================================
//  	dev._address = I2C_ADDRESS_SSD1306;
//  	dev._flip = false;
//  	dev._i2c_num = I2C_NUM;
//  	dev._i2c_bus_handle = i2c_bus_handle;
//  //  //  	dev->_i2c_bus_handle = tool_bus_handle_i2c1; //shcbo test
//  	dev._i2c_dev_handle = i2c_dev_handle;
//  //
//----------------------------------------------------------------------

#if CONFIG_FLIP
	dev._flip = true;
	ESP_LOGW(tag, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
	ESP_LOGI(tag, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
	ESP_LOGW(tag, "Panel is 128x32");
	ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

	#if 0 // org
	while(1) 
	{
		ssd1306_clear_screen(&dev, false);
		ssd1306_contrast(&dev, 0xff);

		ssd1306_display_text_box1(&dev, 0, 48, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 4,  4, false, 100);
		ssd1306_display_text_box1(&dev, 0, 48, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 4, 26, false,   2);
		ssd1306_display_text_box1(&dev, 1, 32, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 8,  8, false, 100);
		ssd1306_display_text_box1(&dev, 1, 32, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 8, 26, false,   3);
		ssd1306_display_text_box1(&dev, 2, 16, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",12, 12, false, 100);
		ssd1306_display_text_box1(&dev, 2, 16, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",12, 26, false,   4);
		ssd1306_display_text_box1(&dev, 3,  0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",16, 16, false, 100);
		ssd1306_display_text_box1(&dev, 3,  0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",16, 26, false,   5);
		vTaskDelay(1000);

#if CONFIG_SSD1306_128x64
		ssd1306_display_text_box2(&dev, 4, 48, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 4, 26, false, 2);
		ssd1306_display_text_box2(&dev, 5, 32, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 8, 26, false, 2);
		ssd1306_display_text_box2(&dev, 6, 16, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",12, 26, false, 2);
		ssd1306_display_text_box2(&dev, 7,  0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",16, 26, false, 2);
		vTaskDelay(100);
#endif

	} // end while
	#else
	// 3.3V를 연결하면 화면이 점점 어두워 진다.
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text_box1(&dev, 1, 8, "STELLA WEARABLE",            15, 15, false, 5);
//  	while(1) 
	{

//  ==================== org demo ====================================================
//  //  		ssd1306_display_text_box1(&dev, 0, 48, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 4,  4, false, 100);
//  //  		ssd1306_display_text_box1(&dev, 0, 48, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 4, 26, false,   2);
//  //  //
//  //  		ssd1306_display_text_box1(&dev, 1, 32, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 8,  8, false, 100);
//  //  		ssd1306_display_text_box1(&dev, 1, 32, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 8, 26, false,   3);
//  //  //
//  //  		ssd1306_display_text_box1(&dev, 2, 16, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",12, 12, false, 100);
//  //  		ssd1306_display_text_box1(&dev, 2, 16, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",12, 26, false,   4);
//  //  //
//  //  		ssd1306_display_text_box1(&dev, 3,  0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",16, 16, false, 100);
//  //  		ssd1306_display_text_box1(&dev, 3,  0, "ABCDEFGHIJKLMNOPQRSTUVWXYZ",16, 26, false,   5);
//  //  //
//  ===================================================================================================
//  		ssd1306_display_text_box1(&dev, 1, 48, "Stella Wearable", 6,  6, false, 100);

//  void ssd1306_display_text_box1(SSD1306_t * dev, int page, 
//                    int seg,  :: start from left(pixel)
//                    char * text, 
//                    int box_width,  // Box에 표시할 문자수  max : 16 ; 8x6 font
//                    int text_len, : Box의 Test Pixel수
//                    bool invert, int delay)
//

//  		                                   12345678901234567890123456789012
//  		ssd1306_display_text_box1(&dev, 3, 8, "Sensing -> Send           ", 15, 15, false, 5);
//  		vTaskDelay(100);
//  		ssd1306_display_text_box1(&dev, 3, 8, "               ", 15, 15, false, 5);
//  		vTaskDelay(50);
//                                                       1         2         3         4
//  		                                    123456789012345678901234567890123456789012

//  	ssd1306_display_text_box1(&dev, 3, 24, "Sensing...->Send to Server...             ", 12, 42, false, 5);
		ssd1306_display_text_box1(&dev, 2, 8, "Sensing->Server",  15, 15, false, 5);
		ESP_LOGI("shcho", " OLED loop ");

//  		uint32_t passkey;
//          size_t rx_bytes = xMessageBufferReceive( passkey_msg_handle, (void*)(&passkey), sizeof(passkey), portMAX_DELAY );
//          assert(rx_bytes == sizeof(uint32_t));
//  		char tmp_buf[20];
//  		sprintf(tmp_buf,"passkey:%" PRIu32,  passkey);
//  		ssd1306_display_text_box1(&dev, 3, 10, tmp_buf,  15, 15, false, 5);


	} // end while
	#endif

    if (i2c_master_bus_rm_device(dev._i2c_dev_handle) != ESP_OK) {
		ESP_LOGE("oled:i2c_master_init()", "i2c_master_bus_rm_device(dev._i2c_dev_handle)");
		ESP_LOGE("oled:i2c_master_init()", "i2c_master_bus_rm_device(dev._i2c_dev_handle)");
		ESP_LOGE("oled:i2c_master_init()", "i2c_master_bus_rm_device(dev._i2c_dev_handle)");
    }

    if (i2c_del_master_bus(dev._i2c_bus_handle) != ESP_OK) {
		ESP_LOGE("oled:i2c_master_init()", "i2c_master_bus_rm_device(dev._i2c_dev_handle)");
		ESP_LOGE("oled:i2c_master_init()", "i2c_master_bus_rm_device(dev._i2c_dev_handle)");
		ESP_LOGE("oled:i2c_master_init()", "i2c_master_bus_rm_device(dev._i2c_dev_handle)");
    }

//  	vTaskDelete(NULL);

}
