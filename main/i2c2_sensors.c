/**
 * Simple example with humidity sensor and SGP40
 */
#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
//  #include <sht3x.h>
#include <sht4x.h> // shcho add : Stella use SHT40
#include <sgp40.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

/* float is used in printf(). you need non-default configuration in
 * sdkconfig for ESP8266, which is enabled by default for this
 * example. see sdkconfig.defaults.esp8266
 */

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "sgp40-example";
static sht4x_t dev_sht4x; //shcho add

static const char *voc_index_name(int32_t voc_index)
{
    if (voc_index <= 0) return "INVALID VOC INDEX";
    else if (voc_index <= 10) return "unbelievable clean";
    else if (voc_index <= 30) return "extremely clean";
    else if (voc_index <= 50) return "higly clean";
    else if (voc_index <= 70) return "very clean";
    else if (voc_index <= 90) return "clean";
    else if (voc_index <= 120) return "normal";
    else if (voc_index <= 150) return "moderately polluted";
    else if (voc_index <= 200) return "higly polluted";
    else if (voc_index <= 300) return "extremely polluted";



    return "RUN!";
}

void task_sgp40(void *pvParamters)
{
//      sht3x_t sht;
    sgp40_t sgp;

//      // setup SHT4x
	//shcho add accoring to ../../sht4x
    memset(&dev_sht4x, 0, sizeof(sht4x_t));
//      ESP_ERROR_CHECK(sht4x_init_desc(&dev_sht4x, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sht4x_init_desc(&dev_sht4x, 0, 16, 15));
    ESP_ERROR_CHECK(sht4x_init(&dev_sht4x));


//      // setup SHT3x
//      memset(&sht, 0, sizeof(sht));
//      ESP_ERROR_CHECK(sht3x_init_desc(&sht, 0, CONFIG_EXAMPLE_SHT3X_ADDR, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
//      ESP_ERROR_CHECK(sht3x_init(&sht));
//      // Start periodic measurements with 2 measurements per second.
//      ESP_ERROR_CHECK(sht3x_start_measurement(&sht, SHT3X_PERIODIC_2MPS, SHT3X_HIGH));
//      ESP_LOGI(TAG, "Humidity sensor initilalized");

    // setup SGP40
    memset(&sgp, 0, sizeof(sgp));
//      ESP_ERROR_CHECK(sgp40_init_desc(&sgp, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sgp40_init_desc(&sgp, 0, 16, 15));
    ESP_ERROR_CHECK(sgp40_init(&sgp));
    ESP_LOGW(TAG, "SGP40 initilalized. Serial: 0x%04x%04x%04x",
            sgp.serial[0], sgp.serial[1], sgp.serial[2]);

    // Wait until all set up
    vTaskDelay(pdMS_TO_TICKS(250));

    TickType_t last_wakeup = xTaskGetTickCount();
    while (1)
    {
        // Get the RHT values
        float temperature, humidity;
//          ESP_ERROR_CHECK(sht3x_get_results(&sht, &temperature, &humidity));
        ESP_ERROR_CHECK(sht4x_measure(&dev_sht4x, &temperature, &humidity));
        printf("sht4x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);

        // Feed it to SGP40
        int32_t voc_index;
        ESP_ERROR_CHECK(sgp40_measure_voc(&sgp, humidity, temperature, &voc_index));

        ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC index: %3" PRIi32 ", Air is [%s]",
                temperature, humidity, voc_index, voc_index_name(voc_index));

//          // Wait until 1 seconds (VOC cycle time) are over.
//          vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(5000)); // shcho change: according to ../../sht4x
    }
}

//  void app_main()
//  {
//      ESP_ERROR_CHECK(i2cdev_init());
//  
//  	//shcho add accoring to ../../sht4x
//      memset(&dev_sht4x, 0, sizeof(sht4x_t));
//  //      ESP_ERROR_CHECK(sht4x_init_desc(&dev_sht4x, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
//      ESP_ERROR_CHECK(sht4x_init_desc(&dev_sht4x, 0, 16, 15));
//      ESP_ERROR_CHECK(sht4x_init(&dev_sht4x));
//  
//  
//      xTaskCreate(task_sgp40, "sgp40", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
//  }
