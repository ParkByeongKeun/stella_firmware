/* SPI Master Half Duplex EEPROM example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <inttypes.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "spi_eeprom.h"

#include "ADS114S08URHBR.h"
#include <unistd.h>
#include <arpa/inet.h>

#include <cJSON.h>

#include <esp_random.h>
#include "ambient_limit.h"

extern SemaphoreHandle_t sema_tcp ;
extern SemaphoreHandle_t sema_spi_ads114s;
extern SemaphoreHandle_t sema_uart2 ;

extern char my_mac_str[32];
extern int flag_IS_WEARABLE ;
extern int fd_uart2 ;
extern int send_to_server(char *payload, int len);
extern esp_err_t ijoon_get_nvs_str(uint8_t *key, uint8_t *value);

static const char *JSON_TAG = "JSON";

double Sensitivity_H2S = 214.13;
double Sensitivity_O3 =   60.66;
double Sensitivity_CO =    4.42;
double Sensitivity_NO2 =  22.48;
double TIA_Gain_H2S =  49.9;
double TIA_Gain_O3 =  499;
double TIA_Gain_CO =  100; 
double TIA_Gain_NO2 = 499;
double M_H2S = 0.0;
double M_O3  = 0.0;
double M_CO  = 0.0;
double M_NO2 = 0.0;
double Vgas0_H2S = 0.83084;
double Vgas0_O3  = 0.83477;
double Vgas0_CO  = 0.82867;
double Vgas0_NO2 = 0.88737;

/*
//   This code demonstrates how to use the SPI master half duplex mode to read/write a AT932C46D EEPROM (8-bit mode).
//   -->
 This code demonstrates how to use the SPI master full-duplex ADS114S.
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  #if CONFIG_IDF_TARGET_ESP32
//  #  if CONFIG_EXAMPLE_USE_SPI1_PINS
//  #   define EEPROM_HOST      SPI1_HOST
//  // Use default pins, same as the flash chip.
//  #   define PIN_NUM_MISO     7
//  #   define PIN_NUM_MOSI     8
//  #   define PIN_NUM_CLK      6
//  #  else
//  #   define EEPROM_HOST      HSPI_HOST
//  #   define PIN_NUM_MISO     18
//  #   define PIN_NUM_MOSI     23
//  #   define PIN_NUM_CLK      19
//  #  endif
//  #  define PIN_NUM_CS        13
//  #else
//  #  define EEPROM_HOST       SPI2_HOST
//  #  define PIN_NUM_MISO      13
//  #  define PIN_NUM_MOSI      12
//  #  define PIN_NUM_CLK       11
//  #  define PIN_NUM_CS        10
//  #endif

#  define ADS114S_SPI_HOST       SPI2_HOST

#  define PIN_NUM_CS    9
//  #  define PIN_NUM_CS   20 // SPI2_CS가 ADS114S가 뜨거워지면서 죽은것 같다. 
//                               --> wearable에서는 USB D+라서 GPIO5로 변경함
//  #  define PIN_NUM_CS        5 // 

#  define PIN_NUM_CLK       10
#  define PIN_NUM_MISO      11
#  define PIN_NUM_MOSI      12
#  define ADS114S_nRDY_PIN  13


#define ESP_INTR_FLAG_DEFAULT 0
#define ADS114S_CLK_FREQ         (1*1000*1000)   //When powered by 3.3V, EEPROM max freq is 1MHz
#define ADS114S_INPUT_DELAY_NS   ((1000*1000*1000/ADS114S_CLK_FREQ)/2+20)
#define ADS114S_BUSY_TIMEOUT_MS  5

#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `spi_eeprom_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `spi_eeprom_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `spi_eeprom_init()`
    bool intr_used;         ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling `spi_eeprom_init()`.
} ads114s_config_t;

/// Context (config and data) of the spi_eeprom
struct ads114s_context_t {
    ads114s_config_t cfg;        ///< Configuration by the caller.
    spi_device_handle_t spi;    ///< SPI device handle
    SemaphoreHandle_t ready_sem; ///< Semaphore for ready signal
};

typedef struct ads114s_context_t  ads114s_context_t;
typedef struct ads114s_context_t* ads114s_handle_t;



static QueueHandle_t gpio_evt_queue = NULL;
static const char TAG[] = "ADS114S";
//  static const char TAG[] = "main";
extern void hexdump3(char *title, void *pack, size_t size) ;
int gpio9_set_to_input_from_spi_cs(void);// 기존 GPIO9(SPI_CS)가 고장이라서 Port를 변경함

extern int ble_send_noti_int(char *id, int value);
extern int ble_send_noti_str(char *id, char* value);
extern int ble_send_noti_float(char *id, float value);

double random_me_double(double min, double max)
{
	double range = 0.0 ;
    double random_value = 0.0;

	range = max - min;
//  //      random_value = min + (range * ((double)rand() / RAND_MAX));
//      random_value = min + (range * ((double)esp_random() / RAND_MAX));
    random_value = min + (range * ((double)(esp_random()%1000) / 1000.0));

	return random_value;

}




static esp_err_t ads114s_wait_done_by_intr(ads114s_context_t* ctx)
{
    xSemaphoreTake(ctx->ready_sem, 0);
    gpio_set_level(ctx->cfg.cs_io, 1);
    gpio_intr_enable(ctx->cfg.miso_io);

    //Max processing time is 5ms, tick=1 may happen very soon, set to 2 at least
    uint32_t tick_to_wait = MAX(ADS114S_BUSY_TIMEOUT_MS / portTICK_PERIOD_MS, 2);
    BaseType_t ret = xSemaphoreTake(ctx->ready_sem, tick_to_wait);
    gpio_intr_disable(ctx->cfg.miso_io);
    gpio_set_level(ctx->cfg.cs_io, 0);

    if (ret != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t ads114s_wait_done_by_polling(ads114s_context_t* ctx)
{
    bool timeout = true;
    gpio_set_level(ctx->cfg.cs_io, 1);
    for (int i = 0; i < ADS114S_BUSY_TIMEOUT_MS * 1000; i ++) {
        if (gpio_get_level(ctx->cfg.miso_io)) {
            timeout = false;
            break;
        }
//          usleep(1);
        vTaskDelay(1);
    }
    gpio_set_level(ctx->cfg.cs_io, 0);
    if (timeout) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t ads114s_wait_done(ads114s_context_t* ctx)
{
    //have to keep cs low for 250ns
//      usleep(1);
    vTaskDelay(1);
    esp_err_t ret = ESP_FAIL;
    if (ctx->cfg.intr_used) {
        ret = ads114s_wait_done_by_intr(ctx);
    } else {
        ret = ads114s_wait_done_by_polling(ctx);
    }
    return ret;
}

static void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((ads114s_context_t*)t->user)->cfg.cs_io);
    gpio_set_level(((ads114s_context_t*)t->user)->cfg.cs_io, 1);
}

static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((ads114s_context_t*)t->user)->cfg.cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((ads114s_context_t*)t->user)->cfg.cs_io);
}

void ready_rising_isr(void* arg)
{
    ads114s_context_t* ctx = (ads114s_context_t*)arg;
    xSemaphoreGive(ctx->ready_sem);
    ESP_EARLY_LOGV(TAG, "ready detected.");
}


esp_err_t spi_ads114s_init(const ads114s_config_t *cfg, ads114s_context_t** out_ctx)
{
    esp_err_t err = ESP_OK;
    if (cfg->intr_used && cfg->host == SPI1_HOST) {
        ESP_LOGE(TAG, "interrupt cannot be used on SPI1 host.");
        return ESP_ERR_INVALID_ARG;
    }

    ads114s_context_t* ctx = (ads114s_context_t*)malloc(sizeof(ads114s_context_t));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }

    *ctx = (ads114s_context_t) {
        .cfg = *cfg,
    };

    spi_device_interface_config_t devcfg = {
//          .command_bits = 10,
        .command_bits = 8,
        .clock_speed_hz = ADS114S_CLK_FREQ,
        .mode = 1,          //SPI mode 0 // CPOL=0, CPHA=1 ( low to high, negative edge )
        /*
         * The timing requirements to read the busy signal from the ads114s cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
//          .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
        .pre_cb = cs_low,
        .post_cb = cs_high,
//          .input_delay_ns = ADS114S_INPUT_DELAY_NS,  //the ads114s output the data half a SPI clock behind.
    };

//  	if ( flag_IS_WEARABLE == 0 ) 
//  	{
//  		gpio9_set_to_input_from_spi_cs(); // GPIO_9 --> GPIO_5
//  		devcfg.spics_io_num = 5 ;  // 9(PIN_NUM_CS)->5
//  	}

    //Attach the ads114s to the SPI bus
    err = spi_bus_add_device(ctx->cfg.host, &devcfg, &ctx->spi);
    if (err != ESP_OK) {
        goto cleanup;
    }

    gpio_set_level(ctx->cfg.cs_io, 0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(ctx->cfg.cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    if (ctx->cfg.intr_used) {
        ctx->ready_sem = xSemaphoreCreateBinary();
        if (ctx->ready_sem == NULL) {
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        gpio_set_intr_type(ctx->cfg.miso_io, GPIO_INTR_POSEDGE);
        err = gpio_isr_handler_add(ctx->cfg.miso_io, ready_rising_isr, ctx);
        if (err != ESP_OK) {
            goto cleanup;
        }
        gpio_intr_disable(ctx->cfg.miso_io);
    }
    *out_ctx = ctx;
    return ESP_OK;

cleanup:
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    if (ctx->ready_sem) {
        vSemaphoreDelete(ctx->ready_sem);
        ctx->ready_sem = NULL;
    }
    free(ctx);
    return err;
}


static void IRAM_ATTR ads114s_nRDY_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGE("gpio_task_example", "ADS114S_nRDY:GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

int spi_ads114s_command(ads114s_context_t* ctx, char command)
{
	char buf_w[20];
	char buf_r[20];
    esp_err_t err;

	memset(buf_w, 0, sizeof(buf_w));
	memset(buf_r, 0, sizeof(buf_r));

//  	ESP_LOGI("spi_ads114s_command", "---------- 0x%02x ------------", command);
	

    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }

//  	buf_w[0] = command; // 1byte만 보낸는 것인데 length를 8로 하지 않고 tx_buffer를 NULL로

    spi_transaction_t t = {
        .cmd = command,
//          .cmd = 0,
        .length = (1)*8,
//          .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);


    spi_device_release_bus(ctx->spi);
    return err;
}

int spi_ads114s_WREG(ads114s_context_t* ctx, char addr, char w_data)
{
	char buf_w[20];
	char buf_r[20];
    esp_err_t err;

	memset(buf_w, 0, sizeof(buf_w));
	memset(buf_r, 0, sizeof(buf_r));

//  //  	ESP_LOGI("spi_ads114s_WREG", "before spi_device_acquire_bus()");
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
//  //  	ESP_LOGI("spi_ads114s_WREG", "after  spi_device_acquire_bus()");
    if (err != ESP_OK) {
        return err;
    }

	buf_w[0] = (addr) | 0x40; // Addr보다 -1한 위치에 Write가 되어서 +1했음 --> 다시 수정
	buf_w[1] = 0; // Datasheet에는 -1하라고 되어있는데 실제는 그렇게 하면 마지막이 읽히지 않는다.
//  	buf_w[1] = 1; // Datasheet에는 -1하라고 되어있는데 실제는 그렇게 하면 마지막이 읽히지 않는다.
	buf_w[2] = w_data;

    spi_transaction_t t = {
        .cmd = buf_w[0],
//          .cmd = 0,
        .length = (1+2)*8, // cmd를 포함한 갯수를 적어야 한다.
//          .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ,
        .tx_buffer = &buf_w[1],
        .rx_buffer = buf_r,
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);

//      if (err == ESP_OK) {
//          err = ads114s_wait_done(ctx);
//      }

//  	ESP_LOGI("spi_ads114s_WREG", "addr(start)=%d, length=%d", (addr & ~(0x20)), 1);
//  //  	hexdump3("WREG:buf_w", buf_w, sizeof(buf_w));
//  //  	hexdump3("WREG:buf_r", buf_r, sizeof(buf_r));
//  //  	hexdump3("WREG:buf_w", buf_w, len+2); // command+len+[values]
//  //  	hexdump3("WREG:buf_r", buf_r, len+2);
//  	hexdump3("WREG:buf_w", buf_w, 3); // command+len+[values]
//  //  	hexdump3("WREG:buf_r", &buf_r[2], len);

    spi_device_release_bus(ctx->spi);
    return err;
}



int spi_ads114s_RREG(ads114s_context_t* ctx, char addr, char* rdata, uint8_t len)
{
	char buf_w[32];
	char buf_r[32];
    esp_err_t err;

	memset(buf_w, 0, sizeof(buf_w));
	memset(buf_r, 0, sizeof(buf_r));

//  //  	ESP_LOGI("spi_ads114s_RREG", "before spi_device_acquire_bus()");
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
//  //  	ESP_LOGI("spi_ads114s_RREG", "after  spi_device_acquire_bus()");
    if (err != ESP_OK) {
        return err;
    }

	buf_w[0] = addr | 0x20 ;
	buf_w[1] = len-1;
//  	buf_w[1] = len; // Datasheet에는 -1하라고 되어있는데 실제는 그렇게 하면 마지막이 읽히지 않는다.
//  			

    spi_transaction_t t = {
        .cmd = buf_w[0],
//          .cmd = 0,
        .length = (len+2)*8, // cmd를 포함한 갯수를 적어야 한다.
//          .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ,
        .tx_buffer = &buf_w[1], //command를 보낸 다음 부터 Buffer에 넣는다
        .rx_buffer = buf_r,
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);

//      if (err == ESP_OK) {
//          err = ads114s_wait_done(ctx);
//      }
    spi_device_release_bus(ctx->spi);

//  	ESP_LOGI("spi_ads114s_RREG", "addr(start)=%d, length=%d", (addr & ~(0x20)), len);
//  //  	hexdump3("RREG:buf_w", buf_w, sizeof(buf_w));
//  //  	hexdump3("RREG:buf_r", buf_r, sizeof(buf_r));
//  //  	hexdump3("RREG:buf_w", buf_w, len+2); // command+len+[values]
//  //  	hexdump3("RREG:buf_r", buf_r, len+2);
//  	hexdump3("RREG:buf_w", buf_w, 2); // command+len+[values]
//  //  	hexdump3("RREG:buf_r", &buf_r[2], len);
//  	hexdump3("RREG:buf_r", &buf_r[1], len);


	memcpy(rdata, &buf_r[1], len);
    return err;
}

int spi_ads114s_RDATA(ads114s_context_t* ctx, char* rdata)
{
	char buf_w[32];
	char buf_r[32];
    esp_err_t err;

	memset(buf_w, 0, sizeof(buf_w));
	memset(buf_r, 0, sizeof(buf_r));

//  //  	ESP_LOGI("spi_ads114s_RDATA", "before spi_device_acquire_bus()");
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
//  //  	ESP_LOGI("spi_ads114s_RDATA", "after  spi_device_acquire_bus()");
    if (err != ESP_OK) {
        return err;
    }

	buf_w[0] = RDATA_OPCODE_CONTROL_COMMAND;

    spi_transaction_t t = {
        .cmd = buf_w[0],
//          .cmd = 0,
        .length = (1+2)*8, // cmd를 포함한 갯수를 적어야 한다.
//          .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ,
        .tx_buffer = &buf_w[1],
        .rx_buffer = buf_r,
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);

//      if (err == ESP_OK) {
//          err = ads114s_wait_done(ctx);
//      }
    spi_device_release_bus(ctx->spi);

//  	ESP_LOGI("spi_ads114s_RDATA", "cmd)=%02x, length=%d", RDATA_OPCODE_CONTROL_COMMAND, 3);
//  	hexdump3("RDATA:buf_w", buf_w, 1); // command+[values[15:0] + val[7:0]) 
//  	hexdump3("RDATA:buf_r", &buf_r[0], 2); //without STATUS & CRC



	//command(여기서는 OPCODE)다음이 바로 buf_r[0]의 시작
	memcpy(rdata, &buf_r[0], 2);
    return err;
}

#define ADS114S06_CH_SEL_0	0x0C
#define ADS114S06_CH_SEL_1	0x1C
#define ADS114S06_CH_SEL_2	0x2C



#define PIN_ADS114S_nRDY	(13)

int gpio9_set_to_input_from_spi_cs(void) // 기존 GPIO9(SPI_CS)가 고장이라서 Port를 변경함
{
    gpio_config_t io_conf;

    // detect Is it Wearable : Static은 Pull-up :10K GPIO_38(MIX_A0) / GPIO_39(MUX_A0)
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL << 9);
    //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//                          0 으로만 읽힌다.
    io_conf.mode = GPIO_MODE_INPUT; //
//      io_conf.mode = direction; //
    //enable pull-up mode
    io_conf.pull_up_en = 0; // 1 --> 0
    io_conf.pull_down_en = 0; //NULL --> 0
    gpio_config(&io_conf);

    return 1;
}


//  void app_main(void)
void spi2_adc_task(void *arg)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL << PIN_ADS114S_nRDY) ; // ADS114S nRDY
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

//  //  	gpio9_set_to_input_from_spi_cs(); // GPIO_9 --> GPIO_5

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
//      //start gpio task
//      xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(ADS114S_nRDY_PIN, ads114s_nRDY_handler, (void*) ADS114S_nRDY_PIN);
//      //hook isr handler for specific gpio pin
//      gpio_isr_handler_add(GPIO_INPUT_IO_1, ads114s_nRDY_handler, (void*) GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(PIN_ADS114S_nRDY);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(ADS114S_nRDY_PIN, ads114s_nRDY_handler, (void*) ADS114S_nRDY_PIN);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    esp_err_t ret;
#ifndef CONFIG_EXAMPLE_USE_SPI1_PINS
    ESP_LOGI(TAG, "Initializing bus SPI%d...", ADS114S_SPI_HOST + 1);
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(ADS114S_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
#else
    ESP_LOGI(TAG, "Attach to main flash bus...");
#endif

    ads114s_config_t ads114s_config = {
        .cs_io = PIN_NUM_CS,
        .host = ADS114S_SPI_HOST,
        .miso_io = PIN_NUM_MISO,
    };

//  	if ( flag_IS_WEARABLE == 0 )  // Static : 시험용은 GPIO9가 고장나서
//  	{
//  		gpio9_set_to_input_from_spi_cs(); // GPIO_9 --> GPIO_5
//  		ads114s_config.cs_io = 5 ;  // 9(PIN_NUM_CS)->5
//  	}
#ifdef CONFIG_EXAMPLE_INTR_USED
    ads114s_config.intr_used = true;
//      gpio_install_isr_service(0);
#endif

    ads114s_handle_t ads114s_handle;

    ESP_LOGI(TAG, "Initializing device...");
    ret = spi_ads114s_init(&ads114s_config, &ads114s_handle);
    ESP_ERROR_CHECK(ret);


	while(1)
	{
		xSemaphoreTake(sema_spi_ads114s, portMAX_DELAY);

	    ESP_LOGI(TAG, "Initializing device... Reset");
		for( int k = 0 ; k < 2 ; k++ )
		{
			ret = spi_ads114s_command(ads114s_handle, (char)(RESET_OPCODE_CONTROL_COMMAND) );
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	
		ret = spi_ads114s_WREG(ads114s_handle, STATUS_ADDR, 0x00);
	    ESP_LOGI(TAG, "Initializing device... Done: FL_POR-->0");
		char rbuf[64];
		for( int k = 0 ; k < 1 ; k++ )
		{
			ESP_LOGW("test", "k=%d", k);
			// PGA(addr:0x03)은 4를 읽으면 0, 모두 읽을 때는 0x14(default)값)이다 .????
			// cmd + len을 보낼때 -1하라고 했는데 그러면 마지막이 읽히지 않는다
		    ret = spi_ads114s_RREG(ads114s_handle, ID_ADDR, rbuf, 5);
	
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}

		//  ret = spi_ads114s_WREG(ads114s_handle, (char)INPMUX_ADDR,   0x5C); // 0x02 // AIN5(3.3V) + AINCOM
		ret = spi_ads114s_WREG(ads114s_handle, (char)INPMUX_ADDR,   0xCC); // 0x02 // AICOM + AINCOM
	
		//  ret = spi_ads114s_WREG(ads114s_handle, (char)PGA_ADDR,  0x04); // 0x03: DELAY :000 / PGA_EN :00 / GAIN 16 : 100
		ret = spi_ads114s_WREG(ads114s_handle, (char)PGA_ADDR,  0x00); // 0x03: DELAY :000 / PGA_EN :00 / GAIN  0 : 000
		//  ret = spi_ads114s_WREG(ads114s_handle, (char)PGA_ADDR,  0x0C); // 0x03: DELAY :000 / PGA_EN :01 / GAIN 16 : 100
	
		ret = spi_ads114s_WREG(ads114s_handle, (char)DATARATE_ADDR, 0x32); // 0x04
		ret = spi_ads114s_WREG(ads114s_handle, (char)REF_ADDR     , 0x3A); // 0x05 // internal ref / always on even in power-down mode
	
		//  ret = spi_ads114s_WREG(ads114s_handle, (char)VBIAS_ADDR,    0x1F); // 0x08: VB_LEVEL:0(1/2) , 
		//  	                                                                   //       VB_AINC : AINCOM connect Disable(0), AIN[5:0]: 011111
		ret = spi_ads114s_WREG(ads114s_handle, (char)VBIAS_ADDR,    0x00); // 0x08: VB_LEVEL:0(1/2) , 
	                                                                   //       VB_AINC : AINCOM connect Disable(0), AIN[5:0]: 000000

		memset(rbuf, 0, sizeof(rbuf));
		ret = spi_ads114s_RREG(ads114s_handle, ID_ADDR, rbuf, 9);
	
	//  	ret = spi_ads114s_command(ads114s_handle, (char)(SLEEP_OPCODE_CONTROL_COMMAND) );
	//  	ret = spi_ads114s_RREG(ads114s_handle, 0, rbuf, 5);
	
		uint16_t adc_val[6];
		memset( adc_val, 0, sizeof(adc_val));
	
		for( int j = 0 ; j < 6 ; j++ )
		{
			char inpmux = (j<<4) | 0x0C;
//  			printf(">>>>>>>>>>>>>>>>>>>. inpmux = 0x%02x<<<<<<<<<<<<<<<<<<<\n", inpmux);
			
			ret = spi_ads114s_WREG(ads114s_handle, (char)INPMUX_ADDR,   inpmux);              
				vTaskDelay( 10 / portTICK_PERIOD_MS);
			ret = spi_ads114s_command(ads114s_handle, (char)(START_OPCODE_CONTROL_COMMAND) ); 
				vTaskDelay(100 / portTICK_PERIOD_MS);
			ret = spi_ads114s_command(ads114s_handle, (char)(STOP_OPCODE_CONTROL_COMMAND) );
				vTaskDelay(100 / portTICK_PERIOD_MS);
	
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			for( int i = 0 ; i < 5 ; i++ )
			{
//  		    	ESP_LOGI(TAG, "================== %3d ====================================", i);
//  	//  			ret = spi_ads114s_RREG(ads114s_handle, STATUS_ADDR, rbuf, 1);
				ret = spi_ads114s_RDATA(ads114s_handle, (char *)&adc_val[j]);
	
		    	ESP_LOGI(TAG, "==================adc_val[%d] %04x ========================", j, (int)htons(adc_val[j]) );
				adc_val[j] = htons(adc_val[j]);
//  				vTaskDelay(100 / portTICK_PERIOD_MS); // Log를 삭제하면 너무 빨라서 모두 0만 나온다
				if( adc_val[j] != 0 )
					break;
				vTaskDelay(1000 / portTICK_PERIOD_MS);
	
			}
		
	//  		ret = spi_ads114s_command(ads114s_handle, (char)(STOP_OPCODE_CONTROL_COMMAND) );
	//  		vTaskDelay(500 / portTICK_PERIOD_MS);
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}

	
		ret = spi_ads114s_command(ads114s_handle, (char)(STOP_OPCODE_CONTROL_COMMAND) );
			vTaskDelay(500 / portTICK_PERIOD_MS);
		ret = spi_ads114s_RREG(ads114s_handle, ID_ADDR, rbuf, 18);
	
		ESP_LOGW("ADC Result", " %04x        %04x       %04x       %04x       %04x       %04x", 
			      adc_val[0], adc_val[1], adc_val[2],
			      adc_val[3], adc_val[4], adc_val[5]);
		ESP_LOGW("ADC Result", " %-8d    %-8d   %-8d   %-8d   %-8d   %-8d", 
			      (uint16_t)adc_val[0], (uint16_t)adc_val[1], (uint16_t)adc_val[2],
			      (uint16_t)adc_val[3], (uint16_t)adc_val[4], (uint16_t)adc_val[5]);
	
	    ESP_LOGI(TAG, "SPI-ADC Read All Channel  finished.");

		xSemaphoreGive(sema_spi_ads114s);

		{
//  		    ESP_LOGI(JSON_TAG, "Serialize.....ADC_Result");
//  		    cJSON *root;
//  		   	root = cJSON_CreateObject();
//  	    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
//  		   	cJSON_AddNumberToObject(root, "ADC_HW_v1(2.5V_ref)_H2S_val",      adc_val[0]);
//  		   	cJSON_AddNumberToObject(root, "ADC_HW_v1(2.5V_ref)_O3_val",       adc_val[1]);
//  		   	cJSON_AddNumberToObject(root, "ADC_HW_v1(2.5V_ref)_CO_val",       adc_val[2]);
//  		   	cJSON_AddNumberToObject(root, "ADC_HW_v1(2.5V_ref)_NO2_val",      adc_val[3]);
//  		   	cJSON_AddNumberToObject(root, "ADC_HW_v1(2.5V_ref)_NH3_val",      adc_val[4]);
//  		   	cJSON_AddNumberToObject(root, "ADC_HW_v1(2.5V_ref)_3.3V_div2_val",adc_val[5]);
//  	
//  		    char *my_json_string = cJSON_Print(root);
//  	
//  		   	ESP_LOGI("FAN", "my_json_string\n%s",my_json_string);
//  			if( flag_IS_WEARABLE == 0 ) //Static Main
//  			{
//  				xSemaphoreTake(sema_uart2, portMAX_DELAY);
//  				write(fd_uart2, my_json_string, strlen(my_json_string));
//  				xSemaphoreGive(sema_uart2);
//  			}
//  			else // Wearable Main
//  			{
//  				xSemaphoreTake(sema_tcp, portMAX_DELAY);
//  				send_to_server(my_json_string, strlen(my_json_string));
//  				xSemaphoreGive(sema_tcp);
//  			}
//  		   	cJSON_Delete(root);

			double H2S_cali_volt = 0 ;
			double  CO_cali_volt = 0 ;
			double  O3_cali_volt = 0 ;
			double NO2_cali_volt = 0 ;

			double NH3_cali_volt = 0 ;
			double NH3_Rs = 0 ;
			double NH3_Ro = 1000 ; //fixed
			double NH3_Rs_divide_by_Ro = 0 ;
			double NH3_Sensitivity = 15 ;
//  			double NH3_slope = -0.013960 ; // fixed
			double NH3_slope = -0.013960 ; // fixed
			double NH3_y_intersect = 0.8351 ; // fixed
			double NH3_log = 0;
			double NH3_cali_ppm = 0;

			double H2S_cali_ppm = 0 ;
			double  CO_cali_ppm = 0 ;
			double  O3_cali_ppm = 0 ;
			double NO2_cali_ppm = 0 ;

			uint8_t S_H2S[10];
			uint8_t S_O3[10];
			uint8_t S_CO[10];
			uint8_t S_NO2[10];

			//---------------------------------------------------------------
			ijoon_get_nvs_str((uint8_t*)"S_H2S", S_H2S);
			ijoon_get_nvs_str((uint8_t*)"S_O3" , S_O3 );
			ijoon_get_nvs_str((uint8_t*)"S_CO" , S_CO );
			ijoon_get_nvs_str((uint8_t*)"S_NO2", S_NO2);

			if( S_H2S[0] != 0 ) { Sensitivity_H2S = atof((char*)S_H2S); }
			if( S_O3 [0] != 0 ) { Sensitivity_O3  = atof((char*)S_O3 ); }
			if( S_CO [0] != 0 ) { Sensitivity_CO  = atof((char*)S_CO ); }
			if( S_NO2[0] != 0 ) { Sensitivity_NO2 = atof((char*)S_NO2); }

			ESP_LOGW("sss", "S_H2S=%s, %.3f, %.2f", S_H2S, atof((char*)S_H2S), Sensitivity_H2S);
			ESP_LOGW("sss", "S_O3 =%s, %.3f, %.2f", S_O3,  atof((char*)S_O3 ), Sensitivity_O3 );
			ESP_LOGW("sss", "S_CO =%s, %.3f, %.2f", S_CO,  atof((char*)S_CO ), Sensitivity_CO );
			ESP_LOGW("sss", "S_NO2=%s, %.3f, %.2f", S_NO2, atof((char*)S_NO2), Sensitivity_NO2);
			//---------------------------------------------------------------

			//---------------------------------------------------------------
			ijoon_get_nvs_str((uint8_t*)"VGAS0_H2S", S_H2S);
			ijoon_get_nvs_str((uint8_t*)"VGAS0_O3" ,S_O3 );
			ijoon_get_nvs_str((uint8_t*)"VGAS0_CO" ,S_CO );
			ijoon_get_nvs_str((uint8_t*)"VGAS0_NO2", S_NO2);

			if( S_H2S[0] != 0 ) { Vgas0_H2S = (2.5*atof((char*)S_H2S))/(1<<16) ; }
			if( S_O3 [0] != 0 ) { Vgas0_O3  = (2.5*atof((char*)S_O3 ))/(1<<16) ; }
			if( S_CO [0] != 0 ) { Vgas0_CO  = (2.5*atof((char*)S_CO ))/(1<<16)  ; }
			if( S_NO2[0] != 0 ) { Vgas0_NO2 = (2.5*atof((char*)S_NO2))/(1<<16) ; }


			ESP_LOGW("sss", "VGAS0_H2S=%s, %.6f, %.6f", S_H2S, atof((char*)S_H2S), Vgas0_H2S);
			ESP_LOGW("sss", "VGAS0_O3 =%s, %.6f, %.6f", S_O3,  atof((char*)S_O3 ), Vgas0_O3 );
			ESP_LOGW("sss", "VGAS0_CO =%s, %.6f, %.6f", S_CO,  atof((char*)S_CO ), Vgas0_CO );
			ESP_LOGW("sss", "VGAS0_NO2=%s, %.6f, %.6f", S_NO2, atof((char*)S_NO2), Vgas0_NO2);
			//---------------------------------------------------------------

//  			M_O3  = (Sensitivity_O3  * TIA_Gain_O3  * (10^-9) * (1e3));
//  			M_CO  = (Sensitivity_CO  * TIA_Gain_CO  * (10^-9) * (1e3));
//  			M_NO2 = (Sensitivity_NO2 * TIA_Gain_NO2 * (10^-9) * (1e3));
			M_H2S = (Sensitivity_H2S   * TIA_Gain_H2S)/(1000000);  //* (10^-9) * (1e3));
			M_O3  = (Sensitivity_O3    * TIA_Gain_O3 )/(1000000);  //* (10^-9) * (1e3));
//  		M_CO  = (Sensitivity_CO    * TIA_Gain_CO )/(1000000);  //* (10^-9) * (1e3));
			M_CO  = (Sensitivity_CO*10 * TIA_Gain_CO )/(1000000);  //* (10^-9) * (1e3));
			M_NO2 = (Sensitivity_NO2   * TIA_Gain_NO2)/(1000000); //* (10^-9) * (1e3));

			ESP_LOGW("sss", "----------------------------------------");
			ESP_LOGW("sss", "M_H2S=%f", M_H2S);
			ESP_LOGW("sss", " M_O3=%f",  M_O3);
			ESP_LOGW("sss", " M_CO=%f",  M_CO);
			ESP_LOGW("sss", "M_NO2=%f", M_NO2);




//  		    H2S_cali_volt = 3.3*(adc_val[0]*(2.5/3.3))/(1<<16);
//  			 O3_cali_volt = 3.3*(adc_val[1]*(2.5/3.3))/(1<<16);
//  			 CO_cali_volt = 3.3*(adc_val[2]*(2.5/3.3))/(1<<16);
//  			NO2_cali_volt = 3.3*(adc_val[3]*(2.5/3.3))/(1<<16);
		    H2S_cali_volt = (adc_val[0]*2.5)/(1<<16);
			 O3_cali_volt = (adc_val[1]*2.5)/(1<<16);
			 CO_cali_volt = (adc_val[2]*2.5)/(1<<16);
			NO2_cali_volt = (adc_val[3]*2.5)/(1<<16);

//  //  			adc_val[4] = 10772; // test
			NH3_cali_volt = ((adc_val[4]*2.5)/(1<<16));
			NH3_Rs        = (( 5 - NH3_cali_volt) * NH3_Ro ) / (NH3_cali_volt);
			NH3_Rs_divide_by_Ro = (NH3_Rs ) / NH3_Ro;
			NH3_log = log10(NH3_Rs_divide_by_Ro/NH3_Sensitivity);
			NH3_cali_ppm = (NH3_log - NH3_y_intersect)/NH3_slope + 1 ;

			ESP_LOGW("sss", "----------------------------------------");
			ESP_LOGW("sss", "NH3_cali_volt=%f",NH3_cali_volt);
			ESP_LOGW("sss", "NH3_Rs=%f",NH3_Rs);
			ESP_LOGW("sss", "NH3_Rs_divide_by_Ro=%f",NH3_Rs_divide_by_Ro);
			ESP_LOGW("sss", "NH3_log=%f",NH3_log);
			ESP_LOGW("sss", "NH3_cali_ppm=%f",NH3_cali_ppm);
			ESP_LOGW("sss", "----------------------------------------");

			ESP_LOGW("sss", "H2S_cali_volt=%f,vgas0_volt=%f", H2S_cali_volt, Vgas0_H2S);
			ESP_LOGW("sss", " O3_cali_volt=%f,vgas0_volt=%f",  O3_cali_volt, Vgas0_O3 );
			ESP_LOGW("sss", " CO_cali_volt=%f,vgas0_volt=%f",  CO_cali_volt, Vgas0_CO );
			ESP_LOGW("sss", "NO2_cali_volt=%f,vgas0_volt=%f", NO2_cali_volt, Vgas0_NO2);


//  			 O3_cali_ppm = (1/M_O3)*(O3_cali_volt-Vgas0_O3);
//  			 CO_cali_ppm = (1/M_CO)*(CO_cali_volt-Vgas0_CO);
//  			NO2_cali_ppm = (1/M_NO2)*(NO2_cali_volt-Vgas0_NO2);
			H2S_cali_ppm = (H2S_cali_volt-Vgas0_H2S)/M_H2S;
			 O3_cali_ppm = ( O3_cali_volt-Vgas0_O3 )/M_O3;
			 CO_cali_ppm = ( CO_cali_volt-Vgas0_CO )/M_CO;
			NO2_cali_ppm = (NO2_cali_volt-Vgas0_NO2)/M_NO2;


			ESP_LOGE("shcho", "O3/NO2는 높게 나와서 /100.0을 함");
			  O3_cali_ppm /= 100.0;
			 NO2_cali_ppm /= 100.0;

			ESP_LOGW("sss", "----------------------------------------");
			ESP_LOGW("sss", "H2S_cali_ppm=%.3f",H2S_cali_ppm);
			ESP_LOGW("sss", " O3_cali_ppm=%.3f", O3_cali_ppm);
			ESP_LOGW("sss", " CO_cali_ppm=%.3f", CO_cali_ppm);
			ESP_LOGW("sss", "NO2_cali_ppm=%.3f",NO2_cali_ppm);



			H2S_cali_ppm = ambient_limit(AMBIENT_H2S, H2S_cali_ppm, 0.002, 0.008);
			 O3_cali_ppm = ambient_limit(AMBIENT_O3,   O3_cali_ppm,  0.002, 0.005);
			 CO_cali_ppm = ambient_limit(AMBIENT_CO,   CO_cali_ppm,  0.2,   0.9);
			NO2_cali_ppm = ambient_limit(AMBIENT_NO2, NO2_cali_ppm, 0.010, 0.020);
			NH3_cali_ppm = ambient_limit(AMBIENT_NH3, NH3_cali_ppm, 0.02,  0.04);

			ESP_LOGW("sss", "-------- some Changed : Min 0.001 --------------------");
			ESP_LOGW("sss", "H2S_cali_ppm=%.3f",H2S_cali_ppm);
			ESP_LOGW("sss", " O3_cali_ppm=%.3f", O3_cali_ppm);
			ESP_LOGW("sss", " CO_cali_ppm=%.3f", CO_cali_ppm);
			ESP_LOGW("sss", "NO2_cali_ppm=%.3f",NO2_cali_ppm);


			ble_send_noti_float("H2S", H2S_cali_ppm);
			ble_send_noti_float("O3",   O3_cali_ppm);
			ble_send_noti_float("CO",   CO_cali_ppm);
			ble_send_noti_float("NO2", NO2_cali_ppm);
			ble_send_noti_float("NH3", NH3_cali_ppm);

		    ESP_LOGI(JSON_TAG, "Serialize.....ADC_Result");
		    cJSON *root;
			char temp[128];
		   	root = cJSON_CreateObject();
	    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
			memset(temp, 0, sizeof(temp)); sprintf(temp, "%.4f", H2S_cali_ppm);
		   	cJSON_AddStringToObject(root, "ADC_HW_v1(2.5V_ref)_H2S_val",   temp   );
			memset(temp, 0, sizeof(temp)); sprintf(temp, "%.4f", O3_cali_ppm);
		   	cJSON_AddStringToObject(root, "ADC_HW_v1(2.5V_ref)_O3_val",   temp   );
			memset(temp, 0, sizeof(temp)); sprintf(temp, "%.3f", CO_cali_ppm);
		   	cJSON_AddStringToObject(root, "ADC_HW_v1(2.5V_ref)_CO_val",   temp   );
			memset(temp, 0, sizeof(temp)); sprintf(temp, "%.4f", NO2_cali_ppm);
		   	cJSON_AddStringToObject(root, "ADC_HW_v1(2.5V_ref)_NO2_val",   temp   );
			memset(temp, 0, sizeof(temp)); sprintf(temp, "%.3f", NH3_cali_ppm);
		   	cJSON_AddStringToObject(root, "ADC_HW_v1(2.5V_ref)_NH3_val",   temp   );

		   	cJSON_AddNumberToObject(root, "val_ADC_HW_v1(2.5V_ref)_H2S_val",      adc_val[0]);
		   	cJSON_AddNumberToObject(root, "val_ADC_HW_v1(2.5V_ref)_O3_val",       adc_val[1]);
		   	cJSON_AddNumberToObject(root, "val_ADC_HW_v1(2.5V_ref)_CO_val",       adc_val[2]);
		   	cJSON_AddNumberToObject(root, "val_ADC_HW_v1(2.5V_ref)_NO2_val",      adc_val[3]);
		   	cJSON_AddNumberToObject(root, "val_ADC_HW_v1(2.5V_ref)_NH3_val",      adc_val[4]);
		   	cJSON_AddNumberToObject(root, "val_ADC_HW_v1(2.5V_ref)_3.3V_div2_val",adc_val[5]);
	
		    char *my_json_string = cJSON_Print(root);
	
		   	ESP_LOGI("FAN", "my_json_string\n%s",my_json_string);
			if( flag_IS_WEARABLE == 0 ) //Static Main
			{
				xSemaphoreTake(sema_uart2, portMAX_DELAY);
				write(fd_uart2, my_json_string, strlen(my_json_string));
				xSemaphoreGive(sema_uart2);
			}
			else // Wearable Main
			{
				xSemaphoreTake(sema_tcp, portMAX_DELAY);
				send_to_server(my_json_string, strlen(my_json_string));
				xSemaphoreGive(sema_tcp);
			}
		   	cJSON_Delete(root);

		}
//  	    while (1) {
//  	        // Add your main loop handling code here.
//  	        vTaskDelay(100);
//  	    }
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}
