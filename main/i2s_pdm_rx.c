/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "driver/i2s_pdm.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "i2s_pdm_example.h"

#include <math.h>
//  #include "i2s_example_pins.h"
//
//
#include <stdio.h>
//  #include "freertos/FreeRTOS.h"
//  #include "freertos/task.h"
#include "freertos/message_buffer.h"
//  #include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>

#include "fft.h" // github.com/fakufaku/esp32-fft

#include <cJSON.h>
#include <string.h>
#include <unistd.h>

uint16_t strongest_k_u16[2];
uint16_t avg_u16[2];
uint16_t peak_u16[2];

//  #define EXAMPLE_PDM_RX_CLK_IO           EXAMPLE_I2S_BCLK_IO1      // I2S PDM RX clock io number
//  #define EXAMPLE_PDM_RX_DIN_IO           EXAMPLE_I2S_DIN_IO1      // I2S PDM RX data in io number
#define EXAMPLE_PDM_RX_CLK_IO           (1)      // I2S PDM RX clock io number
#define EXAMPLE_PDM_RX_DIN_IO           (2)      // I2S PDM RX data in io number

//  #if SOC_I2S_PDM_MAX_RX_LINES == 4
//  	#define EXAMPLE_PDM_RX_DIN1_IO          EXAMPLE_I2S_DIN1_IO1      // I2S PDM RX data line1 in io number
//  	#define EXAMPLE_PDM_RX_DIN2_IO          EXAMPLE_I2S_DIN2_IO1      // I2S PDM RX data line2 in io number
//  	#define EXAMPLE_PDM_RX_DIN3_IO          EXAMPLE_I2S_DIN3_IO1      // I2S PDM RX data line3 in io number
//  #endif

#define EXAMPLE_PDM_RX_FREQ_HZ          16000           // I2S PDM RX frequency

#define GPIO_PDM_LR_SELECT_PIN		(21) 
#define PDM_SELECT_LEFT			(1) // Left : 1 , Right : 0
#define PDM_SELECT_RIGHT		(0) // Left : 1 , Right : 0

/////////////
// USER SETUP

//  #define DEBUG_PRINT_RAW
//  #define DEBUG_PRINT_AVG
//  #define DEBUG_FFT_FULL_BIN_INFO
#define DEBUG_FFT_STRONGEST

//  #define PIN_CLK (5)
//  #define PIN_DATA (35)
//
#define I2S_BITS_PER_SAMPLE_16BIT (16)

//  #define SAMPLE_BIT_SIZE             (I2S_BITS_PER_SAMPLE_16BIT)
#define SAMPLE_BIT_SIZE             (I2S_BITS_PER_SAMPLE_16BIT)
//  #define SAMPLE_RATE_HZ              (44100) // not sure if decimation is considered?
#define SAMPLE_RATE_HZ              (EXAMPLE_PDM_RX_FREQ_HZ) // pdm mic is fixed 16KHz / 16 bit 

#define FFT_BUF_SAMPLES             (2048)   // (4096), <-- does not trigger watchdog ====== triggers watchdog --> (1024) (512)
#define FFT_BUFFERS                 (2)     // double buffering

#define DMA_BUF_BYTES               (1024)  // maximum allowable size is 1024 (by i2s driver implemnetation)

// END USER SETUP
/////////////////

// determine overall buffer size
#if SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_32BIT
	#define FFT_BYTES_PER_SAMPLE        (4)
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_24BIT
	#error "24 bit depth not supported currently"
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_16BIT
	#define FFT_BYTES_PER_SAMPLE        (2)
#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_8BIT
	#define FFT_BYTES_PER_SAMPLE        (1)
#else
	#error "sample bit size incorrectly configured"
#endif

                                     // 2048                2bytes(16 bit)
#define FFT_BUF_BYTES               (FFT_BUF_SAMPLES * FFT_BYTES_PER_SAMPLE)
                                     // 2048* 2      2 ( double buffering) 
#define PDM_BUF_BYTES               (FFT_BUF_BYTES * FFT_BUFFERS)
#define DMA_BUFFERS                 ((FFT_BUF_BYTES * FFT_BUFFERS) / DMA_BUF_BYTES)

                                     // 16KHz                        2048
#define BIN_WIDTH_HZ                ((float)SAMPLE_RATE_HZ / (float)FFT_BUF_SAMPLES) // just a total guess lahmaoh

extern SemaphoreHandle_t sema_tcp ;
extern SemaphoreHandle_t sema_spi_ads114s;
extern SemaphoreHandle_t sema_uart2 ;

extern char my_mac_str[32];
extern int flag_IS_WEARABLE ;
extern int fd_uart2 ;
extern int send_to_server(char *payload, int len);

//shcho from src_nimble_src/led.c
extern void led_on(void) ;
extern void led_off(void) ;

static const char *JSON_TAG = "JSON";

// note: though presented as an unsigned buffer this is really a signed value - functions that interpret 
// these values should use proper casting (to a signed type)
//  uint8_t PDMDataBuffer[PDM_BUF_BYTES];
uint8_t *PDMDataBuffer;

struct _pdm_msg
{
	float strongest_Hz;
	uint16_t avg;
	uint16_t peak;
};

MessageBufferHandle_t buf_idx_msg_handle;
MessageBufferHandle_t buf_send_msg_handle;
const size_t buf_idx_msg_bytes  = sizeof(size_t) + sizeof(size_t); // one size_t for buffer index, another size_t for MessageBuffer overhead
const size_t buf_send_msg_bytes = sizeof(size_t) + sizeof(struct _pdm_msg); // one size_t for buffer index, another size_t for MessageBuffer overhead

// fwd declarations
void disp_buf(uint8_t* buf, size_t length);
void disp_avg_buf(uint8_t* buf, size_t length, uint16_t *avg);


int PDM_LR_select(int pdm_select_val)
{
    gpio_config_t io_conf;

    // detect Is it Wearable : Static은 Pull-up :10K GPIO_38(MIX_A0) / GPIO_39(MUX_A0)
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL << GPIO_PDM_LR_SELECT_PIN);
    //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//                          0 으로만 읽힌다.
//      io_conf.mode = GPIO_MODE_INPUT; //
    io_conf.mode = GPIO_MODE_OUTPUT; //
    //enable pull-up mode
    io_conf.pull_up_en = 1; 
    io_conf.pull_down_en = 0; //NULL --> 0
    gpio_config(&io_conf);

	gpio_set_level(GPIO_PDM_LR_SELECT_PIN, pdm_select_val);

    return 1;
}

void disp_buf(uint8_t* buf, size_t length) {
//  	#if SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_32BIT
//  //  	    int32_t* b = (int32_t*)buf;
//  	    uint32_t* b = (uint32_t*)buf;
//  	#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_24BIT
//  		#error "24 bit depth not supported currently"
//  	#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_16BIT
//  	    int16_t* b = (int16_t*)buf;
	    uint16_t* b = (uint16_t*)buf;
//  	#elif SAMPLE_BIT_SIZE == I2S_BITS_PER_SAMPLE_8BIT
//  //  	    int8_t* b = (int8_t*)buf;
//  	    uint8_t* b = (uint8_t*)buf;
//  	#else
//  		#error "sample bit size incorrectly configured"
//  	#endif

    ESP_LOGW(    "disp_buf    ","length=%d\n", length);
//      for (size_t i = 0; i < (length/FFT_BYTES_PER_SAMPLE); i++)
    for (size_t i = 0; i < 32 ; i +=8 ) 
	{
//          printf("%li\n", b[i]);
		ESP_LOGW("disp_buf    ","[0] %5d [1] %5d [2] %5d [3] %5d [4] %5d [5] %5d [6] %5d [7] %5d",
			b[i+0], b[i+1], b[i+2], b[i+3], b[i+4], b[i+5], b[i+6], b[i+7]);
    }
}


void disp_avg_buf(uint8_t* buf, size_t length, uint16_t *avg_u16) {
//      int64_t acc = 0;
//      int16_t* b = (int16_t*)buf;
//      for (size_t idx = 0; idx < length; idx++) {
//          acc += b[idx];
//      }
//      printf("averrage: %i\n", (int16_t)(acc/length));
    uint64_t acc = 0;
    uint16_t* b = (uint16_t*)buf;
//      for (size_t idx = 0; idx < length; idx++) 
    for (size_t idx = 0; idx < (length/FFT_BYTES_PER_SAMPLE); idx++) 
	{
        acc += b[idx];
    }
    ESP_LOGW("disp_avg_buf","averrage: %i(length=%d)", (int16_t)(acc/ (length/FFT_BYTES_PER_SAMPLE) ), length);
	*avg_u16 = (int16_t)(acc/ (length/FFT_BYTES_PER_SAMPLE) );
    ESP_LOGW("disp_avg_buf","avg_u16: %i (%d)", (uint16_t)*avg_u16, (int16_t)*avg_u16);
}

void calc_avg_peak_buf(uint8_t* buf, size_t length, uint16_t *avg_u16, uint16_t *peak_u16) {
    uint64_t acc = 0;
    uint16_t* b = (uint16_t*)buf;

	*peak_u16 = 0 ; 
    for (size_t idx = 0; idx < (length/FFT_BYTES_PER_SAMPLE); idx++) 
	{
        acc += b[idx];
		if ( *peak_u16 < b[idx] )
		{
			*peak_u16 = b[idx];
		}
    }
	*avg_u16 = (int16_t)(acc/ (length/FFT_BYTES_PER_SAMPLE) );
}

void task_send_JSON (void* arg) 
{
//      size_t buf_idx = 0;
//      uint8_t* buf = NULL;
//  	uint16_t avg_u16[2];
//  	uint16_t peak_u16[2];


//  //  uint8_t PDMDataBuffer[PDM_BUF_BYTES];
//  	PDMDataBuffer = (uint8_t *)calloc(1, PDM_BUF_BYTES);

    while (1) 
	{
		struct _pdm_msg pdm_msg ;
//      	size_t buf_idx = 0;
        size_t rx_bytes = xMessageBufferReceive( buf_send_msg_handle, (void*)(&pdm_msg), sizeof(struct _pdm_msg), portMAX_DELAY );
        assert(rx_bytes == sizeof(struct _pdm_msg));

		{
		    ESP_LOGI(JSON_TAG, "Serialize.....PDM_Result");
		    cJSON *root;
		   	root = cJSON_CreateObject();
	    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
		   	cJSON_AddNumberToObject(root, "PDM_BIN_WIDTH_HZ",      BIN_WIDTH_HZ);
//  		   	cJSON_AddNumberToObject(root, "PDM_Strongest_Hz",      strongest_k[buf_idx]*BIN_WIDTH_HZ);
//  		   	cJSON_AddNumberToObject(root, "PDM_Avg",               avg_u16[buf_idx] );
//  		   	cJSON_AddNumberToObject(root, "PDM_Peak",              peak_u16[buf_idx]);
		   	cJSON_AddNumberToObject(root, "PDM_Strongest_Hz",      pdm_msg.strongest_Hz);
		   	cJSON_AddNumberToObject(root, "PDM_Avg(raw)",               pdm_msg.avg  );
		   	cJSON_AddNumberToObject(root, "PDM_Avg",               20*log10(pdm_msg.avg)  );
		   	cJSON_AddNumberToObject(root, "PDM_Peak",              pdm_msg.peak );

//  			free(pdm_msg);
	
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
	}
}

void task_process (void* arg) 
{
    size_t buf_idx = 0;
    uint8_t* buf = NULL;
//  	uint16_t strongest_k[2];
//  	uint16_t avg_u16[2];
//  	uint16_t peak_u16[2];

//  uint8_t PDMDataBuffer[PDM_BUF_BYTES];
	PDMDataBuffer = (uint8_t *)calloc(1, PDM_BUF_BYTES);

    while (1) 
	{
        size_t rx_bytes = xMessageBufferReceive( buf_idx_msg_handle, (void*)(&buf_idx), sizeof(buf_idx), portMAX_DELAY );
        assert(rx_bytes == sizeof(buf_idx));

        buf = &PDMDataBuffer[buf_idx * FFT_BUF_BYTES]; // get the proper buffer space
//          ESP_LOGW("processor", "received %d bytes, buffer index %d", rx_bytes, buf_idx);
#ifdef DEBUG_PRINT_RAW
        disp_buf(buf, FFT_BUF_BYTES); // print the buffer contents
#endif

#ifdef DEBUG_PRINT_AVG
        disp_avg_buf(buf, FFT_BUF_BYTES, &avg_u16[buf_idx]); // print avg value
#endif
        calc_avg_peak_buf(buf, FFT_BUF_BYTES, &avg_u16[buf_idx], &peak_u16[buf_idx]); // print avg value

#if SAMPLE_BIT_SIZE != I2S_BITS_PER_SAMPLE_16BIT
#error "fft code is currently assuming 16 bit samples - need work to change the sample bit size"
#endif
        // init FFT w/ dynamic memory allocation
        fft_config_t *real_fft_plan = fft_init(FFT_BUF_SAMPLES, FFT_REAL, FFT_FORWARD, NULL, NULL);
        assert(real_fft_plan);
        assert(real_fft_plan->size == FFT_BUF_SAMPLES);

        // prepare input to fft
        int16_t* i16buf = (int16_t*)buf;
        for(size_t k = 0; k < real_fft_plan->size; k++){
            real_fft_plan->input[k] = (float)i16buf[k];
        }

        // perform fft
        fft_execute(real_fft_plan);

        // use the results:
//  #ifdef DEBUG_FFT_FULL_BIN_INFO
//          printf("DC component : %f\n", real_fft_plan->output[0]);  // DC is at [0]
//          for (size_t k = 1 ; k < real_fft_plan->size / 2 ; k++){
//              printf("%d-th freq : %9.2f+j%9.2f\n", k, real_fft_plan->output[2*k], real_fft_plan->output[2*k+1]);
//          }  
//          printf("Middle component : %f\n", real_fft_plan->output[1]);  // N/2 is real and stored at [1]
//  #endif

#ifdef DEBUG_FFT_STRONGEST
        size_t strongest_k = 0;
        float  strongest_val = 0.0;
        for (size_t k = 1 ; k < real_fft_plan->size / 2 ; k++){
            float val = real_fft_plan->output[2*k];
            if(fabs(strongest_val) < fabs(val)){
                strongest_val = val;
                strongest_k = k;
            }
        }
//  	    printf("        avg_u16 ==[0] %6u   [1] %6u -- peak-u16==[0] %6u   [1] %6u\n", avg_u16[0],  avg_u16[1], peak_u16[0], peak_u16[1] );

//  		if( (avg_u16[buf_idx] > 300 ) && ( peak_u16[buf_idx] > 4000 ) )
//  		if( (avg_u16[buf_idx] > 220 ) && ( peak_u16[buf_idx] > 4000 ) )
//  		if( strongest_k > 10 )  // 78.125Hz
		if( strongest_k > 300/BIN_WIDTH_HZ ) 
		{
//  	        ESP_LOGE("i2s_pdm", "strongest(k=%4d, val=%9.3f) \t\t%5.3f(Hz) : avg=%5d peak=%5u\n", strongest_k, strongest_val, 
//  			                                   (float)strongest_k*BIN_WIDTH_HZ, avg_u16[buf_idx], peak_u16[buf_idx] );
	        printf("i2s_pdm: strongest(k=%4d, val=%9.3f) \t\t%5.3f(Hz) : avg=%5d peak=%5u\n", strongest_k, strongest_val, 
			                                   (float)strongest_k*BIN_WIDTH_HZ, avg_u16[buf_idx], peak_u16[buf_idx] );
			strongest_k_u16[buf_idx] = strongest_k;

			led_on();


			struct _pdm_msg *pdm_msg;
			pdm_msg = calloc(1, sizeof(struct _pdm_msg) );

			pdm_msg->strongest_Hz = (float)strongest_k*BIN_WIDTH_HZ;
			pdm_msg->avg          = avg_u16[buf_idx];
			pdm_msg->peak         = peak_u16[buf_idx];
	        size_t tx_bytes = xMessageBufferSend( buf_send_msg_handle, pdm_msg, sizeof(struct _pdm_msg), portMAX_DELAY );
			if( tx_bytes != sizeof(struct _pdm_msg))
			{
				ESP_LOGE("xMessageBufferSend", "failed to send using buf_send_msg_handle");
			}
			free(pdm_msg);
		}
		else
		{
			led_off();
		}
#endif
		

        // clean up output
        fft_destroy(real_fft_plan);
	}
}


static i2s_chan_handle_t i2s_example_init_pdm_rx(void)
{
    i2s_chan_handle_t rx_chan;        // I2S rx channel handler
    /* Setp 1: Determine the I2S channel configuration and allocate RX channel only
     * The default configuration can be generated by the helper macro,
     * but note that PDM channel can only be registered on I2S_NUM_0 */
//      i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
//
//      stella use PDM mic // PDM only supprt on I2S0
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); // if enabled --> i2c2(X), UART2(X)
//      i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER); //reboot
//      i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER); // reboot
//
	//------------------ shcho add  to change dma_num --- start -------------------
	#define I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, i2s_role) { \
	    .id = i2s_num, \
	    .role = i2s_role, \
	    .dma_desc_num = 6, \
	    .dma_frame_num = 240, \
	    .auto_clear_after_cb = false, \
	    .auto_clear_before_cb = false, \
	    .intr_priority = 0, \
	}
	rx_chan_cfg.dma_desc_num = 10;
	//------------------ shcho add  to change dma_num --- end -------------------

    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    /* Step 2: Setting the configurations of PDM RX mode and initialize the RX channel
     * The slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_pdm.h' which can only be used in PDM RX mode.
     * They can help to specify the slot and clock configurations for initialization or re-configuring */
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(EXAMPLE_PDM_RX_FREQ_HZ),
        /* The data bit-width of PDM mode is fixed to 16 */
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = EXAMPLE_PDM_RX_CLK_IO,
//  #if SOC_I2S_PDM_MAX_RX_LINES == 4
//              .dins = {
//                  EXAMPLE_PDM_RX_DIN_IO,
//                  EXAMPLE_PDM_RX_DIN1_IO,
//                  EXAMPLE_PDM_RX_DIN2_IO,
//                  EXAMPLE_PDM_RX_DIN3_IO,
//              },
//  #else
            .din = EXAMPLE_PDM_RX_DIN_IO,
//  #endif
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

	PDM_LR_select(PDM_SELECT_LEFT); //shcho add // 1개만 있어서 LR값이 항상 같다. 왜냐하면 한쪽이 없어서 Driving하는 것이 없기 때문에 

#if CONFIG_IDF_TARGET_ESP32S3
    // Enable all slots for example
//      pdm_rx_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO; //Org // 홀수와 짝수가 항상 같은 값이다. Data만 많아 진다.
//      pdm_rx_cfg.slot_cfg.slot_mask = I2S_PDM_LINE_SLOT_ALL; //Org

    pdm_rx_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    pdm_rx_cfg.slot_cfg.slot_mask = I2S_PDM_SLOT_RIGHT;
//  //      pdm_rx_cfg.slot_cfg.slot_mask = I2S_PDM_SLOT_LEFT;
//      pdm_rx_cfg.slot_cfg.slot_mask = I2S_PDM_SLOT_BOTH;
#endif
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_rx_cfg));

    /* Step 3: Enable the rx channels before reading data */
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    return rx_chan;
}

void i2s_example_pdm_rx_task(void *args)
{
    size_t buf_idx = 0;
//      size_t bytes_read = 0;
    uint8_t* buf = NULL;

    buf_idx_msg_handle = xMessageBufferCreate( buf_idx_msg_bytes );
    assert(buf_idx_msg_handle);

    buf_send_msg_handle = xMessageBufferCreate( buf_send_msg_bytes );
    assert(buf_send_msg_handle);

	TaskHandle_t pdm_send_json_task;
    xTaskCreatePinnedToCore(task_send_JSON, "pdm_JSON", 1024 * 4, NULL, 5, &pdm_send_json_task, 1);

//      xTaskCreate(task_process, "pdm process", 1024 * 4, NULL, 1, NULL); // *2 --> *4
	TaskHandle_t pdm_process_task;
    xTaskCreatePinnedToCore(task_process, "pdm process", 1024 * 4, NULL, 5, &pdm_process_task, 1);

//  	PDM_LR_select(PDM_SELECT_LEFT);

	int count_loop = 0 ;
//      int16_t *r_buf = (int16_t *)calloc(1, EXAMPLE_BUFF_SIZE);
//      assert(r_buf);
    i2s_chan_handle_t rx_chan = i2s_example_init_pdm_rx();

    size_t r_bytes = 0;
    /* ATTENTION: The print and delay in the read task only for monitoring the data by human,
     * Normally there shouldn't be any delays to ensure a short polling time,
     * Otherwise the dma buffer will overflow and lead to the data lost */
    while (1) 
	{
//          bytes_read = 0;
		                              // 2048 * 2 
        buf = &PDMDataBuffer[buf_idx * FFT_BUF_BYTES]; // get the proper buffer space
    	uint16_t *r_buf = (uint16_t *)buf;
        /* Read i2s data */
        if (i2s_channel_read(rx_chan, buf, FFT_BUF_BYTES, &r_bytes, portMAX_DELAY) == ESP_OK) 
		{
			#ifdef DEBUG_PRINT_RAW
			if( count_loop % 10 == 0 ) 
			{
				ESP_LOGI("task rx loop","-------------------------------------------");
	            ESP_LOGI("task rx loop","Read Task: i2s read %d bytes(FFT_BUF_BYTES=%d)-- (%10u)----", 
				                        r_bytes, FFT_BUF_BYTES,count_loop);
	
			    for (size_t i = 0; i < 32 ; i +=8 ) 
				{
		            ESP_LOGI("task rx loop","[0] %5d [1] %5d [2] %5d [3] %5d [4] %5d [5] %5d [6] %5d [7] %5d",
		                   r_buf[i+0], r_buf[i+1], r_buf[i+2], r_buf[i+3], r_buf[i+4], r_buf[i+5], r_buf[i+6], r_buf[i+7]);
			    }
			}
			#endif

	        // signal the processing task which buffer to handle
	        size_t tx_bytes = xMessageBufferSend(buf_idx_msg_handle, &buf_idx, sizeof(size_t), portMAX_DELAY);
			if( tx_bytes != sizeof(size_t))
			{
				ESP_LOGE("xMessageBufferSend", "failed to send");
			}
	        // increment the buffer to use
	        buf_idx++;
	        if(buf_idx >= FFT_BUFFERS)
			{
	            buf_idx = 0;
	        }
        } 
		else {
            ESP_LOGE("PDM Read Task","i2s read failed\n");
        }
		count_loop++;
//          vTaskDelay(pdMS_TO_TICKS(200));
    }

//      free(r_buf);
    vTaskDelete(NULL);
}
