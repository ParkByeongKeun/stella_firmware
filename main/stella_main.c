/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_fat.h"
#include "cmd_system.h"
#include "cmd_i2ctools.h"
#include "driver/i2c_master.h"
//-==================================
//shcho add
#include "esp_chip_info.h"
#include <cJSON.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart_vfs.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <arpa/inet.h>
#include <math.h>
//-==================================
#include "freertos/semphr.h"
#include "esp_mac.h"


#include "nvs.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_event.h"

#include "stella_global.h"

#define STORAGE_NAMESPACE "storage"

// During Cert(PM2008, CM1106, RS9A) : Sensor Connection Used for W5500

    
SemaphoreHandle_t sema_i2c1 = NULL;
SemaphoreHandle_t sema_i2c2 = NULL;
SemaphoreHandle_t sema_uart1 = NULL;
SemaphoreHandle_t sema_uart2 = NULL;
SemaphoreHandle_t sema_tcp = NULL;

static const char *TAG = "i2c-tools";
static uint32_t i2c_frequency = 100 * 1000;
//  static uint32_t i2c_frequency = 100 * 1000;
#define I2C_TOOL_TIMEOUT_VALUE_MS (50)

#define GPIO_INPUT_IO_0     38
#define GPIO_INPUT_IO_1     39
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

int flag_CO2_sensor_OK = 0 ;
int flag_IS_WEARABLE = 0 ;
int flag_USE_W5500_Ethernet = 1 ;

//  i2c_master_dev_handle_t dev_handle_i2c1; // device_address를 그때그때 바꾸려고 했는데
//  											Error  ...add_device() --> ...rm_device()를 해야 한다.

extern int fd_uart2 ;

extern void hexdump3(char *title, void *pack, size_t size) ;
extern void app_main_led_strip_ctrl(void *arg) ;//나중에 R/G/B/W로 변경하자
extern void tcp_client_task(void* arg);
extern int send_to_server(char *payload, int len);
extern void app_main_task_oled(void *arg);

//  //  static gpio_num_t i2c_gpio_sda = CONFIG_EXAMPLE_I2C_MASTER_SDA;
//  //  static gpio_num_t i2c_gpio_scl = CONFIG_EXAMPLE_I2C_MASTER_SCL;
//  static gpio_num_t i2c_gpio_sda = 7; // i2c2 : SDA 16
//  static gpio_num_t i2c_gpio_scl = 6; // i2c2 : SCL 15

static i2c_port_t i2c_port_i2c1 = I2C_NUM_0;
static i2c_port_t i2c_port_i2c2 = I2C_NUM_1;

#if CONFIG_EXAMPLE_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

#define CM1106_CO2_I2C_DEV_ADDR	    0x31 // I2C1
#define PM2008_I2C_DEV_ADDR	        0x28 // I2C1
#define RTC_I2C_DEV_ADDR	        0x32 // I2C1
#define LIGHT_SENSOR_I2C_DEV_ADDR	0x29 // I2C1
#define FAN_CTRL_I2C_DEV_ADDR	    0x2F // I2C1

#define ZMOD4450_I2C_DEV_ADDR	    0x32 // I2C2
#define SHT40_SENSOR_I2C_DEV_ADDR	0x44 // I2C2
#define SGP40_SENSOR_I2C_DEV_ADDR	0x59 // I2C2


int CO2_ppm;
#define CO2_STATUS_NORMAL	(0x00)
int CO2_status;
char CO2_Serial_num_str[50];
char CO2_SW_ver_str[50];
struct _CO2_ppm_packet 
{
	char cmd;
	uint16_t ppm;
	char status;
	char cks;
}__attribute__((packed));

struct _CO2_sn_packet 
{
	char cmd;
	uint16_t digit_5;
	uint16_t digit_4;
	uint16_t digit_3;
	uint16_t digit_2;
	uint16_t digit_1;
	char cks;
}__attribute__((packed));
uint8_t my_mac_factory[20];
char my_mac_str[32];

void app_main_stella_uart1(void);
void app_main_stella_uart2(void);
int send_CM1106_data( struct _CO2_ppm_packet *data );





static const char *JSON_TAG = "JSON";


void test_json(void)
{
    //  I (1756) JSON: Serialize.....
    //  I (1766) JSON: my_json_string
    //  {
    //     	"version":  "v5.2.1-dirty",
    //     	"cores":    2,
    //     	"flag_true":    true,
    //     	"flag_false":   false
    //  }
    //  I (1776) JSON: Deserialize.....
    //  I (1776) JSON: version=v5.2.1-dirty
    //  I (1786) JSON: cores=2
    //  I (1786) JSON: flag_true=1
    //  I (1786) JSON: flag_false=0

    ESP_LOGI(JSON_TAG, "Serialize.....");
    cJSON *root;
    root = cJSON_CreateObject();
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    cJSON_AddStringToObject(root, "version", IDF_VER);
    cJSON_AddNumberToObject(root, "cores", chip_info.cores);
    cJSON_AddTrueToObject(root, "flag_true");
    cJSON_AddFalseToObject(root, "flag_false");

	uint16_t	ttt_1 = htons(100);
    cJSON_AddNumberToObject(root, "test_float_1", (float)(htons(ttt_1)/100.0));
	uint16_t	ttt_2 = htons(103);
    cJSON_AddNumberToObject(root, "test_float_2", (float)(htons(ttt_2)/100.0));
	uint16_t	ttt_3 = htons(110);
    cJSON_AddNumberToObject(root, "test_float_3", (float)(htons(ttt_3)/100.0));
	char tmp_str[100];
    sprintf(tmp_str, "%1.2f", (float)(htons(ttt_3)/100.0));
    cJSON_AddNumberToObject( root, "test_float_4(110/100.0)", atof(tmp_str) );

	uint16_t	ttt_4 = htons(100);
    sprintf(tmp_str, "%1.2f", (float)(htons(ttt_4)/100.0));
    cJSON_AddNumberToObject( root, "test_float_5(100/100.0)", atof(tmp_str) );


    //const char *my_json_string = cJSON_Print(root);
    char *my_json_string = cJSON_Print(root);
    ESP_LOGI(JSON_TAG, "my_json_string\n%s",my_json_string);
    cJSON_Delete(root);

    ESP_LOGI(JSON_TAG, "Deserialize.....");
    cJSON *root2 = cJSON_Parse(my_json_string);
    if (cJSON_GetObjectItem(root2, "version")) {
        char *version = cJSON_GetObjectItem(root2,"version")->valuestring;
        ESP_LOGI(JSON_TAG, "version=%s",version);
    }
    if (cJSON_GetObjectItem(root2, "cores")) {
        int cores = cJSON_GetObjectItem(root2,"cores")->valueint;
        ESP_LOGI(JSON_TAG, "cores=%d",cores);
    }
    if (cJSON_GetObjectItem(root2, "flag_true")) {
        bool flag_true = cJSON_GetObjectItem(root2,"flag_true")->valueint;
        ESP_LOGI(JSON_TAG, "flag_true=%d",flag_true);
    }
    if (cJSON_GetObjectItem(root2, "flag_false")) {
        bool flag_false = cJSON_GetObjectItem(root2,"flag_false")->valueint;
        ESP_LOGI(JSON_TAG, "flag_false=%d",flag_false);
    }
    cJSON_Delete(root2);

    // Buffers returned by cJSON_Print must be freed by the caller.
    // Please use the proper API (cJSON_free) rather than directly calling stdlib free.
    cJSON_free(my_json_string);
}

esp_err_t ijoon_get_nvs_str(uint8_t *key, uint8_t *value)
{
    nvs_handle_t nvs_handle;
    size_t len= 0 ;

    esp_err_t  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("\n", "nvs_open(%s,,,) Failed(%s)", STORAGE_NAMESPACE, esp_err_to_name(err));
    }

    char *blob ;
    if( (err = nvs_get_str(nvs_handle, (char *)key,   NULL, &len)) == ESP_OK )
    {
        blob = (char *)malloc(len);
        if( (err = nvs_get_str(nvs_handle, (char *)key, blob, &len)) == ESP_OK )
        {
//              ESP_LOGI("result nvs_get_str", "nvs_get_str() len=%d, err=%d(%s)(actually read): OK", len, err, esp_err_to_name(err));
//              hexdump3((char *)key, blob, len);

            memcpy((char *)value, blob, len);
        }
        free(blob);
    }
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    return err;

}


esp_err_t  ijoon_set_nvs_str(uint8_t *key, uint8_t *value)
{
    nvs_handle_t nvs_handle;
//      size_t len= 0 ;

    esp_err_t  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("\n", "nvs_open(%s,,,) Failed(%s)", STORAGE_NAMESPACE, esp_err_to_name(err));
//          return err;
    }

    err = nvs_set_str(nvs_handle, (char *)key, (char *)value);

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    return  err;
}

static esp_err_t do_get_nvs_str(int argc, char **argv)
{
    char str[200];
	memset(str, 0, sizeof(str));
    ijoon_get_nvs_str((uint8_t *)argv[1], (uint8_t *)str);
    hexdump3( argv[1], str, sizeof(str));
    return 0;
}

void register_nvs_get_str(void)
{
    const esp_console_cmd_t cmd = {
        .command = "get_nvs_str",
        .help = "get_nvs_str key ",
        .hint = NULL,
        .func = do_get_nvs_str,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static esp_err_t do_set_nvs_str(int argc, char **argv)
{
    char str[200];

	memset(str, 0, sizeof(str));
    ijoon_set_nvs_str((uint8_t *)argv[1], (uint8_t *)argv[2]);

    ijoon_get_nvs_str((uint8_t *)argv[1], (uint8_t *)str);
    hexdump3( argv[1] , str, sizeof(str));
    return 0;
}

void register_nvs_set_str(void)
{
    const esp_console_cmd_t cmd = {
        .command = "set_nvs_str",
        .help = "set_nvs_str key value",
        .hint = NULL,
        .func = do_set_nvs_str,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_EXAMPLE_STORE_HISTORY

char calc_PM2008_cks(uint8_t *data, int len)
{
	int cks = 0;
	for( int  i = 0 ; i < len-1 ; i ++)
	{
		cks ^= data[i];
	}
	ESP_LOGW("calc_PM2008_cks", "cs=0x%02x", (char) cks);
	return (char)cks;
	
}
char calc_CO2_cks(uint8_t *data, int len)
{
	int sum = 0;
	for( int  i = 0 ; i < len-1 ; i ++)
	{
		sum += data[i];
	}
	sum *= -1;
	ESP_LOGW("calc_CO2_cks", "cs=0x%02x", (char) sum);
	return (char)sum;
	
}


//  int get_CO2_ppm( int *ppm)
int get_CO2_ppm( struct _CO2_ppm_packet *CO2_ppm_packet)
{
//  	static int Is_1st = 1 ; 
//  	static i2c_master_dev_handle_t dev_handle_i2c1;
	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;
	int len = sizeof(struct _CO2_ppm_packet);

	int data_addr = 0x01; //cmd
	int8_t cks = 0;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
        return 1;
    }

//  	dev_handle_i2c1->device_address = CM1106_CO2_I2C_DEV_ADDR; // Error
	int loop_count = 0;
CO2_ppm_retry:
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&data_addr, 1, 
	                                 (uint8_t *)CO2_ppm_packet, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) 
	{
		// 1. Power Off --> On
		// 2. All 0x00 (include CS)-->
		// 3. status 0x01 : preheating
		// 4. status 0x00 : Normal
		
		hexdump3("CO2_ppm: i2cget -c 0x31 -r 0x02 -l 5", (char *)CO2_ppm_packet, len);

		if ( CO2_ppm_packet->cmd != data_addr  )
		{

			ESP_LOGW("shcho", "CM1106 reply old cmd: retry again( sleep 2): cmd 0x01");
			loop_count++;
        	vTaskDelay(2000 / portTICK_PERIOD_MS);

			if( loop_count > 10 )
			{
				ESP_LOGW("shcho", "CM1106 retry timeout : return -1");
				return -1;
			}

			goto CO2_ppm_retry;
		}

		if(   ( CO2_ppm_packet->cmd == 0x00) 
		   && ( CO2_ppm_packet->ppm == 0  )
		   && ( CO2_ppm_packet->status == 0 )
		   && ( CO2_ppm_packet->cks == 0 ))
		{
			CO2_status = -10;
			ESP_LOGI("shcho", "CM1106 :Power On : All zero");
			if( loop_count > 100  )
			{
				ESP_LOGE("shcho", "CM1106 :retry Time(All zero)");
				return -10;
			}

			loop_count++;
        	vTaskDelay(2000 / portTICK_PERIOD_MS);

			goto CO2_ppm_retry;
		}
		else if ( CO2_ppm_packet->status != 0 )
		{
			ESP_LOGI("shcho", "CM1106 :Power On : Status is not Normal ");
			if( loop_count > 100  )
			{
				ESP_LOGE("shcho", "CM1106 :retry Time(Status is not Normal)");
				return CO2_ppm_packet->status;
			}

			loop_count++;
        	vTaskDelay(2000 / portTICK_PERIOD_MS);
			CO2_status = CO2_ppm_packet->status;
			goto CO2_ppm_retry;
		}


		cks = calc_CO2_cks((uint8_t *)CO2_ppm_packet, len);
		if( (char)cks != (char)CO2_ppm_packet->cks )
		{
			ESP_LOGE("shcho", "get_CO2_ppm cks differ(0x%02x vs. 0x%02x)", (char)cks, CO2_ppm_packet->cks);
		}
		if( CO2_ppm_packet->cmd != 0x01 )
		{
			ESP_LOGE("shcho", "get_CO2_ppm reply differ(%02x vs. %02x)", 0x01, CO2_ppm_packet->cmd);
		}

		ESP_LOGI("shcho", "CO2_ppm = %d ppm (status = %02x)", htons(CO2_ppm_packet->ppm), CO2_ppm_packet->status);
		ESP_LOGI("shcho", "		0: Normal(Preheating이 아니고 설명에 오류)");
		ESP_LOGI("shcho", "		1: Preheating (Normal operation이 아니고 설명에 오류)");
		ESP_LOGI("shcho", "		2: Operating trouble : Power가 Off->On될때");
		ESP_LOGI("shcho", "		3: Out of FS ");
		ESP_LOGI("shcho", "		5: Non calibrated");

    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
//      free(data);
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
        return 1;
    }
    return 0;
}

int get_CO2_SW_ver(char *sw_ver)
{
//  	static int Is_1st = 1 ; 
//  	static i2c_master_dev_handle_t dev_handle_i2c1;
	char tmp_str[100];
	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;
	int len = 13 ;// 고정

	int data_addr = 0x1E; //cmd
	int8_t cks = 0;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
        return 1;
    }

	int loop_count = 0 ;
CO2_get_SW_ver_retry :
	memset(tmp_str, 0, sizeof(tmp_str));
//  	dev_handle_i2c1->device_address = CM1106_CO2_I2C_DEV_ADDR; // Error
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&data_addr, 1, 
	                                 (uint8_t *)tmp_str, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) 
	{
		hexdump3("CO2_SW_ver: i2cget -c 0x31 -r 0x1E -l 13", (char *)tmp_str, len);
		if ( tmp_str[0] != data_addr  )
		{

			ESP_LOGW("shcho", "CM1106 reply old cmd: retry again( sleep 2)  cmd 0x1E");
			loop_count++;
        	vTaskDelay(2000 / portTICK_PERIOD_MS);

			if( loop_count > 10 )
			{
				ESP_LOGW("shcho", "CM1106 retry timeout : return -1");
				return -1;
			}

			goto CO2_get_SW_ver_retry;
		}

		cks = calc_CO2_cks((uint8_t *)tmp_str, len);
		if( (char)cks != (char)tmp_str[len-1] )
		{
			ESP_LOGE("shcho", "get_CO2_ppm cks differ(0x%02x vs. 0x%02x)", (char)cks, tmp_str[len-1]);
			return -20;
		}
		else
		{
			memcpy(sw_ver, &tmp_str[1], len -2); // exclude cmd + cks
			return 0;
		}

    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
//      free(data);
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
        return -20;
    }
	return 0;
}


int get_CO2_Serial_num(char *sn)
{
//  	static i2c_master_dev_handle_t dev_handle_i2c1;
//  	char tmp_str[100];
	struct _CO2_sn_packet CO2_sn_packet;
	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;
	int len = sizeof(CO2_sn_packet) ;// 고정:12

	int data_addr = 0x1F; //cmd
	int8_t cks = 0;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
        return 1;
    }

	int loop_count = 0 ;
CO2_get_Serial_num_retry :
	memset((char *)&CO2_sn_packet, 0, sizeof(CO2_sn_packet));
//  	dev_handle_i2c1->device_address = CM1106_CO2_I2C_DEV_ADDR; // Error
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&data_addr, 1, 
	                                 (uint8_t *)&CO2_sn_packet, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) 
	{
		hexdump3("CO2_Serial_num: i2cget -c 0x31 -r 0x1F -l 12", (char *)&CO2_sn_packet, len);
		if ( CO2_sn_packet.cmd != data_addr  )
		{
			ESP_LOGW("shcho", "CM1106 reply old cmd: retry again( sleep 2) cmd 0x1F");
			loop_count++;

			if( loop_count > 10 )
			{
				ESP_LOGW("shcho", "CM1106 retry timeout : return -1");
				return -1;
			}

        	vTaskDelay(2000 / portTICK_PERIOD_MS);
			goto CO2_get_Serial_num_retry;
		}

		cks = calc_CO2_cks((uint8_t *)&CO2_sn_packet, len);
		if( (char)cks != (char)CO2_sn_packet.cks )
		{
			ESP_LOGE("shcho", "get_CO2_ppm cks differ(0x%02x vs. 0x%02x)", (char)cks, CO2_sn_packet.cks);
		}
		else
		{
			sprintf(sn, "%01d %03d %04d %04d %04d",
								htons(CO2_sn_packet.digit_5),
								htons(CO2_sn_packet.digit_4),
								htons(CO2_sn_packet.digit_3),
								htons(CO2_sn_packet.digit_2),
								htons(CO2_sn_packet.digit_1));
			return 0;
		}

    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
//      free(data);
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
        return -20;
    }
	return 0;
}

//  void console_cli(void *arg)
//  {
//      esp_console_repl_t *repl = NULL;
//      esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
//  
//  #if CONFIG_EXAMPLE_STORE_HISTORY
//      initialize_filesystem();
//      repl_config.history_save_path = HISTORY_PATH;
//  #endif
//      repl_config.prompt = "i2c-tools>";
//  
//      // install console REPL environment
//  #if CONFIG_ESP_CONSOLE_UART
//      esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
//      ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
//  #elif CONFIG_ESP_CONSOLE_USB_CDC
//      esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
//      ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
//  #elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
//      esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
//      ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
//  #endif
//  
//  //      i2c_master_bus_config_t i2c_bus_config_i2c1 = {
//  //          .clk_source = I2C_CLK_SRC_DEFAULT,
//  //          .i2c_port = i2c_port_i2c1,
//  //          .scl_io_num = 6 , //i2c_gpio_scl,
//  //          .sda_io_num = 7,  //i2c_gpio_sda,
//  //          .glitch_ignore_cnt = 7,
//  //          .flags.enable_internal_pullup = true,
//  //      };
//  //  
//  //      i2c_master_bus_config_t i2c_bus_config_i2c2 = {
//  //          .clk_source = I2C_CLK_SRC_DEFAULT,
//  //          .i2c_port = i2c_port_i2c2,
//  //          .scl_io_num = 15 , //i2c_gpio_scl,
//  //          .sda_io_num = 16,  //i2c_gpio_sda,
//  //          .glitch_ignore_cnt = 7,
//  //          .flags.enable_internal_pullup = true,
//  //      };
//  //  
//  //  //  	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c1, &tool_bus_handle));
//  //      ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c1, &tool_bus_handle_i2c1));
//  //      ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c2, &tool_bus_handle_i2c2));
//  
//      register_i2ctools();
//  
//      printf("\n ==============================================================\n");
//      printf(" |             Steps to Use i2c-tools                         |\n");
//      printf(" |                                                            |\n");
//      printf(" |  1. Try 'help', check all supported commands               |\n");
//      printf(" |  2. Try 'i2cconfig' to configure your I2C bus              |\n");
//      printf(" |  3. Try 'i2cdetect' to scan devices on the bus             |\n");
//      printf(" |  4. Try 'i2cget' to get the content of specific register   |\n");
//      printf(" |  5. Try 'i2cset' to set the value of specific register     |\n");
//      printf(" |  6. Try 'i2cdump' to dump all the register (Experiment)    |\n");
//      printf(" |                                                            |\n");
//      printf(" ==============================================================\n\n");
//  
//      // start console REPL
//      ESP_ERROR_CHECK(esp_console_start_repl(repl));
//  }
//

// The code says that calling uxTaskGetSystemState directly
// rather than VTaskList() is preferred
static int do_tasks_info(int argc, char **argv) {

    const size_t bytes_per_task = 45; /* see vTaskList description */
    // config file has max name length = 24
    //    see component config -> FreeRTOS ->
    // status is a single char. one byte
    // current priority is at most a two digit number. two bytes.
    // StackHighWaterMark is at most four digit numbers. four bytes
    // task number is two digits. two bytes
    // affinity is sign indicator and one digit. two bytes
    // five tabs. five bytes.
    // carriage return line feed. two bytes
    // 24+1+2+4+2+2+5+2 = 42. Set to 45 just to give some spare

    printf("heap size before malloc %ld\n", esp_get_free_heap_size());
    char *task_list_buffer = malloc(uxTaskGetNumberOfTasks() * bytes_per_task);
    if (task_list_buffer == NULL) {
        ESP_LOGE("TASK_INFO", "failed to allocate buffer for vTaskList output");
        return 1;
    }
    printf("heap size after malloc %ld\n", esp_get_free_heap_size());
    fputs("Task Name\t\tStatus\tPrio\tHWM\tTask#", stdout);
#ifdef CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
    fputs("\tAffinity", stdout);
#endif
    fputs("\n", stdout);
    vTaskList(task_list_buffer);
    fputs(task_list_buffer, stdout);
    free(task_list_buffer);
    printf("heap size after free %ld\n", esp_get_free_heap_size());
    return 0;
}

static int  register_view_tasks()
{
    const esp_console_cmd_t cmd = {
        .command = "task",
        .help = "View Task INFO ",
        .hint = NULL,
        .func = do_tasks_info,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}

static int do_esp32_restart(int argc, char **argv) {

	ESP_LOGE("shcho", "ESP32-S3 reboot after 2 secs");
	ESP_LOGE("shcho", "ESP32-S3 reboot after 2 secs");
	ESP_LOGE("shcho", "ESP32-S3 reboot after 2 secs");
	ESP_LOGE("shcho", "ESP32-S3 reboot after 2 secs");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
	esp_restart();
    return 0;
}

static int  register_restart_cmd()
{
    const esp_console_cmd_t cmd = {
        .command = "reboot",
        .help = "reboot ESP32 Task INFO ",
        .hint = NULL,
        .func = do_esp32_restart,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}

int do_fan_report(void)
{
	// FAN Controller Resigter Read
	int chip_addr = FAN_CTRL_I2C_DEV_ADDR;

	int len = 1 ;
	//  i2cget -c 0x2f -r 0x30 -l 1
	int data_addr = 0x30; 

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

	i2c_master_dev_handle_t dev_handle_i2c1;
	if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
		return 1;
	}

	char val;
	char *mode = "PWM Duty";
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&data_addr, 1, 
	                                 (uint8_t *)&val, len, I2C_TOOL_TIMEOUT_VALUE_MS);
//  	float val_percent = floorf((( val * 100.0 )+0.5) / 255.0) ; 
	float val_percent = ceil( ( val * 100.0 ) / 255.0) ; 
	ESP_LOGI("FAN", "fan val=%02x(percent = %3f)\n", val, val_percent);

    if (ret == ESP_OK) 
	{
	    ESP_LOGI(JSON_TAG, "Serialize.....Fan");
	    cJSON *root;
	   	root = cJSON_CreateObject();
	   	cJSON_AddStringToObject(root, "FAN_Mode",   mode);
	   	cJSON_AddNumberToObject(root, "FAN_PWM_percent",  val_percent);
	
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
	else if (ret == ESP_ERR_TIMEOUT) 
	{
        ESP_LOGW(TAG, "Bus is busy");
    } 
	else 
	{
        ESP_LOGW(TAG, "Read failed");
    }

    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
        return 1;
    }

	return 0;
	// ===========================================

}

static int do_esp32_fan_ctrl(int argc, char **argv) 
{
	int chip_addr = FAN_CTRL_I2C_DEV_ADDR;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) 
	{
        return 1;
    }

	int val = ( atoi(argv[1]) * 255 ) / 100 ; 
//  	char str[20];
//  	memset(str, 0, sizeof(str));
//  	sprintf(str, argv[1], strlen(argv[1]));
//  	ESP_LOGW("fan value", "%s(%) = %02x", str, val ); // Type Conversion Error ?????
	ESP_LOGW("fan value", "%s(percent) = %02x", argv[1], val );

	char data[3] ;
	data[0] = 0x2f;
	data[1] = 0x30;
	data[2] = (char)val;

	hexdump3("FAN Duty Change", data, sizeof(data));


    esp_err_t ret = i2c_master_transmit(dev_handle_i2c1, 
	                                    (uint8_t *)data, 
										sizeof(data), 
										I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Write OK : FAN_Ctrl");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy: FAN_Ctrl");
    } else {
        ESP_LOGW(TAG, "Write Failed: FAN_Ctrl");
    }

//  //  	do_fan_report(val, "PWM Duty");
//  //  	직접 읽어서 처리하도록 함
//  	do_fan_report(); //따로 주기적으로 보내도록 함
	

//      free(data);
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
        return -20;
    }
	return 0;
}

static int  register_fan_ctrl()
{
    const esp_console_cmd_t cmd = {
        .command = "fan",
        .help = "fan control ( 0 ~ 100 %)",
        .hint = NULL,
        .func = do_esp32_fan_ctrl,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}

static int do_get_CO2(int argc, char **argv) 
{
	int ret = 0;

	// 1. CO2_SW_ver ==========================================
	ESP_LOGW("shcho", " get_CO2_SW_ver");
	memset(CO2_SW_ver_str, 0, sizeof(CO2_SW_ver_str));
	ret = get_CO2_SW_ver( CO2_SW_ver_str );
	if( ret == 0 )  // OK
	{
		ESP_LOGI("shcho", "CO2 Sensor SW_Ver=%s", CO2_SW_ver_str);
	}

	// 2. CO2_Serial_num  ==========================================
	ESP_LOGW("shcho", " get_CO2_Serial_num");
	memset(CO2_Serial_num_str, 0, sizeof(CO2_Serial_num_str));
	ret = get_CO2_Serial_num( CO2_Serial_num_str );
	if( ret == 0 )  // OK
	{
		ESP_LOGI("shcho", "CO2 Serial_num=%s", CO2_Serial_num_str);
	}

	// 3. CO2_ppm  ==========================================
	ESP_LOGW("shcho", "get_CO2_ppm");
	CO2_ppm = 0 ;
//  	ret = get_CO2_ppm( &CO2_ppm );
//
	struct _CO2_ppm_packet CO2_ppm_packet;
	ret = get_CO2_ppm( &CO2_ppm_packet ) ;
	CO2_ppm = htons(CO2_ppm_packet.ppm);




	//-------------------------------------------------------------------------
	switch( ret  )
	{
		case -10 : // Power On
			ESP_LOGI("shcho","CO2 Sensor(CM1106) : Power On");
			break;
		case 1 : //Preheating
			ESP_LOGI("shcho","CO2 Sensor(CM1106) : Preheating");
			break;
	}
	ESP_LOGW("shcho", "       CO2 Sensor SW_Ver       : %s", CO2_SW_ver_str);
	ESP_LOGW("shcho", "       CO2 Sensor Serial_num   : %s", CO2_Serial_num_str);
	ESP_LOGW("shcho", "-------------- CO2 ppm         : %d ppm ------------------", CO2_ppm);
	ESP_LOGI("shcho", "-------------- CO2 ppm wait - every 20 secs ------------------");

	if( CO2_ppm_packet.status == 0x00 ) 
	{
		send_CM1106_data( &CO2_ppm_packet ); 	
	}
	//-------------------------------------------------------------------------

    return 0;
}


static int  register_get_CO2()
{
    const esp_console_cmd_t cmd = {
        .command = "get_CO2",
        .help = "get_CO2 sensor : val , sw_ver, sn",
        .hint = NULL,
        .func = do_get_CO2,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}



struct _PM2008_data 
{
	char header;
	char len;
	char status;
	uint16_t meas_mode;
	uint16_t calib_coff;
	uint16_t pm1_0_grimm;
	uint16_t pm2_5_grimm;
	uint16_t pm10_0_grimm;
	uint16_t pm1_0_tsi;
	uint16_t pm2_5_tsi;
	uint16_t pm10_0_tsi;
	uint16_t num_0_3um;
	uint16_t num_0_5um;
	uint16_t num_1_0um;
	uint16_t num_2_5um;
	uint16_t num_5_0um;
	uint16_t num_10_0um;
	uint8_t cks;
}__attribute__((packed));

#define PM2008_CMD_CLOSE					(1)
#define PM2008_CMD_OPEN_SINGLE				(2)
#define PM2008_CMD_SETUP_CONTINUOUS			(3)
#define PM2008_CMD_SETUP_TIMING_MEASURE		(4)
#define PM2008_CMD_SETUP_DYNAMIC_MEASURE	(5)
#define PM2008_CMD_SETUP_CALIB_COFF			(6)

struct _PM2008_set_mode
{
	char i2c_dev_addr;
	char header;
	char len;
//  		"Control command of the sensor as:
//  		Close measurement: 1
//  		Open single measurement: 2
//  		Set up continuously measurement: 3 (default mode)
//  		Set up timing measurement: 4
//  		Set up dynamic measurement: 5
//  		Set up calibration coefficient: 6"
	char cmd ; // 
	uint16_t data_16bit;
	char rsvd ; // 
	char cks ;
}__attribute__((packed));

int send_CM1106_data( struct _CO2_ppm_packet *data )
{
	if( data->status == 0 )
	{
	    ESP_LOGI(JSON_TAG, "Serialize.....CM1106");
	    cJSON *root;
    	root = cJSON_CreateObject();
    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
    	cJSON_AddNumberToObject(root, "CO2_ppm",       htons(data->ppm) );
    	cJSON_AddNumberToObject(root, "CO2_status",    data->status );
    	cJSON_AddStringToObject(root, "CO2_Serial_num",CO2_Serial_num_str);
    	cJSON_AddStringToObject(root, "CO2_SW_ver",    CO2_SW_ver_str);

	    char *my_json_string = cJSON_Print(root);

    	ESP_LOGI("CM1106", "my_json_string\n%s",my_json_string);
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
	else
	{
//  		"0 -->1 : Preheating;  --> 1이 아닌가?
//  		1 -->0: Normal operation;  --? 0이 아닌가?
//  		2: Operating trouble; 
//  		3: Out of FS , 
//  		5: Non calibrated
//  		이상하네
//  		CO2 measuring result: DF 0 ] 256 DF 1 ], Fixed output is 550ppm during preheating period
//  		Status bit
//  		DF 2 ]]: Preheating; 1: Normal operation; 2: Operating trouble; 3: Out of FS , 5: Non calibrated"
		ESP_LOGE("CM1106         ", "0:Normal , 1 : Preheating, 2: Operation trouble, 3, Out of FS , 5 : Not Calibrated");
		ESP_LOGE("CM1106         ", "status is not normal: 0x%02x", data->status);
	}

	return 1;
}


int send_PM2008_data( struct _PM2008_data *data )
{
	if( ( data->status == 0x80 ) // 
	 || (  data->status == 0x02 )) // 
	{
		ESP_LOGI("status         ", "0x%02x( should be 0x80 at mode 4 / @ mode 3 :just read value",  data->status    );
		ESP_LOGI("meas_mode      ", "0x%02x(I set to 3(continuous) // 4(timing measuring )",          htons(data->meas_mode) );
		ESP_LOGI("meas_calib_coff", "%1.2f(I set to 100(1.0)",          (float)(htons(data->calib_coff)/100.0) );
		ESP_LOGI("pm1_0_grimm    ", "%d (ug/m^3(GRIMM)",                          htons(data->pm1_0_grimm) );
		ESP_LOGI("pm2_5_grimm    ", "%d (ug/m^3(GRIMM)",                          htons(data->pm2_5_grimm) );
		ESP_LOGI("pm10_0_grimm   ", "%d (ug/m^3(GRIMM)",                          htons(data->pm10_0_grimm ));

	    ESP_LOGI(JSON_TAG, "Serialize.....RS9A");
	    cJSON *root;
    	root = cJSON_CreateObject();
    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
    	cJSON_AddNumberToObject(root, "PM2008_Status",       data->status );
    	cJSON_AddNumberToObject(root, "PM2008_Measure_mode", htons(data->meas_mode) );

		char tmp_str[10];
    	sprintf(tmp_str, "%1.2f", (float)(htons(data->calib_coff)/100.0));
    	cJSON_AddNumberToObject( root, "PM2008_Cali_coff",  atof(tmp_str) );

//      	cJSON_AddNumberToObject(root, "PM2008_Cali_coff", 0.7 ); // test
    	cJSON_AddNumberToObject(root, "PM2008_PM1.0_GRIMM",  htons(data->pm1_0_grimm) );
    	cJSON_AddNumberToObject(root, "PM2008_PM2.5_GRIMM",  htons(data->pm2_5_grimm) );
    	cJSON_AddNumberToObject(root, "PM2008_PM10_GRIMM",   htons(data->pm10_0_grimm) );

	    char *my_json_string = cJSON_Print(root);

    	ESP_LOGI("PM2008", "my_json_string\n%s",my_json_string);

		if( flag_IS_WEARABLE == 0 ) //Static Main
		{
			xSemaphoreTake(sema_uart2, portMAX_DELAY);
			write(fd_uart2, my_json_string, strlen(my_json_string));
			xSemaphoreGive(sema_uart2);
		}
		else
		{
			xSemaphoreTake(sema_tcp, portMAX_DELAY);
			send_to_server(my_json_string, strlen(my_json_string));
			xSemaphoreGive(sema_tcp);
		}

    	cJSON_Delete(root);

	}
	else
	{
		ESP_LOGE("PM2008         ", "status is not normal: 0x%02x", data->status);
	}

	return 1;
}

static int set_PM2008_mode(int cmd, uint16_t value) 
{
//  	1 0  		// Close
//  	2 180       // Open Single Measurement
//  	3 65535     // continuous mode
//  	4 180       // timing measurement
//  	5 ??        // dunamic measurement
//  	6 100       // calibration Coff

	int chip_addr = PM2008_I2C_DEV_ADDR;

//  	struct _PM2008_data PM2008_data;

	struct _PM2008_set_mode PM2008_set_mode;

//  	int8_t cks = 0;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
        return 1;
    }

	PM2008_set_mode.i2c_dev_addr = PM2008_I2C_DEV_ADDR;
	PM2008_set_mode.header = 0x16;
	PM2008_set_mode.len = 0x7;
	PM2008_set_mode.cmd = (char)cmd;
	PM2008_set_mode.data_16bit = htons(value);
	PM2008_set_mode.rsvd = 0;
	PM2008_set_mode.cks = calc_PM2008_cks( (uint8_t *)&PM2008_set_mode.header, sizeof(PM2008_set_mode)-1);

	hexdump3("shcho set_PM2008_mode", (char*)&PM2008_set_mode, sizeof(PM2008_set_mode));


//  PM2008_data_retry:
//  	dev_handle_i2c1->device_address = PM2008_I2C_DEV_ADDR;
    esp_err_t ret = i2c_master_transmit(dev_handle_i2c1, (uint8_t *)&PM2008_set_mode, sizeof(PM2008_set_mode), I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Write OK");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Write Failed");
    }

    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
        return 1;
    }
    return 0;
}




static int do_get_PM2008(int argc, char **argv) 
{
//  	static int Is_1st = 1 ; 
//  	static i2c_master_dev_handle_t dev_handle_i2c1;
	int chip_addr = PM2008_I2C_DEV_ADDR;

	struct _PM2008_data PM2008_data;

	int len = sizeof(PM2008_data);
	int flag_skip = 0;

	//  i2cget -c 0x28 -l 32
	int data_addr = -1; 
	int8_t cks = 0;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

	i2c_master_dev_handle_t dev_handle_i2c1;
	if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
		return 1;
	}

	int loop_count = 0;
PM2008_data_retry:
//  	dev_handle_i2c1->device_address = PM2008_I2C_DEV_ADDR;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&data_addr, 1, 
	                                 (uint8_t *)&PM2008_data, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) 
	{
		// 1. Power Off --> On
		// 2. All 0x00 (include CS)-->
		// 3. status 0x01 : preheating
		// 4. status 0x00 : Normal
		
		hexdump3("PM2008_data: i2cget -c 0x28 -l 32 를 구현", (char *)&PM2008_data, len);

		if ( PM2008_data.header != 0x16 || PM2008_data.len != 0x20 )
		{

			ESP_LOGW("shcho", "PM2008 data reply Error : 0x16 0x20 ..... ");
			loop_count++;
        	vTaskDelay(2000 / portTICK_PERIOD_MS);

			if( loop_count > 10 )
			{
				ESP_LOGW("shcho", "CM1106 retry timeout : return -1");
				return -1;
			}

			goto PM2008_data_retry;
		}

//  			"Close: 1,
//  			Testing: 2,
//  			Alarm: 7,
//  			Data stable: 0x80
//  			Other data is invalid.
//  			(Check 3.3 detailed introduction for every kinds of sensor status)"

		switch(PM2008_data.status)
		{
			case 0x80: // Normal, Data is valid
				ESP_LOGW("pm2008", "status is Normal");
				send_PM2008_data( &PM2008_data );
				flag_skip = 1 ;
				break;
			case 1: //
				ESP_LOGW("pm2008", "status is Close");
				break;
			case 2: //
				ESP_LOGW("pm2008", "status is Testing");
				send_PM2008_data( &PM2008_data );
				break;
			default : //
				ESP_LOGW("pm2008", "status is invalid");
				break;

			loop_count++;
			if( flag_skip == 1 )
	        	vTaskDelay(2000 / portTICK_PERIOD_MS);
		}
		cks = calc_PM2008_cks((uint8_t *)&PM2008_data, sizeof(PM2008_data));
		if( (char)cks != (char)PM2008_data.cks )
		{
			ESP_LOGE("shcho", "PM2008_data cks differ(0x%02x vs. 0x%02x)", (char)cks, PM2008_data.cks);
		}
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
//      free(data);
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
        return 1;
    }
    return 0;
}



void i2c_sensor_task(void *arg)
{
	xSemaphoreTake(sema_i2c1, portMAX_DELAY);
	xSemaphoreTake(sema_i2c2, portMAX_DELAY);

	set_PM2008_mode(PM2008_CMD_CLOSE, 0x00);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
	set_PM2008_mode(PM2008_CMD_SETUP_CONTINUOUS, 0xffff);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
//  	set_PM2008_mode(PM2008_CMD_SETUP_TIMING_MEASURE, 180);
//      vTaskDelay(2000 / portTICK_PERIOD_MS);
	xSemaphoreGive(sema_i2c1);
	xSemaphoreGive(sema_i2c2);




	
	while(1)
	{
		xSemaphoreTake(sema_i2c1, portMAX_DELAY);
		do_get_CO2((int)NULL, (char**)NULL);
		xSemaphoreGive(sema_i2c1);

       	vTaskDelay(1000 / portTICK_PERIOD_MS);

		xSemaphoreTake(sema_i2c1, portMAX_DELAY);
		do_get_PM2008((int)NULL, (char**)NULL);
		xSemaphoreGive(sema_i2c1);

		xSemaphoreTake(sema_i2c1, portMAX_DELAY);
		do_fan_report(); // register 를 읽어서 보냄 mode는 "PWM duty"로 고정
		xSemaphoreGive(sema_i2c1);

       	vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}





void register_stella_cmd(void)
{
	register_view_tasks();
	register_restart_cmd();
	register_get_CO2();
	
	register_nvs_get_str();
	register_nvs_set_str();
	register_fan_ctrl();

}

int Uart_mux_setup(int direction)
{
    gpio_config_t io_conf;

    // detect Is it Wearable : Static은 Pull-up :10K GPIO_38(MIX_A0) / GPIO_39(MUX_A0)
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//                          0 으로만 읽힌다.
//      io_conf.mode = GPIO_MODE_INPUT; //
    io_conf.mode = direction; //
    //enable pull-up mode
    io_conf.pull_up_en = 0; // 1 --> 0
    io_conf.pull_down_en = 0; //NULL --> 0
    gpio_config(&io_conf);

    return 1;
}


void app_main(void)
{

	ESP_ERROR_CHECK(esp_read_mac(my_mac_factory, ESP_MAC_EFUSE_FACTORY ));
	hexdump3("esp_read_mac(ESP_MAC_EFUSE_FACTORY)", my_mac_factory, sizeof(my_mac_factory));

	memset(my_mac_str, 0, sizeof(my_mac_str));
	snprintf(my_mac_str, sizeof(my_mac_str),"%02X%02X%02X_%02X%02X%02X", MAC2STR(my_mac_factory));
	ESP_LOGW("system", "my mac(factory) : %s\n", my_mac_str);


	sema_i2c1 = xSemaphoreCreateBinary();
	sema_i2c2 = xSemaphoreCreateBinary();
	sema_uart1 = xSemaphoreCreateBinary();
	sema_uart2 = xSemaphoreCreateBinary();
	sema_tcp = xSemaphoreCreateBinary();

	xSemaphoreGive(sema_i2c1);
	xSemaphoreGive(sema_i2c2);
	xSemaphoreGive(sema_uart1);
	xSemaphoreGive(sema_uart2);
	xSemaphoreGive(sema_tcp);

	// 0. ---- LED ctrl
    xTaskCreate(app_main_led_strip_ctrl, "led_strip_ctrl", 4 * 1024, NULL, 5, NULL);

	//shcho
	test_json();

	//1. ------------ detect Static / Wearable ---------------------
	Uart_mux_setup(GPIO_MODE_INPUT);

    int val_mux_A1 = gpio_get_level(38);
    int val_mux_A0 = gpio_get_level(39);

    ESP_LOGW("shcho", "val_mux_A0=%d / val_mux_A1=%d", val_mux_A1, val_mux_A0);

	ESP_ERROR_CHECK(nvs_flash_init());

    if( val_mux_A0 == 0 && val_mux_A1 == 0 )
    {
        flag_IS_WEARABLE = 1 ;
        ESP_LOGW("shcho", "This Board is Wearable(%d): No UART_MUX(UART1) / No CM4 Communication(UART2)", flag_IS_WEARABLE);


		if( flag_USE_W5500_Ethernet == 1 ) 
		{
		    ESP_ERROR_CHECK(esp_netif_init());
		    ESP_ERROR_CHECK(esp_event_loop_create_default());
		
		    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
		     * Read "Establishing Wi-Fi or Ethernet Connection" section in
		     * examples/protocols/README.md for more information about this function.
		     */
		    ESP_ERROR_CHECK(example_connect());
	//      	tcp_client(); // org : OK shcho
	//      	xTaskCreate(tcp_client_task, "tcp_client", 4 * 1024, NULL, 5, NULL); // OK shcho // It is Just Test
		}

    }
    else
    {
        flag_IS_WEARABLE = 0 ;
        ESP_LOGW("shcho", "This Board is Static(%d): : UART_MUX(UART1) / CM4 Communication(UART2)", flag_IS_WEARABLE);
        Uart_mux_setup(GPIO_MODE_OUTPUT);
    }
	//--------------------------------------------------------------
	
    if ( flag_IS_WEARABLE == 1 )
	{
//  	    xTaskCreate(app_main_task_oled, "oled", 4 * 1024, NULL, 5, NULL);
		// I2C를 사용하고 완전히 삭제한다.
		app_main_task_oled(NULL);
       	vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

    i2c_master_bus_config_t i2c_bus_config_i2c1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port_i2c1,
        .scl_io_num = 6 , //i2c_gpio_scl,
        .sda_io_num = 7,  //i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_config_t i2c_bus_config_i2c2 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port_i2c2,
        .scl_io_num = 15 , //i2c_gpio_scl,
        .sda_io_num = 16,  //i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

//  	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c1, &tool_bus_handle));
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c1, &tool_bus_handle_i2c1));
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c2, &tool_bus_handle_i2c2));

//      i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = i2c_frequency,
//          .device_address = CM1106_CO2_I2C_DEV_ADDR, //chip_addr,
//      };
//  	ESP_ERROR_CHECK(i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1));
//  
//  	i2c_dev_conf.scl_speed_hz   = i2c_frequency ; 
//  	i2c_dev_conf.device_address = PM2008_I2C_DEV_ADDR ; 
//  	//맨 마직막 device_address로만 설정된다. // 그래서 그때그때 다시 설정해야 한다.
//  	ESP_ERROR_CHECK(i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1));

//  for UART2 Debugging : CM4와 연결된 ttyAMA3이 Enable되면 ESP32 Program을 할 수 없음 / monitoring은 됨
//  	I2C thread
//  	do_get_CO2((int)NULL, (char**)NULL);
    xTaskCreate(i2c_sensor_task, "i2c_sensor", 4 * 1024, NULL, 5, NULL);



//  	if( flag_USE_W5500_Ethernet == 1 ) 
	{
		app_main_stella_uart1(); // get sensor data // using mux_ctrl // thread for RS9A / and ZE08
	}

	if( flag_IS_WEARABLE == 0 ) //Static Main
	{
		app_main_stella_uart2(); // send to CM4
	}

	// Below is Console
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

#if CONFIG_EXAMPLE_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
#endif

//      repl_config.prompt = "i2c-tools>";
    repl_config.prompt = "stella-tools>";

    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif

    register_i2ctools();
	register_stella_cmd();

    printf("\n ==============================================================\n");
    printf(" |             Steps to Use i2c-tools                         |\n");
    printf(" |                                                            |\n");
    printf(" |  1. Try 'help', check all supported commands               |\n");
    printf(" |  2. Try 'i2cconfig' to configure your I2C bus              |\n");
    printf(" |  3. Try 'i2cdetect' to scan devices on the bus             |\n");
    printf(" |  4. Try 'i2cget' to get the content of specific register   |\n");
    printf(" |  5. Try 'i2cset' to set the value of specific register     |\n");
    printf(" |  6. Try 'i2cdump' to dump all the register (Experiment)    |\n");
    printf(" |                                                            |\n");
    printf(" ==============================================================\n\n");

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));


}
