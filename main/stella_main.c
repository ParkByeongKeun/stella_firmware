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

#include <sht4x.h>
#include <sgp40.h>
#include <ets_sys.h>

#include "stella_global.h"
#include "i2s_pdm_example.h"


#define STORAGE_NAMESPACE "storage"
static int count_CO2_ppm_valid = 0;

char DeviceID[20];

// During Cert(PM2008, CM1106, RS9A) : Sensor Connection Used for W5500

    
SemaphoreHandle_t sema_i2c1 = NULL;
SemaphoreHandle_t sema_i2c2 = NULL;
SemaphoreHandle_t sema_uart1 = NULL;
SemaphoreHandle_t sema_uart2 = NULL;
SemaphoreHandle_t sema_tcp = NULL;
SemaphoreHandle_t sema_spi_ads114s = NULL;
SemaphoreHandle_t sema_ble_send_noti = NULL;

static const char *TAG = "i2c-tools";
static uint32_t i2c_frequency = 100 * 1000;
int flag_CO2_autozero_close_run = 1;
//  static uint32_t i2c_frequency = 100 * 1000;
#define I2C_TOOL_TIMEOUT_VALUE_MS (50)

#define GPIO_INPUT_IO_0     38
#define GPIO_INPUT_IO_1     39
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

#define GPIO_ZMOD_RESET    45

#define I2C2__USING_GPIO	(1)
//  #define I2C2__USING_GPIO	(0)

//shcho add
#if I2C2__USING_GPIO
//  sdfsadfasdfasf
#include "soft_i2c_master.h"
soft_i2c_master_bus_t bus_i2c2_gpio = NULL;
#endif

#define GPIO_I2C2_SCL    15
#define GPIO_I2C2_SDA    16


#define SHT4X_CMD_RESET             0x94
#define SHT4X_CMD_SERIAL            0x89
#define SHT4X_CMD_MEAS_HIGH         0xfd
#define SHT4X_CMD_MEAS_MED          0xf6
#define SHT4X_CMD_MEAS_LOW          0xe0
#define SHT4X_CMD_MEAS_H_HIGH_LONG  0x39
#define SHT4X_CMD_MEAS_H_HIGH_SHORT 0x32
#define SHT4X_CMD_MEAS_H_MED_LONG   0x2f
#define SHT4X_CMD_MEAS_H_MED_SHORT  0x24
#define SHT4X_CMD_MEAS_H_LOW_LONG   0x1e
#define SHT4X_CMD_MEAS_H_LOW_SHORT  0x15


#define SGP40_CMD_SOFT_RESET  0x0006
#define SGP40_CMD_FEATURESET  0x202f
#define SGP40_CMD_MEASURE_RAW 0x260f
#define SGP40_CMD_SELF_TEST   0x280e
#define SGP40_CMD_SERIAL      0x3682
#define SGP40_CMD_HEATER_OFF  0x3615

#define SGP40_TIME_SOFT_RESET  (10)
#define SGP40_TIME_FEATURESET  (10)
#define SGP40_TIME_MEASURE_RAW (30)
#define SGP40_TIME_SELF_TEST   (250)
#define SGP40_TIME_HEATER_OFF  (10)
#define SGP40_TIME_SERIAL      (10)

#define SELF_TEST_OK 0xd400



int flag_CO2_sensor_OK = 0 ;
int flag_IS_WEARABLE = 0 ;

//  int flag_USE_W5500_Ethernet = 1 ; // 환경부 인증용, Sensor Board SPI를 W5500으로 사용할때
int flag_USE_W5500_Ethernet = 0 ;

int ZMOD_Reset_GPIO(int val);



//  i2c_master_dev_handle_t dev_handle_i2c1; // device_address를 그때그때 바꾸려고 했는데
//  											Error  ...add_device() --> ...rm_device()를 해야 한다.

extern int fd_uart2 ;

extern void hexdump3(char *title, void *pack, size_t size) ;
extern void app_main_led_strip_ctrl(void *arg) ;//나중에 R/G/B/W로 변경하자
extern void tcp_client_task(void* arg);
extern int send_to_server(char *payload, int len);
extern void app_main_task_oled(void *arg);
extern void app_main_tcp_server(int port);
extern void app_main_nimble_sec(void) ;

extern void spi2_adc_task(void *arg);
extern void app_main_stella_uart2_GPS(void);

static int do_esp32_fan_ctrl(int argc, char **argv) ;
int send_date_json( void );
extern int ble_send_noti_int(char *id, int value);
extern int ble_send_noti_str(char *id, char* value);



//  //  static gpio_num_t i2c_gpio_sda = CONFIG_EXAMPLE_I2C_MASTER_SDA;
//  //  static gpio_num_t i2c_gpio_scl = CONFIG_EXAMPLE_I2C_MASTER_SCL;
//  static gpio_num_t i2c_gpio_sda = 7; // i2c2 : SDA 16
//  static gpio_num_t i2c_gpio_scl = 6; // i2c2 : SCL 15


// i2c2를 Eanble하면 I2S(x), UART2_Tx(x) : PORT를 변경해 봄
//  static i2c_port_t i2c_port_i2c1 = I2C_NUM_0;
static i2c_port_t i2c_port_i2c1 = I2C_NUM_1;
#if I2C2__USING_GPIO
#else
static i2c_port_t i2c_port_i2c2 = I2C_NUM_1;
//  static i2c_port_t i2c_port_i2c2 = LP_I2C_NUM_0; // only for C6 or P4
#endif

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
#define BQ40Z50_I2C_DEV_ADDR	0x0b // I2C2

#define STR_MATCH	(0)

static sht4x_t dev_sht4x; //shcho add
static sgp40_t dev_sgp40;
#define I2C2_FREQ_HZ 400000
uint8_t i2c2_ack = 0;
//  uint8_t i2c2_data[100];

#define G_POLYNOM_SHT4x 0x31

MessageBufferHandle_t passkey_msg_handle;
const size_t passkey_msg_bytes  = sizeof(size_t) + sizeof(uint32_t); // one size_t for buffer index, another size_t for MessageBuffer overhead


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

struct _CO2_cali_resp 
{
	char cmd;
	uint16_t digit_1;
	char cks;
}__attribute__((packed));

struct _CO2_autozero_resp 
{
	char cmd;
	uint8_t wrong_code;
	uint8_t zero_set;
	uint8_t cali_days;
	uint16_t cali_ppm;
	uint8_t resv;
	char cks;
}__attribute__((packed));

uint8_t my_mac_factory[20];
char my_mac_str[32];

void app_main_stella_uart1(void);
void app_main_stella_uart2(void);
int send_CM1106_data( struct _CO2_ppm_packet *data );
esp_err_t ijoon_get_nvs_str(uint8_t *key, uint8_t *value);
esp_err_t ijoon_set_nvs_str(uint8_t *key, uint8_t *value);

esp_err_t get_DeviceName_from_NVS(char *str)
{
	char *DeviceID_Test="3W12345";
    ijoon_get_nvs_str((uint8_t*)"ID", (uint8_t *)str);
	if( str[0] == 0 )
	{
		strcpy(str, DeviceID_Test);	
	}
	return 0;
}




#define BQ40Z50_PEC_POLYNOMIAL	0x07	///< Polynomial for calculating PEC
uint8_t BQ40Z50_get_pec(uint8_t *buff, const uint8_t len);





static const char *JSON_TAG = "JSON";

extern void task_sgp40(void *pvParamters);

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


//  //  int get_CO2_ppm( int *ppm)
//  int get_CO2_ppm( struct _CO2_ppm_packet *CO2_ppm_packet)
int get_CO2_ppm( struct _CO2_ppm_packet *CO2_ppm_packet, i2c_master_dev_handle_t dev_handle_i2c1)
{
	int ret_val = 0 ;
//  	static int Is_1st = 1 ; 

	int len = sizeof(struct _CO2_ppm_packet);

	int data_addr = 0x01; //cmd
	int8_t cks = 0;

//  	caller func에서 한 번만 한다.
//  	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;
//      i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = i2c_frequency,
//          .device_address = chip_addr,
//      };
//  
//      i2c_master_dev_handle_t dev_handle_i2c1;
//      if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
//          return 1;
//      }
//  
//  //  	dev_handle_i2c1->device_address = CM1106_CO2_I2C_DEV_ADDR; // Error

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
//  				return -1;
				ret_val = -1;
				goto error_get_CO2_ppm;
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
//  				return -10;
				ret_val = -10;
				goto error_get_CO2_ppm;
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
//  				return CO2_ppm_packet->status;
				ret_val = CO2_ppm_packet->status;
				goto error_get_CO2_ppm;
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

error_get_CO2_ppm:
//      free(data);

//  //  	caller func에서 한 번만 한다.
//      if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
//  		ret_val = 1 ;
//  //          return 1;
//      }

    return ret_val;
}

//  int get_CO2_SW_ver(char *sw_ver)
int get_CO2_SW_ver(char *sw_ver, i2c_master_dev_handle_t dev_handle_i2c1)
{
	int ret_val = 0 ;
//  	static int Is_1st = 1 ; 
//  	static i2c_master_dev_handle_t dev_handle_i2c1;
	char tmp_str[100];
	int len = 13 ;// 고정

	int data_addr = 0x1E; //cmd
	int8_t cks = 0;

//  	caller func에서 한 번만 한다.
//  	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;
//      i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = i2c_frequency,
//          .device_address = chip_addr,
//      };
//  
//      i2c_master_dev_handle_t dev_handle_i2c1;
//      if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
//          return 1;
//      }

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
        	vTaskDelay(1000 / portTICK_PERIOD_MS); // shcho : 2025.03.26 : 2000->1000

			if( loop_count > 10 )
			{
				ESP_LOGE("shcho", "CM1106 retry timeout : return -1");
//  				return -1;
				ret_val = -1;
				goto error_get_CO2_SW_ver;
			}

			goto CO2_get_SW_ver_retry;
		}

		cks = calc_CO2_cks((uint8_t *)tmp_str, len);
		if( (char)cks != (char)tmp_str[len-1] )
		{
			ESP_LOGE("shcho", "get_CO2_ppm cks differ(0x%02x vs. 0x%02x)", (char)cks, tmp_str[len-1]);
//  			return -20;
			ret_val = -20;
			goto error_get_CO2_SW_ver;
		}
		else
		{
			memcpy(sw_ver, &tmp_str[1], len -2); // exclude cmd + cks
//  			return 0;
			ret_val = 0;
			goto error_get_CO2_SW_ver;
		}

    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
//      free(data);
//
error_get_CO2_SW_ver:

//  	caller func에서 한 번만 한다.
//      if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
//  //          return -20;
//  			ret_val = -20;
//      }
	return ret_val;
}

int do_CO2_autozero(int argc, char **argv)
{
	int ret_val = 0 ;
	struct _CO2_autozero_resp CO2_autozero_resp;
	int len = sizeof(CO2_autozero_resp) ;// 고정:12
	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;

//  	req  : 0x03 [DF1] [DF0]
//  	resp : 0x03 [DF1] [DF0] [CS]
//
	#define CO2_AUTOZERO_COMMAND	(0x10)
	uint16_t tmp_u16 = 0;
	int8_t cks = 0;
	uint8_t tmp_req[7];
	tmp_req[0] = CO2_AUTOZERO_COMMAND;
	tmp_req[1] = 100; //wrong code
	if( strncmp(argv[1], "open", strlen("open")) ==  STR_MATCH )
	{
		tmp_req[2] = 0; 
	}
	else
	{
		tmp_req[2] = 2; 
	}
	tmp_req[3] = atoi(argv[2]); //wrong code
	tmp_u16 = atoi(argv[3]);    
	tmp_req[4] = (tmp_u16 & 0xff00) >> 8; // ppm
	tmp_req[5] = (tmp_u16 & 0x00ff) >> 0; // ppm
	tmp_req[6] = 100; 

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

	ESP_LOGW("do_CO2_autozero", "			wait for sema_i2c until taken");
	ESP_LOGW("do_CO2_autozero", "			wait for sema_i2c until taken");
	ESP_LOGW("do_CO2_autozero", "			wait for sema_i2c until taken");
	ESP_LOGW("do_CO2_autozero", "			wait for sema_i2c until taken");

//  xSemaphoreTake(sema_i2c1, portMAX_DELAY); -->삭제

	ESP_LOGW("do_CO2_autozero", "take sema_i2c during CO2 autozero");
	ESP_LOGW("do_CO2_autozero", "take sema_i2c during CO2 autozero");
	ESP_LOGW("do_CO2_autozero", "take sema_i2c during CO2 autozero");
	ESP_LOGW("do_CO2_autozero", "take sema_i2c during CO2 autozero");

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) 
	{
//  		xSemaphoreGive(sema_i2c1); -->삭제
        return 1;
    }

	int loop_count = 0 ;
CO2_autozero_retry :
	memset((char *)&CO2_autozero_resp, 0, sizeof(CO2_autozero_resp));
//  	dev_handle_i2c1->device_address = CM1106_CO2_I2C_DEV_ADDR; // Error
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&tmp_req, 7, 
	                                 (uint8_t *)&CO2_autozero_resp, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) 
	{
		hexdump3("CO2_autozero_resp:", (char *)&CO2_autozero_resp, len);
		if ( CO2_autozero_resp.cmd != CO2_AUTOZERO_COMMAND  )
		{
			ESP_LOGW("shcho", "CM1106 reply old cmd: retry again( sleep 2) cmd 0x03(auto cali)");
			loop_count++;

			if( loop_count > 20 )
			{
				ESP_LOGW("shcho", "CM1106 retry timeout : return -1");
//  				xSemaphoreGive(sema_i2c1);
//  				return -1;
				ret_val = -1;
				goto error_CO2_autozero;
			}

        	vTaskDelay(2000 / portTICK_PERIOD_MS);
			goto CO2_autozero_retry;
		}

		cks = calc_CO2_cks((uint8_t *)&CO2_autozero_resp, len);
		if( (char)cks != (char)CO2_autozero_resp.cks )
		{
			ESP_LOGE("shcho", "get_CO2_autozero cks differ(0x%02x vs. 0x%02x)", (char)cks, CO2_autozero_resp.cks);
		}
		else
		{
			ESP_LOGW("CO2_autozero", "auto_autozero_success :%d : wait 5 secs", htons(CO2_autozero_resp.cali_ppm));
        	vTaskDelay(5000 / portTICK_PERIOD_MS);
//  			xSemaphoreGive(sema_i2c1); -->삭제
			ESP_LOGW("CO2_autozero", "               other i2c1 task will be run");
			ESP_LOGW("CO2_autozero", "               other i2c1 task will be run");
			ESP_LOGW("CO2_autozero", "               other i2c1 task will be run");
			ESP_LOGW("CO2_autozero", "               other i2c1 task will be run");
			ESP_LOGW("CO2_autozero", "               other i2c1 task will be run\n\n\n");
//  			return 0;
			ret_val = -1;
			goto error_CO2_autozero;
		}

    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
//      free(data);

error_CO2_autozero:
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
//  		xSemaphoreGive(sema_i2c1);
//          return -20;
		ret_val = -20;
		goto error_CO2_autozero;
    }

//  xSemaphoreGive(sema_i2c1); -->삭제
	return ret_val;
}

int do_CO2_autozero_wrap(int argc, char **argv)
{
	xSemaphoreTake(sema_i2c1, portMAX_DELAY);
	do_CO2_autozero(argc, argv);
	xSemaphoreGive(sema_i2c1);
	return 0;
}

static int  register_CO2_autozero()
{
    const esp_console_cmd_t cmd = {
        .command = "co2-autozero",
        .help = "co2-autozero [open|close] [ 1 ~ 15 (days)] [400 ~ 1500(ppm)]",
        .hint = NULL,
        .func = do_CO2_autozero_wrap,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}

int do_CO2_calibration(int argc, char **argv)
{
	struct _CO2_cali_resp CO2_cali_resp;
	int len = sizeof(CO2_cali_resp) ;// 고정:12
	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;

//  	req  : 0x03 [DF1] [DF0]
//  	resp :0x03 [DF1] [DF0] [CS]
//
	uint16_t tmp_u16 = atoi(argv[1]);
	int8_t cks = 0;
	uint8_t tmp_req[3];
	tmp_req[0] = 0x03;
	tmp_req[1] = (tmp_u16 & 0xff00) >> 8;
	tmp_req[2] = (tmp_u16 & 0x00ff) >> 0;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

	ESP_LOGW("do_CO2_calibration", "			wait for sema_i2c until taken");
	ESP_LOGW("do_CO2_calibration", "			wait for sema_i2c until taken");
	ESP_LOGW("do_CO2_calibration", "			wait for sema_i2c until taken");
	ESP_LOGW("do_CO2_calibration", "			wait for sema_i2c until taken");
	xSemaphoreTake(sema_i2c1, portMAX_DELAY);
	ESP_LOGW("do_CO2_calibration", "take sema_i2c during CO2 Calibration");
	ESP_LOGW("do_CO2_calibration", "take sema_i2c during CO2 Calibration");
	ESP_LOGW("do_CO2_calibration", "take sema_i2c during CO2 Calibration");
	ESP_LOGW("do_CO2_calibration", "take sema_i2c during CO2 Calibration");

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) 
	{
		xSemaphoreGive(sema_i2c1);
        return 1;
    }

	int loop_count = 0 ;
CO2_cali_retry :
	memset((char *)&CO2_cali_resp, 0, sizeof(CO2_cali_resp));
//  	dev_handle_i2c1->device_address = CM1106_CO2_I2C_DEV_ADDR; // Error
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&tmp_req, 3, 
	                                 (uint8_t *)&CO2_cali_resp, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) 
	{
		hexdump3("CO2_cali_resp:", (char *)&CO2_cali_resp, len);
		if ( CO2_cali_resp.cmd != 0x03  )
		{
			ESP_LOGW("shcho", "CM1106 reply old cmd: retry again( sleep 2) cmd 0x03(auto cali)");
			loop_count++;

			if( loop_count > 20 )
			{
				ESP_LOGW("shcho", "CM1106 retry timeout : return -1");
				xSemaphoreGive(sema_i2c1);
				return -1;
			}

        	vTaskDelay(2000 / portTICK_PERIOD_MS);
			goto CO2_cali_retry;
		}

		cks = calc_CO2_cks((uint8_t *)&CO2_cali_resp, len);
		if( (char)cks != (char)CO2_cali_resp.cks )
		{
			ESP_LOGE("shcho", "set CO2_cali cks differ(0x%02x vs. 0x%02x)", (char)cks, CO2_cali_resp.cks);
		}
		else
		{
			ESP_LOGW("CO2_cali", "cali_success :%d : wait 5 secs", htons(CO2_cali_resp.digit_1));
        	vTaskDelay(5000 / portTICK_PERIOD_MS);
			xSemaphoreGive(sema_i2c1);
			ESP_LOGW("CO2_cali", "               other i2c1 task will be run");
			ESP_LOGW("CO2_cali", "               other i2c1 task will be run");
			ESP_LOGW("CO2_cali", "               other i2c1 task will be run");
			ESP_LOGW("CO2_cali", "               other i2c1 task will be run");
			ESP_LOGW("CO2_cali", "               other i2c1 task will be run\n\n\n");
			return 0;
		}

    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
//      free(data);
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
		xSemaphoreGive(sema_i2c1);
        return -20;
    }
	xSemaphoreGive(sema_i2c1);
	return 0;
}

static int  register_CO2_cali()
{
    const esp_console_cmd_t cmd = {
        .command = "co2-cali",
        .help = "co2 cali  ( 400 ~ 1500)ppm",
        .hint = NULL,
        .func = do_CO2_calibration,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}


int get_CO2_Serial_num(char *sn, i2c_master_dev_handle_t dev_handle_i2c1)
{
//  	static i2c_master_dev_handle_t dev_handle_i2c1;
//  	char tmp_str[100];
	struct _CO2_sn_packet CO2_sn_packet;
	int len = sizeof(CO2_sn_packet) ;// 고정:12

	int data_addr = 0x1F; //cmd
	int8_t cks = 0;

//  	caller func에서 한 번만 한다.
//  	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;
//      i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = i2c_frequency,
//          .device_address = chip_addr,
//      };
//  
//      i2c_master_dev_handle_t dev_handle_i2c1;
//      if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
//          return 1;
//      }

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

//
//  	caller func에서 한 번만 한다.
//      if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
//          return -20;
//      }

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

static int do_esp32_setid(int argc, char **argv) {

	char id[20];

	memset( id, 0, sizeof(id));

    strncpy(id, argv[1], MIN( 19,strlen(argv[1]) ) );
	ijoon_set_nvs_str((uint8_t*)"ID", (uint8_t*)id);
    ESP_LOGW("cli", "Deivce ID set to :%s", id);
    return 0;
}

static int  register_setid_cmd()
{
    const esp_console_cmd_t cmd = {
        .command = "setid",
        .help = "setid 3W00010",
        .hint = NULL,
        .func = do_esp32_setid,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}


static int do_spec_sensor_sensitivity(int argc, char **argv) {

	char id[20];

	//shcho순서에 주의
	ijoon_set_nvs_str((uint8_t*)"S_NO2", (uint8_t*)argv[1]);
	ijoon_set_nvs_str((uint8_t*)"S_CO",  (uint8_t*)argv[2]);
	ijoon_set_nvs_str((uint8_t*)"S_O3",  (uint8_t*)argv[3]);
	ijoon_set_nvs_str((uint8_t*)"S_H2S", (uint8_t*)argv[4]);
	return 0;
}

static int  register_spec_sensor_sensitivity()
{
    const esp_console_cmd_t cmd = {
        .command = "sens",
        .help = "sens 22.48 4.42 60.66 214.13",
        .hint = NULL,
        .func = do_spec_sensor_sensitivity,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}

static int do_spec_sensor_vgas0(int argc, char **argv) {

	char id[20];
	//shcho순서에 주의
	ijoon_set_nvs_str((uint8_t*)"VGAS0_H2S", (uint8_t*)argv[1]);
	ijoon_set_nvs_str((uint8_t*)"VGAS0_O3" , (uint8_t*)argv[2]);
	ijoon_set_nvs_str((uint8_t*)"VGAS0_CO" , (uint8_t*)argv[3]);
	ijoon_set_nvs_str((uint8_t*)"VGAS0_NO2", (uint8_t*)argv[4]);
	return 0;
}

static int  register_spec_sensor_vgas0()
{
    const esp_console_cmd_t cmd = {
        .command = "vgas0",
        .help = "vgas0 (adcval 4ea)",
        .hint = NULL,
        .func = do_spec_sensor_vgas0,
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
    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
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

int do_get_als(void)
{
//  	// FAN Controller Resigter Read
//  	int chip_addr = FAN_CTRL_I2C_DEV_ADDR;

//  	int len = 1 ;
	//  i2cget -c 0x29 -r 0x04 -l 2
	uint8_t command = 0x04; 

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = LIGHT_SENSOR_I2C_DEV_ADDR,
    };

	i2c_master_dev_handle_t dev_handle_i2c1;
	if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
		return 1;
	}

	uint16_t val;
	float lux_f;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle_i2c1, (uint8_t*)&command, 1, 
	                                 (uint8_t *)&val, 2, I2C_TOOL_TIMEOUT_VALUE_MS);
	ESP_LOGI("ALS", " val=%04x(%d)\n", val, val);
//  	lux_f = 0.2048*val; // 50mse integration time
	lux_f = 0.0256*val; // 50mse integration time

    if (ret == ESP_OK) 
	{
	    ESP_LOGI(JSON_TAG, "Serialize.....Fan");
	    cJSON *root;
	   	root = cJSON_CreateObject();
    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);

		char tmp_str[10];
    	sprintf(tmp_str, "%.2f", (float)(lux_f));
	   	cJSON_AddNumberToObject(root, "ALS_lux",   atof(tmp_str));
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

static int set_fan_pwm(void)
{
	char fan_pwm_str[10];
	memset( fan_pwm_str, 0, sizeof(fan_pwm_str) );
	ijoon_get_nvs_str((uint8_t *)"fan_pwm", (uint8_t*)fan_pwm_str);

	int argc = 2 ;
	char *argv[2];

	if( fan_pwm_str[0] == 0 )
	{
		ESP_LOGW("fan_pwn", "fan_pwm duty is not set --> set to 100(percent)");
		argv[1] = (char *)"100";
	}
	else
	{
		ESP_LOGW("fan_pwn", "fan_pwm is %s %% (pwm_duty percent)", fan_pwm_str);
		argv[1] = (char *)fan_pwm_str;
	}
	do_esp32_fan_ctrl(argc,argv);

	return 0;
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
		ijoon_set_nvs_str((uint8_t*)"fan_pwm", (uint8_t*)argv[1]);
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
static int do_esp32_fan_ctrl_wrap(int argc, char **argv) 
{
	
	xSemaphoreTake(sema_i2c1, portMAX_DELAY);
	do_esp32_fan_ctrl(argc, argv); 
	xSemaphoreGive(sema_i2c1);
	return 0;
}

static int  register_fan_ctrl()
{
    const esp_console_cmd_t cmd = {
        .command = "fan",
        .help = "fan control ( 0 ~ 100 %)",
        .hint = NULL,
        .func = do_esp32_fan_ctrl_wrap,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}

uint8_t BQ40Z50_get_pec(uint8_t *buff, const uint8_t len)
{
	// TODO: use "return crc8ccitt(buff, len);"

	// Initialise CRC to zero.
	uint8_t crc = 0;
	uint8_t shift_register = 0;
	bool invert_crc;

	// Calculate crc for each byte in the stream.
	for (uint8_t i = 0; i < len; i++) {
		// Load next data byte into the shift register
		shift_register = buff[i];

		// Calculate crc for each bit in the current byte.
		for (uint8_t j = 0; j < 8; j++) {
			invert_crc = (crc ^ shift_register) & 0x80;
			crc <<= 1;
			shift_register <<= 1;

			if (invert_crc) {
				crc ^= BQ40Z50_PEC_POLYNOMIAL;
			}
		}
	}

	return crc;
}

static int do_bq40z50_FET_En(int argc, char **argv) 
{
	uint8_t buf_tx[5];
    esp_err_t ret ;
	// 0th : test
//  	i2cset -c 0x0b -r 0x44 0x02 0x2B 0x00 0x55
//  	buf_tx[0] = 0x44;
//  	buf_tx[1] = 0x02;
//  	buf_tx[2] = 0x2B;
//  	buf_tx[3] = 0x00;
//  	buf_tx[4] = BQ40Z50_get_pec(buf_tx,4);
//  
//  	ESP_LOGW("do_bq40z50_FET_En", "for test :LED_Toggle");
//  	hexdump3("buf_tx", buf_tx, sizeof(buf_tx));
	buf_tx[0] = (0x0b<<1) | 0x10;
	buf_tx[1] = 0x00; // 0x00(backward compatibility : 0x44 0x02(len)
	buf_tx[2] = 0x2B;
	buf_tx[3] = 0x00;
	buf_tx[4] = BQ40Z50_get_pec(buf_tx,4);

	ESP_LOGW("do_bq40z50_FET_En", "for test :LED_Toggle");
	hexdump3("buf_tx", &buf_tx[1], sizeof(buf_tx));

	buf_tx[0] = (0x0b<<1) | 0x10;
	buf_tx[1] = 0x44; // 0x00(backward compatibility : 0x44 0x02(len)
	buf_tx[2] = 0x02; // 0x00(backward compatibility : 0x44 0x02(len)
	buf_tx[3] = 0x2B;
	buf_tx[4] = 0x00;
	buf_tx[5] = BQ40Z50_get_pec(buf_tx,5);

	ESP_LOGW("do_bq40z50_FET_En", "for test :LED_Toggle");
	hexdump3("buf_tx", &buf_tx[1], sizeof(buf_tx));


	// 1st : DeviceReset
//  	i2cset -c 0x0b -r 0x44 0x02 0x22 0x00
	buf_tx[0] = 0x44;
	buf_tx[1] = 0x02;
	buf_tx[2] = 0x41;
	buf_tx[3] = 0x00;
	buf_tx[4] = BQ40Z50_get_pec(buf_tx,4);

	xSemaphoreTake(sema_i2c2, portMAX_DELAY);
	ESP_LOGW("ddd", "-----------------------------");
    ret = soft_i2c_master_write(bus_i2c2_gpio, BQ40Z50_I2C_DEV_ADDR, (uint8_t*)buf_tx, 5);
	ESP_LOGW("ddd", "-----------------------------");
	xSemaphoreGive(sema_i2c2);
	

	ESP_LOGW("do_bq40z50_FET_En", "Device Reset ret=%d", ret);
	hexdump3("buf_tx", buf_tx, sizeof(buf_tx));
	// 2nd : FET_En
//  	i2cset -c 0x0b -r 0x44 0x02 0x22 0x00
	buf_tx[0] = 0x44;
	buf_tx[1] = 0x02;
	buf_tx[2] = 0x22;
	buf_tx[3] = 0x00;
	buf_tx[4] = BQ40Z50_get_pec(buf_tx,4);

	xSemaphoreTake(sema_i2c2, portMAX_DELAY);
	ESP_LOGW("ddd", "-----------------------------");
    ret = soft_i2c_master_write(bus_i2c2_gpio, BQ40Z50_I2C_DEV_ADDR, (uint8_t*)buf_tx, 5);
	ESP_LOGW("ddd", "-----------------------------");
	xSemaphoreGive(sema_i2c2);

	ESP_LOGW("do_bq40z50_FET_En", "FET_En ret=%d", ret);
	hexdump3("buf_tx", buf_tx, sizeof(buf_tx));

	return 0;
}

static int  register_charge_en()
{
    const esp_console_cmd_t cmd = {
        .command = "charge_en",
        .help = "FET Enable(All) : Toggle",
        .hint = NULL,
        .func = do_bq40z50_FET_En,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    return 0;
}

static int do_get_CO2(int argc, char **argv) 
{
	int ret = 0;
	int ret_val = 0 ;

	send_date_json();


/////// ---------------------------------------------------------------------------------------------
	// 2025.03.26 : 각 함수에서 하던 것은 --> 여기서 한번만 add_device 하고 마지막에 rm_device
	int chip_addr = CM1106_CO2_I2C_DEV_ADDR;
//  	int data_addr = 0x1E; //cmd
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };

    static i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device(tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) {
        return 1;
    }
/////// ---------------------------------------------------------------------------------------------

	// 1. CO2_SW_ver ==========================================
	ESP_LOGW("shcho", " get_CO2_SW_ver");
	memset(CO2_SW_ver_str, 0, sizeof(CO2_SW_ver_str));
//  	ret = get_CO2_SW_ver( CO2_SW_ver_str );
	ret = get_CO2_SW_ver( CO2_SW_ver_str, dev_handle_i2c1 );
	if( ret == 0 )  // OK
	{
		ESP_LOGI("shcho", "CO2 Sensor SW_Ver=%s", CO2_SW_ver_str);
	}

	// 2. CO2_Serial_num  ==========================================
	ESP_LOGW("shcho", " get_CO2_Serial_num");
	memset(CO2_Serial_num_str, 0, sizeof(CO2_Serial_num_str));
//  	ret = get_CO2_Serial_num( CO2_Serial_num_str );
	ret = get_CO2_Serial_num( CO2_Serial_num_str, dev_handle_i2c1 );
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
//  	ret = get_CO2_ppm( &CO2_ppm_packet ) ;
	ret = get_CO2_ppm( &CO2_ppm_packet, dev_handle_i2c1 ) ;
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

/////// ---------------------------------------------------------------------------------------------
	// 2025.03.26 : 각 함수에서 하던 것은 --> 여기서 한번만 add_device 하고 마지막에 rm_device
    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) {
			ret_val = -20;
    }
    return 0;
/////// ---------------------------------------------------------------------------------------------
}

static int do_get_CO2_wrap(int argc, char **argv) 
{
	xSemaphoreTake(sema_i2c1, portMAX_DELAY);
	do_get_CO2(argc, argv);
	xSemaphoreGive(sema_i2c1);

	return 0;
}



static int  register_get_CO2()
{
    const esp_console_cmd_t cmd = {
        .command = "get_CO2",
        .help = "get_CO2 sensor : val , sw_ver, sn",
        .hint = NULL,
        .func = do_get_CO2_wrap,
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

int send_date_json( void )
{
	// Set timezone to Seoul Standard Time
	time_t now;
	struct tm timeinfo;
	char strftime_buf[64];

	time(&now);
	setenv("TZ", "KST-9", 1);
	tzset();
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

	struct timeval mytime;
	
	// 현재 시간을 얻어온다.
	gettimeofday(&mytime, NULL);


	ESP_LOGI(TAG, "The current date/time in  Seoul   is: %s", strftime_buf);
//  	if( data->status == 0 )
	{
	    ESP_LOGI(JSON_TAG, "Serialize.....Date");
	    cJSON *root;
    	root = cJSON_CreateObject();
    	cJSON_AddStringToObject(root, "date(str)", strftime_buf);
    	cJSON_AddNumberToObject(root, "date(sec)", mytime.tv_sec);

	    char *my_json_string = cJSON_Print(root);

    	ESP_LOGI("Date", "my_json_string\n%s",my_json_string);
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
	return 1;
}


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

		ESP_LOGW("count_CO2_ppm_valid", "val=%d", count_CO2_ppm_valid );
		count_CO2_ppm_valid ++;

		ble_send_noti_int("CO2", (int)htons(data->ppm));
		
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

		ble_send_noti_int("PM2.5", htons(data->pm2_5_grimm));
		ble_send_noti_int("PM1.0", htons(data->pm1_0_grimm));
		ble_send_noti_int("PM10", htons(data->pm10_0_grimm));

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

struct _als_conf
{
	uint16_t resv:3;
	uint16_t SENS:1;
	uint16_t DG:1;
	uint16_t GAIN:1;
	uint16_t ALS_IT:4;
	uint16_t ALS_PERS:2;
	uint16_t INT_Ch:1;
	uint16_t CHAN_EN:1;
	uint16_t INT_EN:1;
	uint16_t SD:1;
}__attribute__((packed));

static esp_err_t als_conf_set(int it_time_ms )
{
//  	struct _als_conf als_conf;
	uint8_t buf[3];

	buf[0] = 0x00; // command : ALS_CONF_0

//  	memset((char*)&als_conf, 0, sizeof(als_conf));
//  
//  	als_conf.resv = 7; //low sensitivity (0: high sensitivity)
//  	als_conf.SENS = 1; //low sensitivity (0: high sensitivity)
//  	als_conf.DG   = 0; // Normal (1: Double Gain) ??
//  	als_conf.GAIN = 0; // Normal (1: Double Gain) ??
	switch(it_time_ms)
	{
//  		case  25 : als_conf.ALS_IT=12; break; // 1100
//  		case  50 : als_conf.ALS_IT=8; break;  // 1000
//  		case 100 : als_conf.ALS_IT=0; break;  // 0000
//  		case 200 : als_conf.ALS_IT=1; break;  // 0001
//  		case 400 : als_conf.ALS_IT=3; break;  // 0010
//  		case 800 : als_conf.ALS_IT=3; break;  // 0011
//  		default:
//  			ESP_LOGE("als_set", "Invalid integration Time set to 25msec ( 25,50,100,200,400,800 )");
//  			als_conf.ALS_IT=0xC;
//  			break;
		case  25 : buf[1] = 0x13; buf[2] = 0x04; break;  // 1100
		case  50 : buf[1] = 0x12; buf[2] = 0x04; break;  // 1000
		case 100 : buf[1] = 0x10; buf[2] = 0x04; break;  // 0000
		case 200 : buf[1] = 0x10; buf[2] = 0x44; break;  // 0001
		case 400 : buf[1] = 0x10; buf[2] = 0x84; break;  // 0010
		case 800 : buf[1] = 0x10; buf[2] = 0xC4; break;  // 0011
	}
//  	als_conf.ALS_PERS = 0; // Interrupt persistent 0:1 / 1:2 ...
//  	als_conf.INT_Ch   = 0; // ALS or WHITE
//  	als_conf.CHAN_EN  = 1; // ALS or ALS+WHITE
//  	als_conf.INT_EN   = 0; // Int Disable 
//  	als_conf.SD       = 0; // Power On 

		buf[1] = 0x1C; buf[2] = 0x04; // SENS=1 DG=1 GAIN=1 it=100ms

    // add command
    i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = 1000000 , // 1MHz //i2c_frequency,
        .scl_speed_hz = 100000 , // 100KHz //i2c_frequency,
        .device_address = LIGHT_SENSOR_I2C_DEV_ADDR, //조도센서 
    };

//  	uint16_t tmp;
//  	memcpy((char *)&tmp, (char *)&als_conf, 2); 
//  	buf[0] = 0x00;
//  	buf[1] = ((tmp>>0) & 0x00ff);
//  	buf[2] = ((tmp>>8) & 0x00ff);

//  	hexdump3("als_conf0", &als_conf, sizeof(als_conf));
	hexdump3("buf", buf, sizeof(buf));

    i2c_master_dev_handle_t dev_handle_i2c1;
    if (i2c_master_bus_add_device( tool_bus_handle_i2c1, &i2c_dev_conf, &dev_handle_i2c1) != ESP_OK) { return 1; }

    esp_err_t ret = i2c_master_transmit(dev_handle_i2c1, 
	                                    (uint8_t *)buf, 
										3, 
										I2C_TOOL_TIMEOUT_VALUE_MS);

    if (i2c_master_bus_rm_device(dev_handle_i2c1) != ESP_OK) 
	{
        return -20;
    }
//      return i2c_dev_write(dev, NULL, 0, buf, sizeof(buf));
    return ret;

}


void CO2_autozero_to_close(void)
{
	int argc = 4;
	char *argv[4] = { "imsi", "close", "15", "400"} ;

	do_CO2_autozero(argc, argv); //do_get_CO2에서 안에서 하면 Semaphore에서 DeadLock이 걸린다.
}



static uint8_t crc8_sht4x(uint8_t data[], size_t len)
{
    uint8_t crc = 0xff;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (size_t i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ G_POLYNOM_SHT4x : crc << 1;
    }
    return crc;
}

static uint8_t crc8_sgp40(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i)
    {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

//  int get_SHT4x_Serial_num(sht4x_t *dev, int cmd, sht4x_raw_data_t *s, int len)
int get_SHT4x_cmd_resp(sht4x_t *dev, int cmd, sht4x_raw_data_t res, int len)
{
#if I2C2__USING_GPIO
	memset((char *)res, 0, sizeof(sht4x_raw_data_t));

	uint8_t command = (uint8_t)cmd;

    esp_err_t ret = soft_i2c_master_write(bus_i2c2_gpio,SHT4X_I2C_ADDRESS, (uint8_t*)&command, 1);
//  	ESP_GOTO_ON_ERROR(ret, error, "get_SHT4x_cmd_resp", "Error writing to I2C device");
//      esp_err_t ret = i2c_master_transmit(dev_handle_i2c2, (uint8_t*)&command, 1, I2C_TOOL_TIMEOUT_VALUE_MS);

    vTaskDelay(pdMS_TO_TICKS(10) + 1);  //바로 응답하지 못해서(NACK) : need Delay

	if ( len != 0 ) 
	{
//  		ret = i2c_master_receive(dev_handle_i2c2, (uint8_t*)res, len, I2C_TOOL_TIMEOUT_VALUE_MS);
		ret = soft_i2c_master_read(bus_i2c2_gpio, SHT4X_I2C_ADDRESS, (uint8_t*)res, len);
//  		ESP_GOTO_ON_ERROR(ret, error, "get_SHT4x_cmd_resp", "Error reading from I2C device");
	    if (ret == ESP_OK) 
		{
	        ESP_LOGI(TAG, "SHT4x I2C Read OK : Get Serial Num(Receive)");
			hexdump3("SHT4x cmd_resp Raw data", res , len);

			ESP_LOGW("SHT4x_cmd_resp", "crc ( 0x%02x, 0x%02x )", crc8_sht4x(res,2), crc8_sht4x(res+3,2));
			if (res[2] != crc8_sht4x(res, 2) || res[5] != crc8_sht4x(res + 3, 2))
			{
			    ESP_LOGE(TAG, "get_SHT4x_cmd_resp : Invalid CRC");
			    return ESP_ERR_INVALID_CRC;
			}
			
	    }
		else
		{
			return -1;
		}

	}
	return 0;
#else
    i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = 1000000 , // 1MHz //i2c_frequency,
        .scl_speed_hz = 100000 , // 100KHz //i2c_frequency,
        .device_address = dev->i2c_dev.addr,
    };
	ESP_LOGW("get_SHT4x_cmd_resp", "chip_addr=%02x, cmd=%02x", dev->i2c_dev.addr, cmd);

    if (i2c_master_bus_add_device( tool_bus_handle_i2c2, 
	                              &i2c_dev_conf, 
								  &dev_handle_i2c2) != ESP_OK) 
	{
        return 1;
    }

//  SHT4x_get_Serial_num_retry :
	memset((char *)res, 0, sizeof(sht4x_raw_data_t));

	uint8_t command = (uint8_t)cmd;
    esp_err_t ret = i2c_master_transmit(dev_handle_i2c2, (uint8_t*)&command, 1, I2C_TOOL_TIMEOUT_VALUE_MS);

    vTaskDelay(pdMS_TO_TICKS(10) + 1);  //바로 응답하지 못해서(NACK) : need Delay

	if ( len != 0 ) 
	{
		ret = i2c_master_receive(dev_handle_i2c2, (uint8_t*)res, len, I2C_TOOL_TIMEOUT_VALUE_MS);
	    if (ret == ESP_OK) {
	        ESP_LOGI(TAG, "SHT4x I2C Fead OK : Get Serial Num(Receive)");
			hexdump3("SHT4x cmd_resp Raw data", res , len);

			ESP_LOGW("SHT4x_cmd_resp", "crc ( 0x%02x, 0x%02x )", crc8_sht4x(res,2), crc8_sht4x(res+3,2));
			if (res[2] != crc8_sht4x(res, 2) || res[5] != crc8_sht4x(res + 3, 2))
			{
			    ESP_LOGE(TAG, "get_SHT4x_cmd_resp : Invalid CRC");
			    return ESP_ERR_INVALID_CRC;
			}
			
	    } else if (ret == ESP_ERR_TIMEOUT) {
	        ESP_LOGW(TAG, "SHT4x I2C Bus is busy: Get Serial Num(Receive)");
			memset((char *)res, 0, sizeof(sht4x_raw_data_t));
	    } else {
	        ESP_LOGW(TAG, "SHT4x I2C Read Failed: Get Serial Num(Receive)");
			memset((char *)res, 0, sizeof(sht4x_raw_data_t));
	    }
	}

    if (i2c_master_bus_rm_device(dev_handle_i2c2) != ESP_OK) {
        return -20;
    }

	return 0;
#endif
}

static inline uint16_t swap16_sgp40(uint16_t v)
{
    return (v << 8) | (v >> 8);
}

static esp_err_t send_cmd_sgp40(sgp40_t *dev, uint16_t cmd, uint16_t *data, size_t words)
{
	uint8_t buf[2 + words * 3];

    // add command
    *(uint16_t *)buf = swap16_sgp40(cmd);
    if (data && words)
        // add arguments
        for (size_t i = 0; i < words; i++)
        {
            uint8_t *p = buf + 2 + i * 3;
            *(uint16_t *)p = swap16_sgp40(data[i]);
   	         *(p + 2) = crc8_sgp40(p, 2);
        }

    ESP_LOGV(TAG, "Sending buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);


#if I2C2__USING_GPIO
	hexdump3("SGP40 send_cmd_sgp40", buf, 2+words*3);
    esp_err_t ret = soft_i2c_master_write(bus_i2c2_gpio, SGP40_ADDR, (uint8_t *)buf, 2+words*3 );
//  	ESP_GOTO_ON_ERROR(ret, error, "send_cmd_sgp40", "Error writing to I2C device");
    return ret;
#else

    i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = 1000000 , // 1MHz //i2c_frequency,
        .scl_speed_hz = 100000 , // 100KHz //i2c_frequency,
        .device_address = dev->i2c_dev.addr,
    };
	ESP_LOGW("send_cmd_sgp40", "chip_addr=%02x, words=%d, cmd=%04x", dev->i2c_dev.addr, words, cmd);

    i2c_master_dev_handle_t dev_handle_i2c2;
    if (i2c_master_bus_add_device(  tool_bus_handle_i2c2, &i2c_dev_conf, &dev_handle_i2c2) != ESP_OK) { return 1; }

	hexdump3("SGP40 send_cmd_sgp40", buf, 2+words*3);
    esp_err_t ret = i2c_master_transmit(dev_handle_i2c2, 
	                                    (uint8_t *)buf, 
										2+words*3, 
										I2C_TOOL_TIMEOUT_VALUE_MS);

    if (i2c_master_bus_rm_device(dev_handle_i2c2) != ESP_OK) 
	{
        return -20;
    }
//      return i2c_dev_write(dev, NULL, 0, buf, sizeof(buf));
    return ret;
#endif
}

static esp_err_t read_resp_sgp40(sgp40_t *dev, uint16_t *data, size_t words)
{
    uint8_t buf[words * 3];
#if I2C2__USING_GPIO
	esp_err_t ret = soft_i2c_master_read(bus_i2c2_gpio, SGP40_ADDR, (uint8_t*)buf, words*(2+1));
//  	ESP_GOTO_ON_ERROR(ret, error, "read_resp_cmd_sgp40", "Error reading from I2C device");

    ESP_LOGV(TAG, "Received buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8_sgp40(p, 2);
        if (crc != *(p + 2))
        {
            ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return ESP_ERR_INVALID_CRC;
        }
        data[i] = swap16_sgp40(*(uint16_t *)p);
    }

    return ret;
#else

    i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = 1000000 , // 1MHz //i2c_frequency,
        .scl_speed_hz = 100000 , // 100KHz //i2c_frequency,
        .device_address = dev->i2c_dev.addr,
    };
	ESP_LOGW("read_resp_sgp40", "chip_addr=%02x, words=%d", dev->i2c_dev.addr, words);

    i2c_master_dev_handle_t dev_handle_i2c2;
    if (i2c_master_bus_add_device(  tool_bus_handle_i2c2, &i2c_dev_conf, &dev_handle_i2c2) != ESP_OK) { return 1; }

//  	vTaskDelay(pdMS_TO_TICKS(wait_ms));
	esp_err_t ret = i2c_master_receive(dev_handle_i2c2, (uint8_t*)buf, words*(2+1), I2C_TOOL_TIMEOUT_VALUE_MS);

    ESP_LOGV(TAG, "Received buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8_sgp40(p, 2);
        if (crc != *(p + 2))
        {
            ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return ESP_ERR_INVALID_CRC;
        }
        data[i] = swap16_sgp40(*(uint16_t *)p);
    }

    if (i2c_master_bus_rm_device(dev_handle_i2c2) != ESP_OK) {
        return -20;
    }

//      return ESP_OK;
    return ret;
#endif
}


int get_SGP40_cmd_resp_inout(sgp40_t *dev, int cmd, uint32_t timeout_ms, 
        uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
//      I2C_DEV_CHECK(&dev->i2c_dev, send_cmd(&dev->i2c_dev, cmd, out_data, out_words));
    send_cmd_sgp40(dev, cmd, out_data, out_words);
    if (timeout_ms)
    {
        if (timeout_ms > 10)
            vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        else
            ets_delay_us(timeout_ms * 1000);
    }
    if (in_data && in_words)
	{
//          I2C_DEV_CHECK(&dev->i2c_dev, read_resp(&dev->i2c_dev, in_data, in_words));
        read_resp_sgp40(dev, in_data, in_words);
	}
	return 0;
}

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


static esp_err_t sgp40_measure_raw_shcho(sgp40_t *dev, float humidity, float temperature, uint16_t *raw);
static esp_err_t sgp40_measure_voc_shcho(sgp40_t *dev, float humidity, float temperature, int32_t *voc_index)
{
//      CHECK_ARG(dev && voc_index);

    uint16_t raw;
//      CHECK(sgp40_measure_raw(dev, humidity, temperature, &raw));
    sgp40_measure_raw_shcho(dev, humidity, temperature, &raw);

    VocAlgorithm_process(&dev->voc, raw, voc_index);

    return ESP_OK;
}



int get_SGP40_cmd_resp(sgp40_t *dev, int cmd, uint16_t *data, int words, int wait_ms)
{
	uint8_t sgp40_resp[10*3];

	char i2c_cmd[2] ;
	i2c_cmd[0] = (char)((cmd & 0xff00) >> 8);
	i2c_cmd[1] = (char)((cmd & 0x00ff) >> 0);
	hexdump3("SGP40 cmd packet", i2c_cmd, sizeof(i2c_cmd));

#if I2C2__USING_GPIO
    esp_err_t ret = soft_i2c_master_write(bus_i2c2_gpio, SGP40_ADDR, (uint8_t *)i2c_cmd, 2 );
//  	ESP_GOTO_ON_ERROR(ret, error, "send_cmd_sgp40", "Error writing to I2C device");

	vTaskDelay(pdMS_TO_TICKS(wait_ms));

	ret = soft_i2c_master_read(bus_i2c2_gpio, SGP40_ADDR, (uint8_t*)sgp40_resp, words*(2+1));
//  	ESP_GOTO_ON_ERROR(ret, error, "send_cmd_sgp40", "Error reading from I2C device");
    if (ret == ESP_OK) 
	{
        ESP_LOGI(TAG, "SGP40 I2C Fead OK : Get Serial Num(Receive)");
		hexdump3("SGP40_cmd_resp Raw data", sgp40_resp , words*(2+1));

		for (size_t i = 0; i < words; i++)
		{
//  		    uint8_t *p = buf + i * 3;
		    uint8_t *p = sgp40_resp + i * 3;
		    uint8_t crc = crc8_sgp40(p, 2);
		    if (crc != *(p + 2))
		    {
		        ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
		        return ESP_ERR_INVALID_CRC;
		    }
		    data[i] = swap16_sgp40(*(uint16_t *)p);
		}
    }
	else
	{
		return -1;
	}

	return 0;
#else
	uint8_t sgp40_resp[10*3];

    i2c_device_config_t i2c_dev_conf = {
//          .scl_speed_hz = 1000000 , // 1MHz //i2c_frequency,
        .scl_speed_hz = 100000 , // 100KHz //i2c_frequency,
        .device_address = dev->i2c_dev.addr,
    };
	ESP_LOGW("get_SGP40_cmd_resp", "chip_addr=%02x, words=%d, cmd=%04x", dev->i2c_dev.addr, words, cmd);

    i2c_master_dev_handle_t dev_handle_i2c2;
    if (i2c_master_bus_add_device(  tool_bus_handle_i2c2, 
	                               &i2c_dev_conf, 
								   &dev_handle_i2c2) != ESP_OK) 
	{
        return 1;
    }

    esp_err_t ret = i2c_master_transmit(dev_handle_i2c2, 
	                                    (uint8_t *)i2c_cmd, 
										2, 
										I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) 
	{
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW("get_SGP40_Serial_num", "Bus is busy");
		return -1;
    } else {
        ESP_LOGW("get_SGP40_Serial_num", "Read failed(transmit_receive)");
		return -1;
    }

//      vTaskDelay( wait_ms / portTICK_PERIOD_MS );
	vTaskDelay(pdMS_TO_TICKS(wait_ms));
	ret = i2c_master_receive(dev_handle_i2c2, (uint8_t*)sgp40_resp, words*(2+1), I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SGP40 I2C Fead OK : Get Serial Num(Receive)");
		hexdump3("SGP40_cmd_resp Raw data", sgp40_resp , words*(2+1));

		for (size_t i = 0; i < words; i++)
		{
//  		    uint8_t *p = buf + i * 3;
		    uint8_t *p = sgp40_resp + i * 3;
		    uint8_t crc = crc8_sgp40(p, 2);
		    if (crc != *(p + 2))
		    {
		        ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
		        return ESP_ERR_INVALID_CRC;
		    }
		    data[i] = swap16_sgp40(*(uint16_t *)p);
		}
	
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "SGP40 I2C Bus is busy: Get Serial Num(Receive)");
    } else {
        ESP_LOGW(TAG, "SGP40 I2C Read Failed: Get Serial Num(Receive)");
    }

    if (i2c_master_bus_rm_device(dev_handle_i2c2) != ESP_OK) {
        return -20;
    }

	return 0;
#endif
}

esp_err_t sht4x_compute_values_shcho(sht4x_raw_data_t raw_data, float *temperature, float *humidity)
{

	//shcho comment: E (1063) i2c: CONFLICT! driver_ng is not allowed to be used with this old driver
//      CHECK_ARG(raw_data && (temperature || humidity));

    if (temperature)
        *temperature = ((uint16_t)raw_data[0] << 8 | raw_data[1]) * 175.0 / 65535.0 - 45.0;

    if (humidity)
        *humidity = ((uint16_t)raw_data[3] << 8 | raw_data[4]) * 125.0 / 65535.0 - 6.0;

    return ESP_OK;
}

static inline uint8_t get_meas_cmd(sht4x_t *dev)
{
    switch (dev->heater)
    {
        case SHT4X_HEATER_HIGH_LONG:
            return SHT4X_CMD_MEAS_H_HIGH_LONG;
        case SHT4X_HEATER_HIGH_SHORT:
            return SHT4X_CMD_MEAS_H_HIGH_SHORT;
        case SHT4X_HEATER_MEDIUM_LONG:
            return SHT4X_CMD_MEAS_H_MED_LONG;
        case SHT4X_HEATER_MEDIUM_SHORT:
            return SHT4X_CMD_MEAS_H_MED_SHORT;
        case SHT4X_HEATER_LOW_LONG:
            return SHT4X_CMD_MEAS_H_LOW_LONG;
        case SHT4X_HEATER_LOW_SHORT:
            return SHT4X_CMD_MEAS_H_LOW_SHORT;
        default:
            switch (dev->repeatability)
            {
                case SHT4X_HIGH:
                    return SHT4X_CMD_MEAS_HIGH;
                case SHT4X_MEDIUM:
                    return SHT4X_CMD_MEAS_MED;
                default:
                    return SHT4X_CMD_MEAS_LOW;
            }
    }
	return 0;	// shcho add
}

static esp_err_t sgp40_measure_raw_shcho(sgp40_t *dev, float humidity, float temperature, uint16_t *raw)
{
//  	CHECK_ARG(dev && raw);

    uint16_t params[2];
    if (isnan(humidity) || isnan(temperature))
    {
        params[0] = 0x8000;
        params[1] = 0x6666;
        ESP_LOGW(TAG, "Uncompensated measurement");
    }
    else
    {
        if (humidity < 0)
            humidity = 0;
        else if (humidity > 100)
            humidity = 100;

        if (temperature < -45)
            temperature = -45;
        else if (temperature > 129.76)
            temperature = 129.76;

        params[0] = (uint16_t)(humidity / 100.0 * 65536);
        params[1] = (uint16_t)((temperature + 45) / 175.0 * 65535);
    }

//      return (dev, CMD_MEASURE_RAW, TIME_MEASURE_RAW, params, 2, raw, 1);
    return get_SGP40_cmd_resp_inout(dev, SGP40_CMD_MEASURE_RAW, SGP40_TIME_MEASURE_RAW, params, 2, raw, 1);


}

int gpio15_16_set_to_input(void) // GPIO_3 --> GPIO_8
{
    gpio_config_t io_conf;

    // detect Is it Wearable : Static은 Pull-up :10K GPIO_38(MIX_A0) / GPIO_39(MUX_A0)
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = ((1ULL << 15) | (1ULL << 16));
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

int do_rht_voc_report(sht4x_t *dev_sht4x, sgp40_t *dev_sgp40,
                  float temperature, float humidity, int voc_index )
{
	char  buffer_sht40_temp[30];
	char  buffer_sht40_humi[30];
	char  buffer_sgp40[30];
    ESP_LOGI(JSON_TAG, "Serialize.....RHT_VOC");
    cJSON *root;
   	root = cJSON_CreateObject();

   	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);

//     	cJSON_AddStringToObject(root, "SHT40_Serial_num",   mode);

	memset(buffer_sht40_temp, 0, sizeof(buffer_sht40_temp));
	sprintf(buffer_sht40_temp, "%" PRIu32,dev_sht4x->serial);
   	cJSON_AddStringToObject(root, "SHT40_Serial_num",  buffer_sht40_temp);

	memset(buffer_sht40_temp, 0, sizeof(buffer_sht40_temp));
	sprintf(buffer_sht40_temp, "%3.2f", temperature);
   	cJSON_AddNumberToObject(root, "SHT40_T",  atof(buffer_sht40_temp));

	memset(buffer_sht40_humi, 0, sizeof(buffer_sht40_humi));
	sprintf(buffer_sht40_humi, "%3.2f", humidity);
   	cJSON_AddNumberToObject(root, "SHT40_RH", atof(buffer_sht40_humi));


	memset(buffer_sgp40, 0, sizeof(buffer_sgp40));
	sprintf(buffer_sgp40, "%04X_%04X_%04X", dev_sgp40->serial[0],
	                                  dev_sgp40->serial[1], 
									  dev_sgp40->serial[2]);
   	cJSON_AddStringToObject(root, "SGP40_Serial_num",  buffer_sgp40);
   	cJSON_AddNumberToObject(root, "SGP40_Voc_index",       voc_index);
   	cJSON_AddStringToObject(root, "SGP40_Voc_index_name",  voc_index_name(voc_index));
	sprintf(buffer_sgp40, "%3.2f", temperature);
   	cJSON_AddNumberToObject(root, "SGP40_T",  atof(buffer_sgp40));
	sprintf(buffer_sgp40, "%3.2f", humidity);
   	cJSON_AddNumberToObject(root, "SGP40_RH", atof(buffer_sgp40));

    char *my_json_string = cJSON_Print(root);

   	ESP_LOGI("RHT_Voc", "my_json_string\n%s",my_json_string);
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

	ble_send_noti_str("Temperature", buffer_sht40_temp);
//  	ble_send_noti_str("KEEP_ALIVE", "term1");
    vTaskDelay(100 / portTICK_PERIOD_MS);
	ble_send_noti_str("Humidity",    buffer_sht40_humi);
    vTaskDelay(100 / portTICK_PERIOD_MS);
//  	ble_send_noti_str("KEEP_ALIVE", "term2");
	ble_send_noti_int("TVOC",        voc_index);
    vTaskDelay(100 / portTICK_PERIOD_MS);
	return 0;
}

void i2c1_i2c2_sensor_task(void *arg)
{
	ESP_LOGW("check", "1:1st force set : count_CO2_ppm_valid=%d / flag_CO2_autozero_close_run=%d", 
	                                   count_CO2_ppm_valid, flag_CO2_autozero_close_run );
	ESP_LOGW("check", "2:1st force set : count_CO2_ppm_valid=%d / flag_CO2_autozero_close_run=%d", 
	                                   count_CO2_ppm_valid, flag_CO2_autozero_close_run );
	ESP_LOGW("check", "3:1st force set : count_CO2_ppm_valid=%d / flag_CO2_autozero_close_run=%d", 
	                                   count_CO2_ppm_valid, flag_CO2_autozero_close_run );

	if( flag_CO2_autozero_close_run != 0 ) 
	{
		ESP_LOGW("check", "1st force set : count_CO2_ppm_valid=%d / flag_CO2_autozero_close_run=%d", 
		                              count_CO2_ppm_valid, flag_CO2_autozero_close_run );
//  		내부에서 Take/Give한다. cli에서도 사용하기 때문에
		xSemaphoreTake(sema_i2c1, portMAX_DELAY);
			CO2_autozero_to_close();
		xSemaphoreGive(sema_i2c1);
	}


	xSemaphoreTake(sema_i2c1, portMAX_DELAY);

		set_PM2008_mode(PM2008_CMD_CLOSE, 0x00);              vTaskDelay(5000 / portTICK_PERIOD_MS);
		set_PM2008_mode(PM2008_CMD_SETUP_CONTINUOUS, 0xffff); vTaskDelay(2000 / portTICK_PERIOD_MS);
	//  	set_PM2008_mode(PM2008_CMD_SETUP_TIMING_MEASURE, 180);
	//      vTaskDelay(2000 / portTICK_PERIOD_MS);
	
	xSemaphoreGive(sema_i2c1);


	// ALS ( Ambient Light Sensor : Conf : integration 25msec )
	xSemaphoreTake(sema_i2c1, portMAX_DELAY);
		als_conf_set(50); //이것을 무시하고 // SENS=1 DG=1 GAIN=1 it=100ms
	xSemaphoreGive(sema_i2c1);







//-------------------------------------------------------------------------------------------------
	int flag_SHT4x_is_OK = 0 ;
	int flag_SGP40_is_OK = 0 ;
	// 1. ZMOD reset : Power on시에는  0x32가 보이다가  
	//                 바로 사라짐
	ZMOD_Reset_GPIO(0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
//  	ZMOD_Reset_GPIO(1);

	
//		=========================================================================
#if I2C2__USING_GPIO // I2C2 : soft_i2c : setup
//  	gpio15_16_set_to_input();
//      esp_err_t ret = ESP_OK;
// 	    soft_i2c_master_bus_t bus = NULL;
    soft_i2c_master_config_t config = {
        .scl_pin = GPIO_I2C2_SCL,
        .sda_pin = GPIO_I2C2_SDA,
//          .scl_pin = 14,
//          .sda_pin = 48,
        .freq = SOFT_I2C_100KHZ
    };

    ESP_LOGW("i2c2_sensor_task", "--------------------- Initialize and configure the software I2C bus -----------------------");
    ESP_LOGW("i2c2_sensor_task", "--------------------- Initialize and configure the software I2C bus -----------------------");
    /* Initialize and configure the software I2C bus */
    ESP_ERROR_CHECK(soft_i2c_master_new(&config, &bus_i2c2_gpio));
	
#endif

//		=========================================================================
	// 
	// 2. SHT4x
    memset(&dev_sht4x, 0, sizeof(dev_sht4x));
    dev_sht4x.i2c_dev.addr = SHT4X_I2C_ADDRESS;
    dev_sht4x.i2c_dev.cfg.master.clk_speed = I2C2_FREQ_HZ;
    dev_sht4x.repeatability = SHT4X_HIGH;
    dev_sht4x.heater = SHT4X_HEATER_OFF;
//  	ESP_ERROR_CHECK(sht4x_init_desc(&dev_sht4x, 0, 16, 15));

	// 2. SGP40 : SHT4x등 온습도가 반드시 있어야 함
    memset(&dev_sgp40, 0, sizeof(dev_sgp40));
    dev_sgp40.i2c_dev.addr = SGP40_ADDR;
    dev_sgp40.i2c_dev.cfg.master.clk_speed = I2C2_FREQ_HZ;
//  	ESP_ERROR_CHECK(sgp40_init_desc(&dev_sgp40, 0, 16, 15));


    sht4x_raw_data_t resp;


	// 3. SHT4x Serial_num
	xSemaphoreTake(sema_i2c2, portMAX_DELAY);

		memset( resp, 0, sizeof(resp));
		get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_SERIAL, resp, sizeof(resp));
	    dev_sht4x.serial = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) | ((uint32_t)resp[3] << 8) | resp[4];
		ESP_LOGW(TAG, "SHT4x initilalized. Serial: %" PRIu32, dev_sht4x.serial);
		get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_RESET, resp, 0);
		if( dev_sht4x.serial != 0 )
		{
			flag_SHT4x_is_OK = 1 ; 
		}

	xSemaphoreGive(sema_i2c2);
	
	// 3. SGP40 Serial_num -> get featureset --> init VocAlgorithm
	xSemaphoreTake(sema_i2c2, portMAX_DELAY);

		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SERIAL, dev_sgp40.serial, 3, SGP40_TIME_SERIAL);
		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_FEATURESET, &dev_sgp40.featureset, 1, SGP40_TIME_FEATURESET);
	    ESP_LOGW(TAG, "SGP40 initilalized. Serial: %04X_%04X_%04X featureset 0x%04x",
	            dev_sgp40.serial[0], dev_sgp40.serial[1], dev_sgp40.serial[2], dev_sgp40.featureset);

		if(    ( dev_sgp40.serial[0] != 0 ) 
		    || ( dev_sgp40.serial[1] != 0 )
		    || ( dev_sgp40.serial[2] != 0 ) )
		{
			flag_SGP40_is_OK = 1 ; 
		}
		VocAlgorithm_init(&dev_sgp40.voc);
		hexdump3("Voc Algo init data", &dev_sgp40.voc , sizeof(dev_sgp40.voc));
//  		No need : sgp40 example does not execute SOFT)RESET
//  		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SOFT_RESET, NULL, 0, SGP40_TIME_SOFT_RESET);

	xSemaphoreGive(sema_i2c2);

    vTaskDelay(100 / portTICK_PERIOD_MS);
	float temperature, humidity;
//-------------------------------------------------------------------------------------------------



	
	while(1)
	{
		{
			if( flag_SHT4x_is_OK != 1 )
			{
				ESP_LOGE("SHT4x", "Serial Num is not valid");
				// 3. SHT4x Serial_num
				xSemaphoreTake(sema_i2c2, portMAX_DELAY);
			
					memset( resp, 0, sizeof(resp));
					get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_SERIAL, resp, sizeof(resp));
				    dev_sht4x.serial = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) | ((uint32_t)resp[3] << 8) | resp[4];
					ESP_LOGW(TAG, "SHT4x initilalized. Serial: %" PRIu32 "(again)", dev_sht4x.serial);
					get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_RESET, resp, 0);
					if( dev_sht4x.serial != 0 )
					{
						flag_SHT4x_is_OK = 1 ; 
					}
			
				xSemaphoreGive(sema_i2c2);
			}
		
			if( flag_SGP40_is_OK != 1 )
			{
				ESP_LOGE("SGP40", "Serial Num is not valid");
				// 3. SGP40 Serial_num -> get featureset --> init VocAlgorithm
				xSemaphoreTake(sema_i2c2, portMAX_DELAY);
			
					get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SERIAL, dev_sgp40.serial, 3, SGP40_TIME_SERIAL);
					get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_FEATURESET, &dev_sgp40.featureset, 1, SGP40_TIME_FEATURESET);
				    ESP_LOGW(TAG, "SGP40 initilalized. Serial: %04X_%04X_%04X featureset 0x%04x",
				            dev_sgp40.serial[0], dev_sgp40.serial[1], dev_sgp40.serial[2], dev_sgp40.featureset);
	
					if(    ( dev_sgp40.serial[0] != 0 ) 
					    || ( dev_sgp40.serial[1] != 0 )
					    || ( dev_sgp40.serial[2] != 0 ) )
					{
						flag_SGP40_is_OK = 1 ; 
					}
					else
					{
					    ESP_LOGW(TAG, "SGP40 initilalized. Serial: %04X_%04X_%04X featureset 0x%04x ( invalid Serial_num )",
					            dev_sgp40.serial[0], dev_sgp40.serial[1], dev_sgp40.serial[2], dev_sgp40.featureset);
					}
					VocAlgorithm_init(&dev_sgp40.voc);
					hexdump3("Voc Algo init data again", &dev_sgp40.voc , sizeof(dev_sgp40.voc));
			//  		No need : sgp40 example does not execute SOFT)RESET
			//  		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SOFT_RESET, NULL, 0, SGP40_TIME_SOFT_RESET);
			
				xSemaphoreGive(sema_i2c2);
			}
	
			if( flag_SHT4x_is_OK == 1 && flag_SGP40_is_OK == 1 )
			{
				// 4. 온습도
				xSemaphoreTake(sema_i2c2, portMAX_DELAY);
					get_SHT4x_cmd_resp(&dev_sht4x, get_meas_cmd(&dev_sht4x), resp, sizeof(resp));
				xSemaphoreGive(sema_i2c2);
		
		    	sht4x_compute_values_shcho(resp, &temperature, &humidity);
				ESP_LOGW("sht4x Sensor", " %.2f °C, %.2f %%\n", temperature, humidity);
		
				// 5. 온습도 --> VOC Index
				int32_t voc_index;
				xSemaphoreTake(sema_i2c2, portMAX_DELAY);
					sgp40_measure_voc_shcho(&dev_sgp40, humidity, temperature, &voc_index);
				xSemaphoreGive(sema_i2c2);
		
				ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC index: %3" PRIi32 ", Air is [%s]",
							temperature, humidity, voc_index, voc_index_name(voc_index));
		
				do_rht_voc_report(&dev_sht4x, &dev_sgp40, temperature, humidity, voc_index );
			}
	
	
//  	       	vTaskDelay(10000 / portTICK_PERIOD_MS);
	       	vTaskDelay(1000 / portTICK_PERIOD_MS);
		}




// I2C1  : Fan , CM1106(CO2), PM2008, ALS(조도센서)
		xSemaphoreTake(sema_i2c1, portMAX_DELAY);
		set_fan_pwm();
		xSemaphoreGive(sema_i2c1);

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

		xSemaphoreTake(sema_i2c1, portMAX_DELAY);
		do_get_als(); // 
		xSemaphoreGive(sema_i2c1);

		//이미 위에서 1로 변경되니까 1에서 시작하자
		ESP_LOGW("check", "count_CO2_ppm_valid=%d / flag_CO2_autozero_close_run=%d", count_CO2_ppm_valid, flag_CO2_autozero_close_run );
		if( ((count_CO2_ppm_valid % 50) == 10) && ( flag_CO2_autozero_close_run != 0 ) )
		{
//  //  			char close[]="close";
//  //  			char cali_day = "15";
//  //  			char cali_ppm="400";
//  			int argc = 4;
//  			char *argv[4] = { "imsi", "close", "15", "400"} ;
//  //  			argv[1] = close;
//  //  			argv[2] = cali_day;
//  //  			argv[3] = cali_ppm;
//  
//  			//내부에서 Sema Take하고 Release를 한다. --> 내부에서 삭제
//  			do_CO2_autozero(argc, argv); //do_get_CO2에서 안에서 하면 Semaphore에서 DeadLock이 걸린다.
			xSemaphoreTake(sema_i2c1, portMAX_DELAY);
			CO2_autozero_to_close();
			xSemaphoreGive(sema_i2c1);
		}

       	vTaskDelay(10000 / portTICK_PERIOD_MS);




	}
}


void i2c2_sensor_task(void *arg) // Not used : i2c2 sensor :get value @ i2c1_i2c2_sensor_task
{
	int flag_SHT4x_is_OK = 0 ;
	int flag_SGP40_is_OK = 0 ;
	// 1. ZMOD reset : Power on시에는  0x32가 보이다가  
	//                 바로 사라짐
	ZMOD_Reset_GPIO(0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
	ZMOD_Reset_GPIO(1);

	
#if I2C2__USING_GPIO
//      esp_err_t ret = ESP_OK;
// 	    soft_i2c_master_bus_t bus = NULL;
    soft_i2c_master_config_t config = {
        .scl_pin = GPIO_I2C2_SCL,
        .sda_pin = GPIO_I2C2_SDA,
        .freq = SOFT_I2C_100KHZ
    };

    ESP_LOGW("i2c2_sensor_task", "--------------------- Initialize and configure the software I2C bus -----------------------");
    ESP_LOGW("i2c2_sensor_task", "--------------------- Initialize and configure the software I2C bus -----------------------");
    /* Initialize and configure the software I2C bus */
    ESP_ERROR_CHECK(soft_i2c_master_new(&config, &bus_i2c2_gpio));
		// 

	
#endif

	// 2. SHT4x
    memset(&dev_sht4x, 0, sizeof(dev_sht4x));
    dev_sht4x.i2c_dev.addr = SHT4X_I2C_ADDRESS;
    dev_sht4x.i2c_dev.cfg.master.clk_speed = I2C2_FREQ_HZ;
    dev_sht4x.repeatability = SHT4X_HIGH;
    dev_sht4x.heater = SHT4X_HEATER_OFF;
//  	ESP_ERROR_CHECK(sht4x_init_desc(&dev_sht4x, 0, 16, 15));

	// 2. SGP40 : SHT4x등 온습도가 반드시 있어야 함
    memset(&dev_sgp40, 0, sizeof(dev_sgp40));
    dev_sgp40.i2c_dev.addr = SGP40_ADDR;
    dev_sgp40.i2c_dev.cfg.master.clk_speed = I2C2_FREQ_HZ;
//  	ESP_ERROR_CHECK(sgp40_init_desc(&dev_sgp40, 0, 16, 15));


    sht4x_raw_data_t resp;


	// 3. SHT4x Serial_num
	xSemaphoreTake(sema_i2c2, portMAX_DELAY);

		memset( resp, 0, sizeof(resp));
		get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_SERIAL, resp, sizeof(resp));
	    dev_sht4x.serial = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) | ((uint32_t)resp[3] << 8) | resp[4];
		ESP_LOGW(TAG, "SHT4x initilalized. Serial: %" PRIu32, dev_sht4x.serial);
		get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_RESET, resp, 0);
		if( dev_sht4x.serial != 0 )
		{
			flag_SHT4x_is_OK = 1 ; 
		}

	xSemaphoreGive(sema_i2c2);
	
	// 3. SGP40 Serial_num -> get featureset --> init VocAlgorithm
	xSemaphoreTake(sema_i2c2, portMAX_DELAY);

		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SERIAL, dev_sgp40.serial, 3, SGP40_TIME_SERIAL);
		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_FEATURESET, &dev_sgp40.featureset, 1, SGP40_TIME_FEATURESET);
	    ESP_LOGW(TAG, "SGP40 initilalized. Serial: %04X_%04X_%04X featureset 0x%04x",
	            dev_sgp40.serial[0], dev_sgp40.serial[1], dev_sgp40.serial[2], dev_sgp40.featureset);

		if(    ( dev_sgp40.serial[0] != 0 ) 
		    || ( dev_sgp40.serial[1] != 0 )
		    || ( dev_sgp40.serial[2] != 0 ) )
		{
			flag_SGP40_is_OK = 1 ; 
		}
		VocAlgorithm_init(&dev_sgp40.voc);
		hexdump3("Voc Algo init data", &dev_sgp40.voc , sizeof(dev_sgp40.voc));
//  		No need : sgp40 example does not execute SOFT)RESET
//  		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SOFT_RESET, NULL, 0, SGP40_TIME_SOFT_RESET);

	xSemaphoreGive(sema_i2c2);

    vTaskDelay(100 / portTICK_PERIOD_MS);
	float temperature, humidity;

	while(1)
	{

		if( flag_SHT4x_is_OK != 1 )
		{
			ESP_LOGE("SHT4x", "Serial Num is not valid");
			// 3. SHT4x Serial_num
			xSemaphoreTake(sema_i2c2, portMAX_DELAY);
		
				memset( resp, 0, sizeof(resp));
				get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_SERIAL, resp, sizeof(resp));
			    dev_sht4x.serial = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) | ((uint32_t)resp[3] << 8) | resp[4];
				ESP_LOGW(TAG, "SHT4x initilalized. Serial: %" PRIu32 "(again)", dev_sht4x.serial);
				get_SHT4x_cmd_resp(&dev_sht4x, SHT4X_CMD_RESET, resp, 0);
				if( dev_sht4x.serial != 0 )
				{
					flag_SHT4x_is_OK = 1 ; 
				}
		
			xSemaphoreGive(sema_i2c2);
		}
	
		if( flag_SGP40_is_OK != 1 )
		{
			ESP_LOGE("SGP40", "Serial Num is not valid");
			// 3. SGP40 Serial_num -> get featureset --> init VocAlgorithm
			xSemaphoreTake(sema_i2c2, portMAX_DELAY);
		
				get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SERIAL, dev_sgp40.serial, 3, SGP40_TIME_SERIAL);
				get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_FEATURESET, &dev_sgp40.featureset, 1, SGP40_TIME_FEATURESET);
			    ESP_LOGW(TAG, "SGP40 initilalized. Serial: %04X_%04X_%04X featureset 0x%04x",
			            dev_sgp40.serial[0], dev_sgp40.serial[1], dev_sgp40.serial[2], dev_sgp40.featureset);

				if(    ( dev_sgp40.serial[0] != 0 ) 
				    || ( dev_sgp40.serial[1] != 0 )
				    || ( dev_sgp40.serial[2] != 0 ) )
				{
					flag_SGP40_is_OK = 1 ; 
				}
				else
				{
				    ESP_LOGW(TAG, "SGP40 initilalized. Serial: %04X_%04X_%04X featureset 0x%04x ( invalid Serial_num )",
				            dev_sgp40.serial[0], dev_sgp40.serial[1], dev_sgp40.serial[2], dev_sgp40.featureset);
				}
				VocAlgorithm_init(&dev_sgp40.voc);
				hexdump3("Voc Algo init data again", &dev_sgp40.voc , sizeof(dev_sgp40.voc));
		//  		No need : sgp40 example does not execute SOFT)RESET
		//  		get_SGP40_cmd_resp(&dev_sgp40, SGP40_CMD_SOFT_RESET, NULL, 0, SGP40_TIME_SOFT_RESET);
		
			xSemaphoreGive(sema_i2c2);
		}

		if( flag_SHT4x_is_OK == 1 && flag_SGP40_is_OK == 1 )
		{
			// 4. 온습도
			xSemaphoreTake(sema_i2c2, portMAX_DELAY);
				get_SHT4x_cmd_resp(&dev_sht4x, get_meas_cmd(&dev_sht4x), resp, sizeof(resp));
			xSemaphoreGive(sema_i2c2);
	
	    	sht4x_compute_values_shcho(resp, &temperature, &humidity);
			ESP_LOGW("sht4x Sensor", " %.2f °C, %.2f %%\n", temperature, humidity);
	
			// 5. 온습도 --> VOC Index
			int32_t voc_index;
			xSemaphoreTake(sema_i2c2, portMAX_DELAY);
				sgp40_measure_voc_shcho(&dev_sgp40, humidity, temperature, &voc_index);
			xSemaphoreGive(sema_i2c2);
	
			ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC index: %3" PRIi32 ", Air is [%s]",
						temperature, humidity, voc_index, voc_index_name(voc_index));
	
			do_rht_voc_report(&dev_sht4x, &dev_sgp40, temperature, humidity, voc_index );
		}


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
	register_CO2_cali();
	register_CO2_autozero();

	register_charge_en();

	register_setid_cmd();
	register_spec_sensor_sensitivity();
	register_spec_sensor_vgas0();

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

int gpio3_set_to_input_from_uart(void) // GPIO_3 --> GPIO_8
{
    gpio_config_t io_conf;

    // detect Is it Wearable : Static은 Pull-up :10K GPIO_38(MIX_A0) / GPIO_39(MUX_A0)
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<3);
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


int ZMOD_Reset_GPIO(int val)
{
    gpio_config_t io_conf;

    // detect Is it Wearable : Static은 Pull-up :10K GPIO_38(MIX_A0) / GPIO_39(MUX_A0)
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<< GPIO_ZMOD_RESET); // 45
    //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//                          0 으로만 읽힌다.
//      io_conf.mode = GPIO_MODE_INPUT; //
    io_conf.mode = GPIO_MODE_OUTPUT; //
    //enable pull-up mode
    io_conf.pull_up_en = 0; // 1 --> 0
    io_conf.pull_down_en = 0; //NULL --> 0
    gpio_config(&io_conf);

	gpio_set_level(GPIO_ZMOD_RESET, val);


    return 1;
}

#define I2C2_DELAY	(10)
void	i2c2_sda_input(void)
{
    gpio_config_t io_conf;
	//---------------------------------------------------------
	// SDA : 16 : INPUT_OUTPUT
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    io_conf.pin_bit_mask = (1ULL << GPIO_I2C2_SDA);
    //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//                          0 으로만 읽힌다.
    io_conf.mode = GPIO_MODE_INPUT;
//      io_conf.mode = direction; //
    //enable pull-up mode
    io_conf.pull_up_en = 0; // 1 --> 0
    io_conf.pull_down_en = 0; //NULL --> 0

    gpio_config(&io_conf);
	//---------------------------------------------------------

}


void I2C2_SCL_HI(void)
{
	gpio_set_level(GPIO_I2C2_SCL, 1);
}

void I2C2_SCL_LO(void)
{
	gpio_set_level(GPIO_I2C2_SCL, 0);
}

void I2C2_SDA_HI(void)
{
//  	i2c2_sda_input();
}

void I2C2_SDA_LO(void)
{
	gpio_set_level(GPIO_I2C2_SDA, 0);
}

int I2C2_SDA_IN(void)
{
	return gpio_get_level(GPIO_I2C2_SDA);
}

void I2C2_Start(void)
{
//  	ESP_LOGW("I2C2_Start", "Start begin");
//  	I2C2_SDA_HI(); ets_delay_us(I2C2_DELAY );
//  	I2C2_SCL_HI(); ets_delay_us(I2C2_DELAY );
//  
//  	I2C2_SDA_LO(); ets_delay_us(I2C2_DELAY );
//  	I2C2_SCL_LO();
//  	ESP_LOGW("I2C2_Start", "Start end");
//  //      ets_delay_us(10 );
//  //
	ESP_LOGW("I2C2_Start", "Start begin");
	I2C2_SDA_HI(); vTaskDelay(1);
//  	I2C2_SCL_HI(); vTaskDelay(1);
//  
//  	I2C2_SDA_LO(); vTaskDelay(1);
//  	I2C2_SCL_LO();
	ESP_LOGW("I2C2_Start", "Start end");
}

void I2C2_Stop(void)
{
	I2C2_SDA_LO(); ets_delay_us(I2C2_DELAY );
	I2C2_SCL_HI(); ets_delay_us(I2C2_DELAY );

	I2C2_SDA_HI(); ets_delay_us(I2C2_DELAY );
}

void I2C2_write_bit(bool bit)
{
	if(bit)
	{
		I2C2_SDA_HI(); 
	}
	else
	{
		I2C2_SDA_LO(); 
	}
	ets_delay_us(I2C2_DELAY );
	I2C2_SCL_HI(); ets_delay_us(I2C2_DELAY );
	I2C2_SCL_LO(); ets_delay_us(I2C2_DELAY );
}

uint8_t I2C2_read_bit(void)
{
	uint8_t bit;

	I2C2_SDA_HI(); ets_delay_us(I2C2_DELAY );
	I2C2_SCL_HI(); ets_delay_us(I2C2_DELAY );
	bit = (I2C2_SDA_IN() ? 1 : 0 );
	I2C2_SCL_LO(); ets_delay_us(I2C2_DELAY );

	return bit;
}

uint8_t I2C2_read_byte(bool ack)
{
	uint8_t byte = 0;

	for( int bit = 0 ; bit < 8 ; ++bit)
	{
		byte = ( byte << 1 ) | I2C2_read_bit() ;
	}

	I2C2_write_bit(!ack);

	return byte;
}

uint8_t I2C2_write_byte(uint8_t byte)
{
	for( int bit = 0 ; bit < 8 ; ++ bit)
	{
		I2C2_write_bit( (byte * 0x80 ) != 0);
		byte <<=1 ;
	}
	bool nack = I2C2_read_bit(); // ack= sda low, nack = sda high

	return (nack ? 1 : 0 ) ;
}

uint8_t I2C2_write(uint8_t *data, size_t n)
{
	size_t cnt = 0;
	for( size_t i = 0 ; i < n ; i++)
	{
		cnt += I2C2_write_byte(data[i]);
	}
	return cnt;
}

uint8_t I2C2_endTransmission(bool sendStop)
{
	if( sendStop )
	{
		I2C2_Stop();
	}
	return i2c2_ack;

}

uint8_t I2C2_requestFrom(uint8_t address, uint8_t *i2c2_data, size_t len, bool stopBit)
{
	int i ;

	I2C2_Start();
	I2C2_write_byte( address<<1 | 1 ) ;
	for( i = 0 ; i < len -1  ; i++)
	{
		i2c2_data[i] = I2C2_read_byte(true);
	}
	i2c2_data[i++] = I2C2_read_byte(false);

	return i;
}

	
uint8_t I2C2_beginTransmission(uint8_t address)
{
	ESP_LOGW("I2C2_beginTransmission", "I2C2_beginTransmission begin");
	I2C2_Start();
	i2c2_ack = I2C2_write_byte(address<<1);
	ESP_LOGW("I2C2_beginTransmission", "I2C2_beginTransmission end");
	return i2c2_ack;
}

//  esp_err_t i2c_master_transmit(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, int xfer_timeout_ms)
//  esp_err_t ret = i2c_master_transmit(i2c_master_dev_handle_t i2c_dev, (uint8_t *)i2c_cmd, 2, I2C_TOOL_TIMEOUT_VALUE_MS);
esp_err_t i2c2_master_transmit_gpio(uint8_t chip_address, uint8_t *write_buffer, size_t write_size)
{
	ESP_LOGW("i2c2_master_transmit_gpio", "i2c2_master_transmit_gpio begin");
	int ret=0;
	ret = I2C2_beginTransmission(chip_address);
	if( ret != 0 )
	{
		ESP_LOGE("i2c2_master_transmit_gpio", "I2C2_beginTransmission(%d) : Error", ret);
		return ret;
	}
	I2C2_write(write_buffer,write_size);
	ESP_LOGW("i2c2_master_transmit_gpio", "i2c2_master_transmit_gpio end");
	return 1;
}


void	i2c2_using_gpio_init(void)
{

    gpio_config_t io_conf_scl;
    gpio_config_t io_conf_sda;

	//---------------------------------------------------------
	// SCL : 15 : OUTPUT
    //interrupt of rising edge
    io_conf_scl.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    io_conf_scl.pin_bit_mask = (1ULL << GPIO_I2C2_SCL);
    //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//                          0 으로만 읽힌다.
    io_conf_scl.mode = GPIO_MODE_OUTPUT; //
//      io_conf.mode = direction; //
    //enable pull-up mode
    io_conf_scl.pull_up_en = 0; // 1 --> 0
    io_conf_scl.pull_down_en = 0; //NULL --> 0

    gpio_config(&io_conf_scl);
	//---------------------------------------------------------

	//---------------------------------------------------------
	// SDA : 16 : INPUT_OUTPUT
    //interrupt of rising edge
    io_conf_sda.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
    //bit mask of the pins, use GPIO4/5 here
    io_conf_sda.pin_bit_mask = (1ULL << GPIO_I2C2_SDA);
    //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//                          0 으로만 읽힌다.
    io_conf_sda.mode = GPIO_MODE_INPUT_OUTPUT; //
//      io_conf.mode = direction; //
    //enable pull-up mode
    io_conf_sda.pull_up_en = 0; // 1 --> 0
    io_conf_sda.pull_down_en = 0; //NULL --> 0

    gpio_config(&io_conf_sda);
	//---------------------------------------------------------

}


//  #define USE_ESP_IDF_LIB_I2C	(1)
void app_main(void)
{

	ESP_ERROR_CHECK(esp_read_mac(my_mac_factory, ESP_MAC_EFUSE_FACTORY ));
	hexdump3("esp_read_mac(ESP_MAC_EFUSE_FACTORY)", my_mac_factory, sizeof(my_mac_factory));

	memset(my_mac_str, 0, sizeof(my_mac_str));
	snprintf(my_mac_str, sizeof(my_mac_str),"%02X%02X%02X_%02X%02X%02X", MAC2STR(my_mac_factory));
	ESP_LOGW("system", "my mac(factory) : %s\n", my_mac_str);



	sema_i2c1 = xSemaphoreCreateBinary();
//  	#if ( USE_ESP_IDF_LIB_I2C == 0 ) 
	sema_i2c2 = xSemaphoreCreateBinary();
//  	#endif
	sema_uart1 = xSemaphoreCreateBinary();
	sema_uart2 = xSemaphoreCreateBinary();
	sema_tcp = xSemaphoreCreateBinary();
	sema_spi_ads114s = xSemaphoreCreateBinary();
	sema_ble_send_noti = xSemaphoreCreateBinary();

	xSemaphoreGive(sema_i2c1);
//  	#if ( USE_ESP_IDF_LIB_I2C == 0 ) 
	xSemaphoreGive(sema_i2c2);
//  	#endif
	xSemaphoreGive(sema_uart1);
	xSemaphoreGive(sema_uart2);
	xSemaphoreGive(sema_tcp);
	xSemaphoreGive(sema_spi_ads114s);
	xSemaphoreGive(sema_ble_send_noti);

//  	// 0. ---- LED ctrl
//      xTaskCreate(app_main_led_strip_ctrl, "led_strip_ctrl", 4 * 1024, NULL, 5, NULL);

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

//  		// 0. ---- LED ctrl
//  	    xTaskCreate(app_main_led_strip_ctrl, "led_strip_ctrl", 4 * 1024, NULL, 5, NULL);
//
//  	    여기는 너무빠름
//  		Battery Power On시에 OLED에 표시가 없음
//  		// -------------------------------------------------------------
//  		// I2C를 사용하고 완전히 삭제한다.
//  		app_main_task_oled(NULL);
//  //  		// ========= i2c 이후로 이동해야 하고, OLED가 계속 i2c를 붙잡고 있어서 Release할 수 있도록 해야 함.===
//  //  	    xTaskCreate(app_main_task_oled, "oled", 4 * 1024, NULL, 5, NULL);
//  		// -------------------------------------------------------------


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
	


	    	char listen_port_str[100];
			int listen_port = 0 ;
			memset(listen_port_str, 0, sizeof(listen_port_str));
			ijoon_get_nvs_str((uint8_t*)"listenoort", (uint8_t*)listen_port_str);
			if( listen_port_str[0] == 0 )
			{
				ESP_LOGW("nvs_relate", "set_nvs_str listenport 33333 (example)");
				ESP_LOGW("nvs_relate", "set_nvs_str listenport 33333 (example)");
				ESP_LOGW("nvs_relate", "set_nvs_str listenport 33333 (example)");
				listen_port  = CONFIG_EXAMPLE_SERVER_PORT;
			}
			else
			{
				listen_port  = atoi(listen_port_str);
			}
	
			app_main_tcp_server(listen_port);
		}

    }
    else
    {
        flag_IS_WEARABLE = 0 ;
        ESP_LOGW("shcho", "This Board is Static(%d): : UART_MUX(UART1) / CM4 Communication(UART2)", flag_IS_WEARABLE);
        Uart_mux_setup(GPIO_MODE_OUTPUT);
    }
	//--------------------------------------------------------------
	char tmp_buf[10];
	memset(tmp_buf, 0, sizeof(tmp_buf));
	ijoon_get_nvs_str((uint8_t*)"autozero-close", (uint8_t*)tmp_buf);
	if( tmp_buf[0] == 0 )
	{
		ESP_LOGW("nvs_relate", "set_nvs_str autozero-close 1 : autozero를 하지 못하게 주기적으로 close함");
		ESP_LOGW("nvs_relate", "                             : 이름이 길어서 저장되자 않기 때문애 줄임");
		ESP_LOGW("nvs_relate", "set_nvs_str autozero-close 0 : autozero를 close packet를 보내지 않음");
		ESP_LOGW("nvs_relate", "                             : 이름이 길어서 저장되자 않기 때문애 줄임");
		flag_CO2_autozero_close_run = 0;
	}
	else
	{
		flag_CO2_autozero_close_run = atoi(tmp_buf);
		ESP_LOGW("nvs_relate", "flag_CO2_autozero_close_run=%d", flag_CO2_autozero_close_run);
		ESP_LOGW("nvs_relate", "flag_CO2_autozero_close_run=%d", flag_CO2_autozero_close_run);
		ESP_LOGW("nvs_relate", "flag_CO2_autozero_close_run=%d", flag_CO2_autozero_close_run);
		ESP_LOGW("nvs_relate", "flag_CO2_autozero_close_run=%d", flag_CO2_autozero_close_run);
	}
	//--------------------------------------------------------------
	
	//--------------------------------------------------------------	
	get_DeviceName_from_NVS( DeviceID ) ;

    if ( flag_IS_WEARABLE == 1 )
	{

	    passkey_msg_handle = xMessageBufferCreate( passkey_msg_bytes );
	    assert(passkey_msg_handle);

		//여기서 Delay가 있어야 OLED가 동작한다. : 5V 가 늦게 On되나???
       	vTaskDelay(2000 / portTICK_PERIOD_MS);
		// -------------------------------------------------------------
		// I2C를 사용하고 완전히 삭제한다.
		app_main_task_oled(NULL);
//  		// ========= i2c 이후로 이동해야 하고, OLED가 계속 i2c를 붙잡고 있어서 Release할 수 있도록 해야 함.===
//  	    xTaskCreate(app_main_task_oled, "oled", 4 * 1024, NULL, 5, NULL);
		// -------------------------------------------------------------
       	vTaskDelay(2000 / portTICK_PERIOD_MS);

		app_main_nimble_sec(); // Static / Wearable모두에서 실행
	}
	else
	{
		//--------------------------------------------------------------
		//
		passkey_msg_handle = xMessageBufferCreate( passkey_msg_bytes );
		assert(passkey_msg_handle);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	
		app_main_nimble_sec();
	}

	if( flag_IS_WEARABLE == 0 ) //Static Main
	{
		//CM4에 신고하기 위해서 가장 먼저 Enable되어야 한다.
		app_main_stella_uart2(); // send to CM4
	}
	else
	{
		app_main_stella_uart2_GPS(); 
	}

//  	if( flag_USE_W5500_Ethernet == 1 ) 
//  	{
		app_main_stella_uart1(); // get sensor data // using mux_ctrl // thread for RS9A / and ZE08
//  	}


    i2c_master_bus_config_t i2c_bus_config_i2c1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port_i2c1,
        .scl_io_num = 6 , //i2c_gpio_scl,
        .sda_io_num = 7,  //i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

	#if I2C2__USING_GPIO
//  	// I2C SCL(GPIO_OUTPUT: 15) / SDA(GPIO_INPUT_OUTPUT: 16)
//  	i2c2_using_gpio_init();
//  //  	uint8_t tmp_data[2];
//  //  	tmp_data[0] = 0xAA ;
//  //  	tmp_data[0] = 0x55 ;
//  	while(1)
//  	{
//  //  		i2c2_master_transmit_gpio(SHT4X_I2C_ADDRESS, tmp_data, sizeof(tmp_data));
//  		I2C2_Start();
//          vTaskDelay(1000 / portTICK_PERIOD_MS);
//  	}
	#else
    i2c_master_bus_config_t i2c_bus_config_i2c2 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port_i2c2,
        .scl_io_num = 15 , //i2c_gpio_scl,
        .sda_io_num = 16,  //i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
	#endif

#if 1 // I2C2__USE_GPIO_TEST
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c1, &tool_bus_handle_i2c1));

//      xTaskCreate(i2c1_sensor_task, "i2c1_sensor", 4 * 1024, NULL, 8, NULL);
    xTaskCreate(i2c1_i2c2_sensor_task, "i2c1_sensor", 8 * 1024, NULL, 8, NULL);

	#if I2C2__USING_GPIO
	#else
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config_i2c2, &tool_bus_handle_i2c2));
	#endif

//  	#if I2C2__USING_GPIO
//      xTaskCreate(i2c2_sensor_task, "i2c2_sensor", 4 * 1024, NULL, 8, NULL); // Priority가 높다(너무 높아서 다른  Task가 동작하지 못했나 보다
//  	#else
//  //  	// i2c2_sensor_task를 실행하면 i2s_dpm Buffer가 고정된값으로만 읽힌다.
//          // UART2도 동작하지 않나. GPIO3 --> GPIO8로 변경하면 동작하는데(GPIO3은 Input으로 하고, Jumper연결)
//      xTaskCreate(i2c2_sensor_task, "i2c2_sensor", 4 * 1024, NULL, 8, NULL); // Priority가 높다(너무 높아서 다른  Task가 동작하지 못했나 보다
//  	#endif

//  	------------------------------------------------------------------------
//  	i2c2_sensor_task를 실행하면 I2S read buffer에 같은 값만 찍힌다.

	printf("I2S PDM RX example start\n---------------------------\n");
	xTaskCreate(i2s_example_pdm_rx_task, "pdm_rx", 4096+2048, NULL, 1, NULL); // + 2048
//  //  	TaskHandle_t pdm_rx_task;
//  //  	xTaskCreatePinnedToCore(i2s_example_pdm_rx_task, "pdm_rx", 4096+2048, NULL, 1, &pdm_rx_task, 1);
//  //  	------------------------------------------------------------------------


//  	------------------------------------------------------------------------
	if( flag_USE_W5500_Ethernet == 0 ) 
	{
	    xTaskCreate(spi2_adc_task, "adc_task", 4 * 1024, NULL, 8, NULL);
	}
//  	------------------------------------------------------------------------

//  //  아래로 이동시켜봄
//  //  	// i2c2_sensor_task를 실행하면 i2s_dpm Buffer가 고정된값으로만 읽힌다.
//          // UART2도 동작하지 않나. GPIO3 --> GPIO8로 변경하면 동작하는데(GPIO3은 Input으로 하고, Jumper연결)
//      xTaskCreate(i2c2_sensor_task, "i2c2_sensor", 4 * 1024, NULL, 1, NULL);
//  //  //  



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

//  	#if ( USE_ESP_IDF_LIB_I2C == 0 ) 
    register_i2ctools();
//  	#endif
	register_stella_cmd();

	xSemaphoreTake(sema_i2c1, portMAX_DELAY);
	set_fan_pwm(); //주기적으로 하자 ? 너무 빨리하면 Power On시에 FAN Controller가 나중에 Access되는 경우가 있다.
	xSemaphoreGive(sema_i2c1);

//  //  	#if ( USE_ESP_IDF_LIB_I2C == 1 ) 
//      ESP_ERROR_CHECK(i2cdev_init_stella_i2c2_only()); // old driver  conflict
//  //      E (1066) i2c: CONFLICT! driver_ng is not allowed to be used with this old driver
//      xTaskCreate(task_sgp40, "sgp40", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL); // old driver  conflict
//  //  	#endif


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
#endif // I2C__USE_GPIO_TEST

}
