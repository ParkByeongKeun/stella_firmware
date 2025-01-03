/* UART Select Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/fcntl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/select.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart_vfs.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "freertos/semphr.h"
#include <cJSON.h>

static const char* TAG = "uart_select_example";
extern SemaphoreHandle_t sema_uart1 ;
extern SemaphoreHandle_t sema_uart2 ;
extern char my_mac_str[32];

#define GPIO_MUX_SEL_A0    39
#define GPIO_MUX_SEL_A1    38
#define GPIO_OUTPUT_MUX_SEL  ((1ULL<<GPIO_MUX_SEL_A0) | (1ULL<<GPIO_MUX_SEL_A1))

#define MUX_SEL_ZE08	(0x08)
#define MUX_SEL_RS9A	(0x09)

#define STR_MATCH		(0)

#define  STELLA_STATIC_OR_WEARABLE

#define GPIO_MUX_A0     38
#define GPIO_MUX_A1     39
#define GPIO_MUX_PIN_SEL  ((1ULL<<GPIO_MUX_A0) | (1ULL<<GPIO_MUX_A1))

extern int flag_IS_WEARABLE; 
extern void hexdump3(char *title, void *pack, size_t size) ;
extern int send_to_server(char *payload, int len);
int fd_uart2 = -1 ;
char buf_uart2[1024];

void uart_mux_select(int kind)
{
	switch(kind)
	{
		case MUX_SEL_ZE08:
	    	gpio_set_level(GPIO_MUX_SEL_A1, 0);
	    	gpio_set_level(GPIO_MUX_SEL_A0, 0);
			break;			
		case MUX_SEL_RS9A:
	    	gpio_set_level(GPIO_MUX_SEL_A1, 0);
	    	gpio_set_level(GPIO_MUX_SEL_A0, 1);
			break;			
		default:
			ESP_LOGE("shcho_test", "mux_select : only support 0x08(ZE08) / 0x09 ( RS9A)");
	}
}


#define LEN_BUF_STR_RS9A	(100)

char buf_str0[LEN_BUF_STR_RS9A];
char buf_str1[LEN_BUF_STR_RS9A];
char buf_str2[LEN_BUF_STR_RS9A];


//             $ ./a.out 'a/bbb///cc;xxx:yyy:' ':;' '/'
//             1: a/bbb///cc
//                      --> a
//                      --> bbb
//                      --> cc
//             2: xxx
//                      --> xxx
//             3: yyy
//                      --> yyy

// STATUS NORMAL:VALUE 1.1:ROU 0.9:rTime 5:UNIT 0
// VERSION V0.9.7
// SERIAL_No IB07AA001424
// ===========================================
//  1: VERSION V0.9.7
//   --> VERSION  		// flag :0
//   --> V0.9.7
//  ===========================================
//  1: SERIAL_No IB07AA001424
//   --> SERIAL_No  	// flag :1
//   --> IB07AA001424
//  ===========================================
//  1: STATUS NORMA1
//   --> STATUS 		// flag : 2
//   --> NORMAL
//  2: VALUE 1.5
//   --> VALUE /		/ flag : 3
//   --> 1.5
//  3: ROU 0.9
//   --> ROU 			// flag : 4
//   --> 0.9
//  4: rTime 1
//   --> rTime 			// flag : 5
//   --> 1
//  5: UNIT 0
//   --> UNIT 			// flag : 6
//   --> 0

struct _RS9A_format
{
	char RS9A_SW_ver[30];
	char RS9A_Serial_num[30];
	char RS9A_Status[10];
	char RS9A_Val[10];
	char RS9A_ROU[10];
	char RS9A_rTime[10];
	char RS9A_Unit[10];
}__attribute__((packed));

struct _RS9A_format RS9A_format;
int flag_RS9A_data_valid = 0  ;

int extract_info_RS9A_send(char *ver_str, char *sn_str, char *value_str)
{
	char* str1 = (char*)0;
	char* str2 = (char*)0;
	char* token = (char*)0;
	char* subtoken = (char*)0;
	char* saveptr1 = (char*)0;
	char* saveptr2 = (char*)0;

	int flag_item = -1;

	// 0 : ver_str ================================================
	// 1 : sn_str ================================================
	// 2 : value_str ================================================
	
	for( int i = 0 ; i < 3 ; i++ )
	{
		switch(i)
		{
			case 0 :
				str1 = (char *)ver_str;
				break;
			case 1 :
				str1 = (char *)sn_str;
				break;
			case 2 :
				str1 = (char *)value_str;
				break;
		}
		printf("===========================================\n");
//  		for( int j = 1 , str1 = (char*)ver_str ; ; j++, str1 = NULL)
		for( int j = 1 ; ; j++, str1 = NULL)
		{
			token = strtok_r( (char*)str1, ":\r\n", &saveptr1);
			if( token == NULL ) 
			{
				break;
			}
			printf("%d: %s\n", j, token);
	
			for( str2 = token; ; str2 = (char *)NULL )
			{
				subtoken = strtok_r( str2, " ", &saveptr2);
				if( subtoken == NULL)
				{
					break;
				}
				printf(" --> %s(flag_item=%d)\n", subtoken, flag_item);

			//---------------------------------------------------------------------------------------------------
				switch(flag_item)
				{
					case 0 : // RS9A_SW_Ver
//  						printf("				0000000000000000000000000 %s\n", subtoken);
						memset((char *)&RS9A_format, 0, sizeof(RS9A_format));
						sprintf(RS9A_format.RS9A_SW_ver, subtoken);
						flag_item=-1; //반드시
						break;
					case 1 : // RS9A_Serial_num
//  						printf("				11111111111111111111111111 %s\n", subtoken);
						sprintf(RS9A_format.RS9A_Serial_num, subtoken);
						flag_item=-1; //반드시
						break;
					case 2 : // RS9A_Status
//  						printf("				22222222222222222222222222 %s\n", subtoken);
						sprintf(RS9A_format.RS9A_Status, subtoken);
						if( strncasecmp( RS9A_format.RS9A_Status, "NORMAL", strlen("NORMAL")) == STR_MATCH )
						{
							flag_RS9A_data_valid = 1  ;
						}
						flag_item=-1; //반드시
						break;
					case 3 : // RS9A_Value
//  						printf("				33333333333333333333333333 %s\n", subtoken);
						sprintf(RS9A_format.RS9A_Val, subtoken);
						flag_item=-1; //반드시
						break;
					case 4 : // RS9A_ROU
//  						printf("				444444444444444444444444444 %s\n", subtoken);
						sprintf(RS9A_format.RS9A_ROU, subtoken);
						flag_item=-1; //반드시
						break;
					case 5 : // RS9A_rTime
//  						printf("				55555555555555555555555555 %s\n", subtoken);
						sprintf(RS9A_format.RS9A_rTime, subtoken);
						flag_item=-1; //반드시
						break;
					case 6 : // RS9A_UNIT
//  						printf("				66666666666666666666666666 %s\n", subtoken);
						sprintf(RS9A_format.RS9A_Unit, subtoken);
						flag_item = -1; //반드시
						hexdump3("RS9A_format", (char*)&RS9A_format, sizeof(RS9A_format));
						break;
				}
				// 위 Code가 항상 앞에 있고 아래Code가 항상 아래에 있어야 함,>>
					
				     if( strncmp(subtoken, "VERSION"  , strlen("VERSION" )  ) == STR_MATCH ) { flag_item = 0 ; }
				else if( strncmp(subtoken, "SERIAL_No", strlen("SERIAL_No") ) == STR_MATCH ) { flag_item = 1 ; }
				else if( strncmp(subtoken, "STATUS"   , strlen("STATUS" )   ) == STR_MATCH ) { flag_item = 2 ; }
				else if( strncmp(subtoken, "VALUE"    , strlen("VALUE" )    ) == STR_MATCH ) { flag_item = 3 ; }
				else if( strncmp(subtoken, "ROU"      , strlen("ROU" )      ) == STR_MATCH ) { flag_item = 4 ; }
				else if( strncmp(subtoken, "rTime"     , strlen("UNIT" )    ) == STR_MATCH ) { flag_item = 5 ; }
				else if( strncmp(subtoken, "UNIT"     , strlen("UNIT" )     ) == STR_MATCH ) { flag_item = 6 ; }
//  				printf(" --> %d(flag_item)\n", flag_item);
			//---------------------------------------------------------------------------------------------------
			}
	
		}
	}

	cJSON *root;
    root = cJSON_CreateObject();
	char *my_json_string ;

	ESP_LOGW("test", "RS9A_Status=%s", RS9A_format.RS9A_Status ) ;
	if( strncasecmp(RS9A_format.RS9A_Status, "NORMAL", strlen("NORMAL")) == STR_MATCH ) 
	{

		ESP_LOGI("RS9A", "Serialize.....RS9A");
    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
	    cJSON_AddStringToObject(root, "RS9A_SW_ver",       RS9A_format.RS9A_SW_ver );
	    cJSON_AddStringToObject(root, "RS9A_Serial_num",   RS9A_format.RS9A_Serial_num );
	    cJSON_AddStringToObject(root, "RS9A_Status",       RS9A_format.RS9A_Status );

	    cJSON_AddNumberToObject(root, "RS9A_Val",          atof(RS9A_format.RS9A_Val) );
	    cJSON_AddNumberToObject(root, "RS9A_ROU",          atof(RS9A_format.RS9A_ROU ) );
	    cJSON_AddNumberToObject(root, "RS9A_rTime",        atoi(RS9A_format.RS9A_rTime ) );
	    cJSON_AddNumberToObject(root, "RS9A_UNIT",         atoi(RS9A_format.RS9A_Unit ) );
	
		my_json_string = cJSON_Print(root);
	
	}
	else
	{
    	cJSON_AddStringToObject(root, "Board_Serial_Num",my_mac_str);
	    cJSON_AddStringToObject(root, "RS9A_is_not_normal",  "Not Normal" );
	    ESP_LOGW("RS9A", "status is not NORMAL ( no need to send to CM4 )");
		my_json_string = cJSON_Print(root);
	}
	ESP_LOGI("RS9A", "my_json_string\n%s",my_json_string);

	if( strncasecmp(RS9A_format.RS9A_Status, "NORMAL", strlen("NORMAL") ) == STR_MATCH ) 
	{
		
		if( flag_IS_WEARABLE == 0 ) //Static Main
		{
			xSemaphoreTake(sema_uart2, portMAX_DELAY);
			write(fd_uart2, my_json_string, strlen(my_json_string));
			xSemaphoreGive(sema_uart2);
		}
		else // Wearable Main
		{
			xSemaphoreTake(sema_uart2, portMAX_DELAY);
			send_to_server(my_json_string, strlen(my_json_string));
			xSemaphoreGive(sema_uart2);
		}
	}

    cJSON_Delete(root);

	return 1;

//  	
//  	// 1 : sn_str ================================================
//  	for( int j = 1 , str1 = (char*)sn_str ; ; j++, str1 = (char *)NULL)
//  	{
//  		token = strtok_r( (char*)str1, ":\r\n", &saveptr1);
//  		if( token == NULL ) 
//  		{
//  			break;
//  		}
//  		printf("%d: %s\n", j, token);
//  
//  		for( str2 = token; ; str2 = (char *)NULL )
//  		{
//  			subtoken = strtok_r( str2, " ", &saveptr2);
//  			if( subtoken == NULL)
//  			{
//  				break;
//  			}
//  			printf(" --> %s\n", subtoken);
//  		}
//  
//  	}
//  
//  	// 2 : value_str ================================================
//  	for( int j = 1 , str1 = (char*)value_str ; ; j++, str1 = (char *)NULL)
//  	{
//  		token = strtok_r( (char*)str1, ":\r\n", &saveptr1);
//  		if( token == NULL ) 
//  		{
//  			break;
//  		}
//  		printf("%d: %s\n", j, token);
//  
//  		for( str2 = token; ; str2 = (char *)NULL )
//  		{
//  			subtoken = strtok_r( str2, " ", &saveptr2);
//  			if( subtoken == NULL)
//  			{
//  				break;
//  			}
//  			printf(" --> %s\n", subtoken);
//  		}
//  
//  	}
//
//  	return 1;
}

static void uart_select_task_uart1(void *arg)
{

    if (uart_driver_install(UART_NUM_1, 2 * 1024, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Driver installation failed");
        vTaskDelete(NULL);
    }

    uart_config_t uart_config = {
//          .baud_rate = 115200,
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//          .source_clk = UART_SCLK_DEFAULT,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, 17, 18, -1, -1);   // NUM_1 for Sensor

	int loop_count_rs9a = 0 ; 
	char *Query_str_RS9A[] ={ "VERSION?\r\n", "SERIALNO?\r\n", "VALUE?\r\n" };
    char *buf_str = NULL;

    while (1) 
	{
        int fd_uart1;

        ESP_LOGE(TAG, "before xSemaphoreTake(sema_uart1, portMAX_DELAY)");
		xSemaphoreTake(sema_uart1, portMAX_DELAY);
        if ((fd_uart1 = open("/dev/uart/1", O_RDWR)) == -1) {
            ESP_LOGE(TAG, "Cannot open UART1");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        // We have a driver now installed so set up the read/write functions to use driver also.
        uart_vfs_dev_use_driver(1);


		loop_count_rs9a %= 3 ; 
		ESP_LOGW("shcho_test", "\n>>>uart1 write: %s", Query_str_RS9A[loop_count_rs9a]);
	    write(fd_uart1, Query_str_RS9A[loop_count_rs9a], strlen(Query_str_RS9A[loop_count_rs9a]));

//  		RS9A일 때는 아래와 같이 // 아닐때는 어떻게 모두 한번에 받지????
//  		esp_vfs_dev_uart_port_set_[tx, rx]_line_endings(1, ESP_LINE_ENDINGS_LF)?
//  		esp_vfs_dev_uart_port_set_tx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.2.1
//  		esp_vfs_dev_uart_port_set_rx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.2.1
		uart_vfs_dev_port_set_tx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.3.2
		uart_vfs_dev_port_set_rx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.3.2

        while (1) 
		{
            int s;
            fd_set rfds;
            struct timeval tv = {
                .tv_sec = 5,
                .tv_usec = 0,
            };

            FD_ZERO(&rfds);
            FD_SET(fd_uart1, &rfds);

            s = select(fd_uart1 + 1, &rfds, NULL, NULL, &tv);

            if (s < 0) {
                ESP_LOGE(TAG, "Select failed: errno %d", errno);
                break;
            } else if (s == 0) {
                ESP_LOGI(TAG, "Timeout has been reached and nothing has been received");
				break;
            } else {
                if (FD_ISSET(fd_uart1, &rfds)) 
				{
					switch( loop_count_rs9a %3 )
					{
						case 0 : // VERSION?
							buf_str = buf_str0 ;
							break;
						case 1 : // SERIALNO?
							buf_str = buf_str1 ;
							break;
						case 2 : // VALUE?
							buf_str = buf_str2 ;
							break;
					} 
					int len_read = 0;
					memset(buf_str, 0, LEN_BUF_STR_RS9A);
                    len_read = read(fd_uart1, buf_str, LEN_BUF_STR_RS9A-1 ) ;
					ESP_LOGW("shcho", "len_read=%d :  from RS9A", len_read);
                    if (len_read > 0)
					{
                        ESP_LOGI(TAG, "Received: %s", buf_str);
                        // Note: Only one character was read even the buffer contains more. The other characters will
                        // be read one-by-one by subsequent calls to select() which will then return immediately
                        // without timeout.
						hexdump3("RS9A reply", buf_str, len_read);	
						if( loop_count_rs9a %3  == 2)
						{
							extract_info_RS9A_send(buf_str0, buf_str1, buf_str2);
							if (flag_RS9A_data_valid == 1 )
							{
        						if( flag_IS_WEARABLE == 0 )  // Static Main
								{
//  									send_to_CM4_RS9A();
								};
							}
						} 
						break;
                    } else {
                        ESP_LOGE(TAG, "UART read error");
                        break;
                    }
                } else {
                    ESP_LOGE(TAG, "No FD has been set in select()");
                    break;
                }
            }
        }
		loop_count_rs9a ++ ; 

        close(fd_uart1);

        ESP_LOGE(TAG, "before xSemaphoreGive(sema_uart1)");
		xSemaphoreGive(sema_uart1);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void uart_select_task_uart2(void *arg) // receive 만 한다.
{
	
    while (1) 
	{
        int s;
        fd_set rfds;
        struct timeval tv = {
            .tv_sec = 5,
            .tv_usec = 0,
        };

        FD_ZERO(&rfds);
        FD_SET(fd_uart2, &rfds);

        s = select(fd_uart2 + 1, &rfds, NULL, NULL, &tv);

        if (s < 0) {
            ESP_LOGE("uart2", "Select failed: errno %d", errno);
            continue;
        } else if (s == 0) {
            ESP_LOGI("uart2", "Timeout has been reached and nothing has been received");
			continue;
        } else {
            if (FD_ISSET(fd_uart2, &rfds)) 
			{
				int len_read = 0;
				memset(buf_uart2, 0, sizeof(buf_uart2));
                len_read = read(fd_uart2, buf_uart2, sizeof(buf_uart2)-1 ) ;
				ESP_LOGW("uart2", "len_read=%d :  from CM4", len_read);
                if (len_read > 0)
				{
                    ESP_LOGI("uart2", "Received: %s", buf_uart2);
					hexdump3("RS9A reply", buf_uart2, len_read);	
                } else {
                    ESP_LOGE("uart2", "UART2 read error");
                    continue;
                }
            } else {
                ESP_LOGE("uart2", "No FD has been set in select()");
                continue;
            }
        }
    }
    ESP_LOGE("uart2", "before vTaskDelete(NULL) : uart_select_task_uart2");

    vTaskDelete(NULL);
}
//  void app_main_stella_uart(void)
void app_main_stella_uart1(void)
{
//  	gpio_config_t io_conf;
//  
//  	// detect Is it Wearable : Static은 Pull-up :10K GPIO_38(MIX_A0) / GPIO_39(MUX_A0)
//      //interrupt of rising edge
//      io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_POSEDGE -->GPIO_INTR_DISABLE
//      //bit mask of the pins, use GPIO4/5 here
//      io_conf.pin_bit_mask = GPIO_MUX_PIN_SEL;
//      //set as input mode
//      io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // GPIO_MODE_INPUT --> GPIO_MODE_INPUT_OUTPUT
//      //enable pull-up mode
//      io_conf.pull_up_en = 0; // 1 --> 0 
//      io_conf.pull_down_en = 0; //NULL --> 0 
//      gpio_config(&io_conf);
//  
//  	int val_mux_A0 = gpio_get_level(GPIO_MUX_A0);
//  	int val_mux_A1 = gpio_get_level(GPIO_MUX_A1);
//  
//  	if( val_mux_A0 == 0 && val_mux_A1 == 0 ) 
//  	{
//  		flag_IS_WEARABLE = 1 ; 
//  		ESP_LOGW("shcho", "This Board is Wearable(%d): No UART_MUX(UART1) / No CM4 Communication(UART2)", flag_IS_WEARABLE);
//  	}
//  	else
//  	{
//  		flag_IS_WEARABLE = 0 ; 
//  		ESP_LOGW("shcho", "This Board is Static(%d): : UART_MUX(UART1) / CM4 Communication(UART2)", flag_IS_WEARABLE);
//  	}
//  
	ESP_LOGW("shcho", "uart_mux_select(MUX_SEL_RS9A)");
	uart_mux_select(MUX_SEL_RS9A);

    xTaskCreate(uart_select_task_uart1, "task_uart1", 4 * 1024, NULL, 5, NULL);
}
void app_main_stella_uart2(void)
{
	xSemaphoreTake(sema_uart2, portMAX_DELAY);

    if (uart_driver_install(UART_NUM_2, 2 * 1024, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Driver installation failed");
        vTaskDelete(NULL);
    }

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//          .source_clk = UART_SCLK_DEFAULT,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM_2, &uart_config);

	uart_set_pin(UART_NUM_2, 3, 46, -1, -1);   // NUM_2 for Sensor

	if ((fd_uart2 = open("/dev/uart/2", O_RDWR)) == -1) 
	{
		ESP_LOGE(TAG, "Cannot open UART2");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
	
	// We have a driver now installed so set up the read/write functions to use driver also.
	uart_vfs_dev_use_driver(2);
	
	
//  //  	esp_vfs_dev_uart_port_set_tx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.2.1
//  //  	esp_vfs_dev_uart_port_set_rx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.2.1
//  	uart_vfs_dev_port_set_tx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.3.2
//  	uart_vfs_dev_port_set_rx_line_endings(1, ESP_LINE_ENDINGS_LF); //v5.3.2

	xSemaphoreGive(sema_uart2);

    xTaskCreate(uart_select_task_uart2, "task_uart2", 4 * 1024, NULL, 5, NULL);
	return;
}
