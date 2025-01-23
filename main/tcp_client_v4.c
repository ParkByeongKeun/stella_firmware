/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "sdkconfig.h"
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>            // struct addrinfo
#include <arpa/inet.h>
#include "esp_netif.h"
#include "esp_log.h"
#if defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
#include "addr_from_stdin.h"
#endif
//shcho add
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stella_global.h"

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
static const char *payload = "(from ESP32)Type Message to ESP32 :";
extern esp_err_t ijoon_get_nvs_str(uint8_t *key, uint8_t *value);

extern int flag_USE_W5500_Ethernet;

//  int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
int recv_using_select( int sock, char *rx_buffer, int len )
{
    int s;
    fd_set rfds;
    struct timeval tv = {
        .tv_sec = 5,
        .tv_usec = 0,
    };
	int ret = -1;

    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);

    s = select(sock + 1, &rfds, NULL, NULL, &tv);

    if (s < 0) 
	{
        ESP_LOGE("recv_using_select", "Select failed: errno %d", errno);
		ret = -1;
    } 
	else if (s == 0) 
	{
        ESP_LOGI("recv_using_select", "Timeout has been reached and nothing has been received");
		ret = 1;
    } 
	else 
	{
	    if (FD_ISSET(sock, &rfds)) 
		{
			int len_read = recv(sock, rx_buffer, len-1, 0);
            if (len_read > 0)
			{
				ret = len_read;
			} 
			else 
			{
	            ESP_LOGE(TAG, "UART read error");
	            ret = -1;
            }
		}
	}

	return ret;
}

//  void tcp_client(void)
//  #error 111111111111111111111111111111111111111
int send_to_server(char *payload, int len)
{
	int ret = -1 ;

	if ( flag_USE_W5500_Ethernet == 1 )
	{
	    char rx_buffer[1024];
	
	//      char host_ip[] = HOST_IP_ADDR;
	    char host_ip[100];
		uint16_t port;
	
	    int addr_family = 0;
	;    int ip_protocol = 0;
	
		//============================================================
		memset(rx_buffer, 0, sizeof(rx_buffer));
		ijoon_get_nvs_str((uint8_t*)"serverport", (uint8_t*)rx_buffer);
		if( rx_buffer[0] == 0 )
		{
			ESP_LOGW("nvs_relate", "set_nvs_str serverport 33333 (example)");
			ESP_LOGW("nvs_relate", "set_nvs_str serverport 33333 (example)");
			ESP_LOGW("nvs_relate", "set_nvs_str serverport 33333 (example)");
			port = PORT;
		}
		else
		{
			port = atoi(rx_buffer);
		}
		//----------------------------------------------------------------------
		memset(host_ip, 0, sizeof(host_ip));
		ijoon_get_nvs_str((uint8_t*)"serverip", (uint8_t*)host_ip);
		if( host_ip[0] == 0 )
		{
			ESP_LOGW("nvs_relate", "set_nvs_str serverip 192.68.10.111 (example)");
			ESP_LOGW("nvs_relate", "set_nvs_str serverip 192.68.10.111 (example)");
			ESP_LOGW("nvs_relate", "set_nvs_str serverip 192.68.10.111 (example)");
			strcpy(host_ip, HOST_IP_ADDR);
		}
		//============================================================
		
	//      while (1) 
		{
	        struct sockaddr_in dest_addr;
	        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
	        dest_addr.sin_family = AF_INET;
	        dest_addr.sin_port = htons(port);
	        addr_family = AF_INET;
	        ip_protocol = IPPROTO_IP;
			ESP_LOGW("shcho", "1111111111111111111111111111111111");
	
	        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
	        if (sock < 0) {
	            ESP_LOGE("send_to_server", "Unable to create socket: errno %d", errno);
	            ret = -1 ;
	        }
//  	        ESP_LOGI("send_to_server", "Socket created, connecting to %s:%d", host_ip, PORT);
	        ESP_LOGI("send_to_server", "Socket created, connecting to %s:%d", host_ip, port);
	
	        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
	        if (err != 0) {
	            ESP_LOGE("send_to_server", "Socket unable to connect: errno %d", errno);
	            ret = err;
	        }
	        ESP_LOGI("send_to_server", "Successfully connected");
	
	//          while (1) 
			{
	//              int err = send(sock, payload, strlen(payload), 0);
	            int err = send(sock, payload, len, 0);
	            if (err < 0) {
	                ESP_LOGE("send_to_server", "Error occurred during sending: errno %d", errno);
	                ret = err;
	            }
	
	            int len_read = recv_using_select(sock, rx_buffer, sizeof(rx_buffer));
	            // Error occurred during receiving
	            if (len_read < 0) {
	                ESP_LOGE("send_to_server", "recv failed: errno %d", errno);
	                ret = len_read;
	            }
	            // Data received
	            else if (len_read < 0) 
				{
	                ESP_LOGE("send_to_server", "recv timeout : errno %d", errno);
	                ret = len_read;
				}
	            else {
	                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
	                ESP_LOGI("send_to_server", "Received %d bytes from %s:", len, host_ip);
	                ESP_LOGI("send_to_server", "%s", rx_buffer);
	            }
	        }
	
	        if (sock != -1) {
	            ESP_LOGW("send_to_server", "Shutting down socket");
	            shutdown(sock, 0);
	            close(sock);
	        }
	    }
	
	}
	else // 이제는 BT로 보낼까? // 자체 DB에 저장????
	{
		// test : just return
		ret = 0 ;
	}

	return ret;
	
}

//  void tcp_client(void)
void tcp_client_task(void* arg)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;

	    dest_addr.sin_port = htons(PORT);


        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
		ESP_LOGW("shcho", "1111111111111111111111111111111111");
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1) {
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }

	vTaskDelete(NULL);
}
