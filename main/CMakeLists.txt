idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    set(requires esp_event esp_stubs protocol_examples_common nvs_flash esp_idf_lib_helpers )
endif()

if(CONFIG_SOC_I2S_SUPPORTS_PDM_TX AND CONFIG_EXAMPLE_PDM_TX)
    list(APPEND srcs_i2s_pdm "i2s_pdm_tx.c")
endif()

if(CONFIG_SOC_I2S_SUPPORTS_PDM_RX AND CONFIG_EXAMPLE_PDM_RX)
    list(APPEND srcs_i2s_pdm "i2s_pdm_rx.c")
endif()


idf_component_register(SRCS "stella_main.c"
                            "stella_uart.c"
                            "cmd_i2ctools.c"
							"hexdump3.c"
							"led_strip_ctrl.c"
							"led_strip_encoder.c"
							"tcp_client_v4.c"
							"oled_ssd1306.c"
							"i2c2_sensors.c"
							"tcp_server.c"
							"spi_ads114S_test.c"
							"./esp32-fft/components/fft/fft.c"
							"${srcs_i2s_pdm}"
                       INCLUDE_DIRS "."
					         "./esp32-fft/components/fft/include"
					   REQUIRES ${requires})

