idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    set(requires esp_event esp_stubs protocol_examples_common nvs_flash)
endif()

idf_component_register(SRCS "stella_main.c"
                            "stella_uart.c"
                            "cmd_i2ctools.c"
							"hexdump3.c"
							"led_strip_ctrl.c"
							"led_strip_encoder.c"
							"tcp_client_v4.c"
                       INCLUDE_DIRS "."
					   REQUIRES ${requires})

