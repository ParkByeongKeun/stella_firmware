/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "common.h"
#include "heart_rate.h"

/* Private variables */
//  static uint8_t heart_rate;
static int heart_rate;
//shcho
extern int CO2_ppm;

/* Public functions */
//  uint8_t get_heart_rate(void) { return heart_rate; }
//  
//  void update_heart_rate(void) { heart_rate = 60 + (uint8_t)(esp_random() % 21); }
// shcho change
//  uint8_t get_heart_rate(void) { return (CO2_ppm % 256); }
int get_heart_rate(void) { return (CO2_ppm); }

//  void update_heart_rate(void) { heart_rate = (CO2_ppm % 256) ; }
void update_heart_rate(void) { heart_rate = (CO2_ppm ) ; }
