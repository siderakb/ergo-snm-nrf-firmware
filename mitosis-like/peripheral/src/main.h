/**
 * @file  main.h
 * @brief ErgoSNM keyboard wireless mitosis-like edition firmware, peripheral.
 * @author SideraKB / ZiTe (honmonoh@gmail.com)
 * @note SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h> /* PMW3360. */
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <nrf_gzll.h>  /* Gazell. */
#include <gzll_glue.h> /* Gazell. */
#include <inttypes.h>
#include <string.h>
#include <stdio.h>

bool keymatrix_init(void);
uint8_t keymatrix_read_cols(uint8_t row);
void keymatrix_scan(uint8_t *matrix);

bool gzll_init(void);
void gzll_work_callback(struct k_work *work);

bool pmw3360_init(void);
int pmw3360_read(int16_t *x, int16_t *y);
void pmw3360_callback(const struct device *dev, const struct sensor_trigger *trig);
