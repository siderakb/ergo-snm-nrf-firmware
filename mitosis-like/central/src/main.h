/**
 * @file  main.h
 * @brief ErgoSNM keyboard wireless mitosis-like edition firmware, central.
 * @author SideraKB / ZiTe (honmonoh@gmail.com)
 * @note SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
// #include <zephyr/drivers/sensor.h> /* PMW3360. */
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <nrf_gzll.h>  /* Gazell. */
#include <gzll_glue.h> /* Gazell. */
#include <inttypes.h>
#include <string.h>
#include <stdio.h>

bool qmk_uart_init(void);
void qmk_uart_callback(const struct device *dev, void *user_data);
void qmk_uart_send_bytes(uint8_t *buf, uint16_t len);
void qmk_uart_send(uint8_t *keymatrix, uint8_t *mouse);

bool gzll_init(void);
void gzll_work_callback(struct k_work *work);
