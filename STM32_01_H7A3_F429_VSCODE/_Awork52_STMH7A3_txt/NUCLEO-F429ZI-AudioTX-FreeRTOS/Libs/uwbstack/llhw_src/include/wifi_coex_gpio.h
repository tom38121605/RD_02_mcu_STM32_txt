/*
 * Copyright (c) 2022 Qorvo, Inc
 *
 * All rights reserved.
 *
 * NOTICE: All information contained herein is, and remains the property
 * of Qorvo, Inc. and its suppliers, if any. The intellectual and technical
 * concepts herein are proprietary to Qorvo, Inc. and its suppliers, and
 * may be covered by patents, patent applications, and are protected by
 * trade secret and/or copyright law. Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior written
 * permission is obtained from Qorvo, Inc.
 *
 */

#ifndef _WIFI_COEX_GPIO_H_
#define _WIFI_COEX_GPIO_H_

/**
 * wifi_coex_uwb_wlan_gpio_configure() - Configure UWB_WLAN_GPIO.
 *
 * @coex_hw_assisted: Pointer to a boolean defining if WiFi Coex is handled by the hardware.
 * FIXME: Currently always false.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
int wifi_coex_uwb_wlan_gpio_configure(bool *coex_hw_assisted);

/**
 * wifi_coex_uwb_wlan_gpio_write() - Set level of UWB_WLAN_GPIO.
 */
void wifi_coex_uwb_wlan_gpio_write(bool value);

/**
 * wifi_coex_wlan_uwb_gpio_configure() - Configure WLAN_UWB_GPIO.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
int wifi_coex_wlan_uwb_gpio_configure();

/**
 * wifi_coex_wlan_uwb_gpio_read_state() - Read level of WLAN_UWB_GPIO.
 *
 * Return: true if WLAN_UWB_GPIO is HIGH.
 */
bool wifi_coex_wlan_uwb_gpio_read_state();

/**
 * wifi_coex_wlan_uwb_gpio_read_interrupts() - Read interrupt status that occurred on WLAN_UWB_GPIO.
 *
 * Return: true if an interrupt (Rising or Falling edge) has occurred
 * since the last call to wifi_coex_wlan_uwb_gpio_clear_interrupts().
 */
bool wifi_coex_wlan_uwb_gpio_read_interrupts();

/**
 * wifi_coex_wlan_uwb_gpio_clear_interrupts() - Clear interrupts that occurred on WLAN_UWB_GPIO.
 *
 */
void wifi_coex_wlan_uwb_gpio_clear_interrupts();

#endif /* _WIFI_COEX_GPIO_H_ */
