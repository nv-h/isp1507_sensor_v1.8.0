/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sensor);

#include "sht3x.h"
#include "qmp6988.h"
#include "sgp30.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)

void main(void)
{
	LOG_MODULE_DECLARE(sensor);

	bool led_is_on = true;
	uint32_t count = 0;
	int ret;

	const struct device *led_dev = device_get_binding(LED0);
	if (led_dev == NULL) {
		LOG_ERR("Cannot get LED device");
		return;
	}

	ret = gpio_pin_configure(led_dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		LOG_ERR("Cannot configure LED pin [%d]", ret);
		return;
	}

	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    float temperature;
    float humidity;
	float pressure;
	float temperature_p;

	sht3x_sensor_t *sht3x = sht3x_init_sensor(i2c_dev, SHT3x_ADDR_1);
	if (sht3x == NULL) {
		LOG_ERR("Cannot init sht3x sensor");
		return;
	}
	// Start periodic measurements with 1 measurement per second.
	sht3x_start_measurement(sht3x, sht3x_periodic_1mps, sht3x_high);

	// Wait until first measurement is ready (constant time of at least 30 ms
	// or the duration returned from *sht3x_get_measurement_duration*).
	k_msleep(sht3x_get_measurement_duration(sht3x_high));

	qmp6988_sensor_t *qmp6988 = qmp6988_init_sensor(i2c_dev, QMP6988_SLAVE_ADDRESS_L);
	if (qmp6988 == NULL) {
		LOG_ERR("Cannot init qmp6988 sensor");
		return;
	}

	sgp30_sensor_t *sgp30 = sgp30_init_sensor(i2c_dev, SGP30_I2C_DEFAULT_ADDRESS);
	if (sgp30 == NULL) {
		LOG_ERR("Cannot init sgp30 sensor");
		return;
	}
	sgp30_initAirQuality(sgp30);
	k_msleep(INIT_AIR_QUALITY_DURATION_MS);

	while (1) {
		gpio_pin_set(led_dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;

		if (sht3x_get_results(sht3x, &temperature, &humidity) &&
		    qmp6988_calcPressure(qmp6988, &pressure, &temperature_p) &&
			!sgp30_measureAirQuality(sgp30)) {
			LOG_INF("[%d]: %d C %d %% %4d hPa(t=%2d) %d ppm CO2 %d ppm TVOC",
				++count,
				(int)temperature, (int)humidity,
				(int)pressure, (int)temperature_p,
				sgp30->CO2, sgp30->TVOC
				);
		}
		sgp30_setCompensation(sgp30, humidity, temperature);
		k_msleep(SLEEP_TIME_MS);
	}
}
