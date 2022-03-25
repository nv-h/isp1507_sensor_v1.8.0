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

struct sensors {
	const struct device *i2c_dev;
	sht3x_sensor_t *sht3x;
	qmp6988_sensor_t *qmp6988;
#ifdef CONFIG_SGP30
	sgp30_sensor_t *sgp30;
#endif
	float temperature;
	float humidity;
	float pressure;
	float temperature_p;
};

static struct sensors env_sensors = {
	.i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0)),
	.sht3x = NULL,
	.qmp6988 = NULL,
#ifdef CONFIG_SGP30
	.sgp30 = NULL,
#endif
};

bool init_i2c_sensors(struct sensors *env_sensors)
{
	env_sensors->sht3x = sht3x_init_sensor(env_sensors->i2c_dev, SHT3x_ADDR_1);
	if (env_sensors->sht3x == NULL) {
		LOG_ERR("Cannot init sht3x sensor");
		return false;
	}

	// Wait until first measurement is ready (constant time of at least 30 ms
	// or the duration returned from *sht3x_get_measurement_duration*).
	k_msleep(sht3x_get_measurement_duration(sht3x_high));

	env_sensors->qmp6988 = qmp6988_init_sensor(env_sensors->i2c_dev, QMP6988_SLAVE_ADDRESS_L);
	if (env_sensors->qmp6988 == NULL) {
		LOG_ERR("Cannot init qmp6988 sensor");
		return false;
	}

#ifdef CONFIG_SGP30
	env_sensors->sgp30 = sgp30_init_sensor(env_sensors->i2c_dev, SGP30_I2C_DEFAULT_ADDRESS);
	if (env_sensors->sgp30 == NULL) {
		LOG_ERR("Cannot init sgp30 sensor");
		return false;
	}
	sgp30_initAirQuality(env_sensors->sgp30);
	k_msleep(INIT_AIR_QUALITY_DURATION_MS);
#endif

	return true;
}

void get_i2c_sensors_values(struct sensors *env_sensors)
{
	int ret;
	ret = sht3x_measure(env_sensors->sht3x, &env_sensors->temperature, &env_sensors->humidity);
	qmp6988_calcPressure(env_sensors->qmp6988, &env_sensors->pressure, &env_sensors->temperature_p);
#ifdef CONFIG_SGP30
	sgp30_measureAirQuality(env_sensors->sgp30);
	if (ret) {
		sgp30_setCompensation(env_sensors->sgp30, env_sensors->humidity, env_sensors->temperature);
	}
#endif
}

void main(void)
{
	LOG_MODULE_DECLARE(sensor);

	bool led_is_on = true;
	int ret;

	const struct device *led_dev = device_get_binding(LED0);
	if (led_dev == NULL) {
		LOG_ERR("Cannot get LED device: %s", LED0);
		return;
	}

	ret = gpio_pin_configure(led_dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		LOG_ERR("Cannot configure LED pin [%d]", ret);
		return;
	}

	ret = init_i2c_sensors(&env_sensors);
	if (!ret) {
		LOG_ERR("Cannot init i2c sensors [%d]", ret);
		return;
	}

	while (1) {
		gpio_pin_set(led_dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;

		get_i2c_sensors_values(&env_sensors);
#ifdef CONFIG_SGP30
		LOG_INF("%d C %d %% %4d hPa(t=%2d) %d ppm CO2 %d ppm TVOC",
			(int)env_sensors.temperature, (int)env_sensors.humidity,
			(int)env_sensors.pressure, (int)env_sensors.temperature_p,
			env_sensors.sgp30->CO2, env_sensors.sgp30->TVOC
			);
#else
		LOG_INF("%d C %d %% %4d hPa(t=%2d)",
			(int)env_sensors.temperature, (int)env_sensors.humidity,
			env_sensors.sgp30->CO2, env_sensors.sgp30->TVOC
			);
#endif

		k_msleep(SLEEP_TIME_MS);
	}
}
