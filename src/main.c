/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sensor);

#include "sht3x.h"
#include "qmp6988.h"
#include "sgp30.h"
#include "battery.h"

#include "app_bt.h"

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

const static struct device *led_dev;

/** A discharge curve specific to the power source. */
static const struct battery_level_point levels[] = {
	/* Linear from maximum voltage to minimum voltage. */
	{ 100, 3600 },
	{ 0, 1700 },
};
static int app_battery_level = 100;
static int app_battery_mV = 3000;

static bool init_i2c_sensors(struct sensors *env_sensors);
static void get_i2c_sensors_values(struct sensors *env_sensors);
static void app_work_handler(struct k_work *work);
static void app_timer_handler(struct k_timer *dummy);

K_WORK_DEFINE(app_work, app_work_handler);
K_TIMER_DEFINE(app_timer, app_timer_handler, NULL);

static void app_work_handler(struct k_work *work)
{
	gpio_pin_set(led_dev, PIN, 0);

	battery_measure_enable(true);
	int batt_mV = battery_sample();
	if (batt_mV < 0) {
		LOG_WRN("Failed to read battery voltage: %d\n",
				batt_mV);
	} else {
		app_battery_mV = batt_mV;
		app_battery_level = battery_level_pptt(batt_mV, levels);
	}
	bt_app_send_battery_level(app_battery_level);
	battery_measure_enable(false);

	get_i2c_sensors_values(&env_sensors);
#ifdef CONFIG_SGP30
	LOG_INF("%d C %d %% %4d hPa(t=%2d) %d ppm CO2 %d ppm TVOC %d mV(%d%%) %d mV(%d%%)",
		(int)env_sensors.temperature, (int)env_sensors.humidity,
		(int)env_sensors.pressure, (int)env_sensors.temperature_p,
		env_sensors.sgp30->CO2, env_sensors.sgp30->TVOC,
		app_battery_mV, app_battery_level
		);
#else
	LOG_INF("%d C %d %% %4d hPa(t=%2d) %d mV(%d%%)",
		(int)env_sensors.temperature, (int)env_sensors.humidity,
		(int)env_sensors.pressure, (int)env_sensors.temperature_p,
		app_battery_mV, app_battery_level
		);
#endif

	gpio_pin_set(led_dev, PIN, 1);
}

static void app_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&app_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
};

static int app_battery_cb(void)
{
	return app_battery_level;
}

static struct bt_app_cb app_callbacks = {
	.battery_cb = app_battery_cb,
};

static bool init_i2c_sensors(struct sensors *env_sensors)
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

static void get_i2c_sensors_values(struct sensors *env_sensors)
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
	int ret;

	led_dev = device_get_binding(LED0);
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

	ret = bt_app_init(&app_callbacks);
	if (ret) {
		LOG_ERR("Failed to init bt [%d]", ret);
		return;
	}

	ret = bt_app_advertise_start();
	if (ret) {
		LOG_ERR("Failed to start advertising [%d]", ret);
		return;
	}

	/* start periodic timer that expires once every second */
	k_timer_start(&app_timer, K_SECONDS(1), K_SECONDS(1));

	// do nothing
}
