#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <logging/log.h>

#include "app_bt.h"

LOG_MODULE_REGISTER(app_bt, CONFIG_LOG_DEFAULT_LEVEL);

static bool notify_enabled;
static int batery_level;
static struct bt_app_cb app_cb;

static void app_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_button(struct bt_conn *conn,
						   const struct bt_gatt_attr *attr,
						   void *buf,
						   uint16_t len,
						   uint16_t offset)
{
	const char *value = attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p",
		attr->handle, (void *)conn);

	if (app_cb.battery_cb) {
		batery_level = app_cb.battery_cb();
		return bt_gatt_attr_read(
			conn, attr, buf, len, offset, value, sizeof(*value)
			);
	}

	return 0;
}

BT_GATT_SERVICE_DEFINE(app_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_BASE),
	BT_GATT_CHARACTERISTIC(
		BT_UUID_BATTERY,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ, read_button, NULL,
		&batery_level),
	BT_GATT_CCC(app_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int bt_app_init(struct bt_app_cb *callbacks)
{
	if (callbacks) {
		app_cb.battery_cb = callbacks->battery_cb;
	}

	return bt_enable(NULL);
}

int bt_app_send_battery_level(int battery_level)
{
	if (!notify_enabled) {
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &app_svc.attrs[2],
				  &battery_level,
				  sizeof(battery_level));
}

int bt_app_advertise_start(void)
{
	const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	};

	const struct bt_data sd[] = {
		BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BASE_VAL),
	};

	return bt_le_adv_start(
		BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
		sd, ARRAY_SIZE(sd));
}