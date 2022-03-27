import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from struct import unpack

DEVICE_NAME = "isp1507_sensor"
SERVICE_UUID = "c7839aa8-1903-40b5-a8f0-426e09ffb390"
BATTERY_CHARACTERISTIC_UUID = "c7839aa9-1903-40b5-a8f0-426e09ffb390"


def notify_handler(sender, data):
    bt_lvl = unpack("<i", data)[0]
    print(f"notify: {bt_lvl}")


async def main():
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name and d.name.lower() == DEVICE_NAME.lower()
    )
    if not device:
        print(f"{DEVICE_NAME} not found")
        return

    print(f"Found device: {device.name} / {device.address}")

    async with BleakClient(device, timeout=20.0) as client:
        print("connected")

        print('notify handler start')
        await client.start_notify(BATTERY_CHARACTERISTIC_UUID, notify_handler)

        await asyncio.sleep(5.0)

        print('notify handler stop')
        await client.stop_notify(BATTERY_CHARACTERISTIC_UUID)


asyncio.run(main())