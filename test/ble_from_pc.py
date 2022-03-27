import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from struct import unpack

DEVICE_NAME = "isp1507_sensor"
SERVICE_UUID = "c7839aa8-1903-40b5-a8f0-426e09ffb390"
CHARACTERISTIC_UUID = "c7839aa9-1903-40b5-a8f0-426e09ffb390"


def notify_handler(sender, data):
    unpacked = unpack("<ifff", data)
    print(f"notify: {unpacked}")


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

        val = await client.read_gatt_char(CHARACTERISTIC_UUID)
        print(f'read: {unpack("<ifff", val)}')

        print('notify handler start')
        await client.start_notify(CHARACTERISTIC_UUID, notify_handler)

        await asyncio.sleep(10.0)

        print('notify handler stop')
        await client.stop_notify(CHARACTERISTIC_UUID)


asyncio.run(main())