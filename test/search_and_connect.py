import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from bleak import exc
from struct import unpack

DEVICE_NAME = "nrf52_sensor"
SERVICE_UUID = "c7839aa8-1903-40b5-a8f0-426e09ffb390"
CHARACTERISTIC_UUID = "c7839aa9-1903-40b5-a8f0-426e09ffb390"


async def main(target: str):
    found_devices = []
    for i in range(5):
        devices = await BleakScanner.discover(timeout=1.0)
        # Retry multiple times because it may not be found
        for d in devices:
            if d.name is None :
                continue

            if d.name == target:
                found_devices.append(d)

        if len(found_devices) == 0:
            print(f"No board found. [{i}]")
        else:
            break

    for device in found_devices:
        async with BleakClient(device, timeout=20.0) as client:
            print(f"connected: {device.name}")

            read_bytes = await client.read_gatt_char(CHARACTERISTIC_UUID)
            read_data = unpack("<ifffhh", read_bytes)
            print(f"{read_data[0]/1000:.3f} V {read_data[1]:.2f} C {read_data[2]:.2f} % {read_data[3]:.2f} hPa {read_data[4]} ppm {read_data[5]} ppm")

            print("disconnecting ...")
            break


if __name__ == "__main__":
    asyncio.run(main(DEVICE_NAME))