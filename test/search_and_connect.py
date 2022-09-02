import asyncio
from bleak import discover
from bleak import BleakClient

DEVICE_NAME = "isp1507_sensor"
SERVICE_UUID = "c7839aa8-1903-40b5-a8f0-426e09ffb390"
CHARACTERISTIC_UUID = "c7839aa9-1903-40b5-a8f0-426e09ffb390"


async def main(target: str):
    found_devices = []
    for i in range(5):
        devices = await discover()
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
        for i in range(10):
            # Even connected, may get a mysterious `CancelledError`, so retry 10 times
            try:
                async with BleakClient(device, timeout=20.0) as client:
                    print(f"connected: {device.name}")

                    read_data = await client.read_gatt_char(CHARACTERISTIC_UUID)
                    print(f"read: {read_data}")

                    print("disconnecting ...")
                    break
            except asyncio.exceptions.CancelledError as e:
                print(f"Cancelled[{i}]: {e}")
                continue
            except OSError as e:
                print(f"OSError[{i}]: {e}")
                continue
            except asyncio.exceptions.TimeoutError as e:
                print(f"TimeoutError: {e}")
                break
            except KeyboardInterrupt as e:
                print(f"KeyboardInterrupt: {e}")
                break


if __name__ == "__main__":
    asyncio.run(main(DEVICE_NAME))