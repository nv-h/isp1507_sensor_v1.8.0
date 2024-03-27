import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from struct import unpack
from datetime import datetime

import argparse

DEVICE_NAME = "nrf52_sensor"
SERVICE_UUID = "c7839aa8-1903-40b5-a8f0-426e09ffb390"
CHARACTERISTIC_UUID = "c7839aa9-1903-40b5-a8f0-426e09ffb390"

sec = 10
notify_count = 0

def notify_handler(sender, data):
    unpacked = unpack("<ifffhh", data)
    now = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
    print(f"{now} {unpacked[0]}, {unpacked[1]:.2f}, {unpacked[2]:.2f}, {unpacked[3]:.2f}, {unpacked[4]}, {unpacked[5]}")

    global notify_count
    if notify_count % 120 == 0:
        with open("sensor_data.csv", "a") as f:
            f.write(f"{now}, {unpacked[0]}, {unpacked[1]:.2f}, {unpacked[2]:.2f}, {unpacked[3]:.2f}, {unpacked[4]}, {unpacked[5]}\n")

    notify_count += 1


async def main():
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    if not device:
        print(f"{DEVICE_NAME} not found")
        return

    print(f"Found device: {device.name} / {device.address}")

    for i in range(10):
        try:
            async with BleakClient(device) as client:
                print(f"time, battery, temperature, humidity, pressure, co2, tvoc")
                with open("sensor_data.csv", "w") as f:
                    f.write("time, battery, temperature, humidity, pressure, co2, tvoc\n")
                await client.start_notify(CHARACTERISTIC_UUID, notify_handler)
                await asyncio.sleep(sec)
                await client.stop_notify(CHARACTERISTIC_UUID)

                break
        except Exception as e:
            print(e)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sec", type=int, default=10)
    args = parser.parse_args()
    sec = args.sec

    asyncio.run(main())
