import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from struct import unpack
import urllib.request

from spread_sheet_config import DEPLOY_ID

DEVICE_NAME = "nrf52_sensor"
SERVICE_UUID = "c7839aa8-1903-40b5-a8f0-426e09ffb390"
CHARACTERISTIC_UUID = "c7839aa9-1903-40b5-a8f0-426e09ffb390"

DEPLOY_URL = f'https://script.google.com/macros/s/{DEPLOY_ID}/exec?'
PLACE = 'pc_room'
SENSORS = 'SHT30,QMP6988,SGP30'


# Apps Script
#
# function doGet(e) {
#   const url = "https://docs.google.com/spreadsheets/d/<sheet id>/edit#gid=0";
#   const ss = SpreadsheetApp.openByUrl(url);
#   const sheet = ss.getSheets()[0];
#   const params = {
#     "timestamp": new Date(),
#     "place": e.parameter.place,
#     "sensors": e.parameter.sensors,
#     "temperature": e.parameter.temperature,
#     "humidity": e.parameter.humidity,
#     "pressure": e.parameter.pressure,
#     "co2": e.parameter.co2,
#     "tvoc": e.parameter.tvoc
#   };
#   sheet.appendRow(Object.values(params));
#   return ContentService.createTextOutput('success');
# }


def post_sensor_data(battery, temperature, humidity, pressure, co2, tvoc):
    """Post sensor data to Google Apps Script

    Args:
        battery (int): battery mV
        temperature (float): temperature °C
        humidity (float): humidity %
        pressure (float): pressure hPa
        co2 (int): co2 ppm
        tvoc (int): tvoc ppb
    """
    url = f"{DEPLOY_URL}temperature={temperature}&humidity={humidity}&pressure={pressure}&co2={co2}&tvoc={tvoc}"

    response = urllib.request.urlopen(url)
    html = response.read()
    print(html.decode('utf-8'))


async def read_sensor(client: BleakClient):
    """Read sensor data from nrf52_sensor

    Args:
        client (BleakClient): BleakClient instance

    Returns:
        int: battery mV
        float: temperature °C
        float: humidity %
        float: pressure hPa
        int: co2 ppm
        int: tvoc ppb
    """
    vals = await client.read_gatt_char(CHARACTERISTIC_UUID)

    return unpack("<ifffhh", vals)


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

        battery, temperature, humidity, pressure, co2, tvoc = await read_sensor(client)
        print(f'battery: {battery} mV, temperature: {temperature} °C, humidity: {humidity} %, pressure: {pressure} hPa, co2: {co2} ppm, tvoc: {tvoc} ppb')

        post_sensor_data(battery, temperature, humidity, pressure, co2, tvoc)


if __name__ == "__main__":
    asyncio.run(main())
