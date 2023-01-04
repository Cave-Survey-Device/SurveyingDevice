import asyncio
from bleak import BleakScanner
from bleak import BleakClient


address = "8C:4B:14:10:E0:92"
MODEL_NBR_UUID = "00002a24-0000-1000-8000-00805f9b34fb"
BEAN_BOI_SERVICE = "8040889e-1df9-4279-ad09-7cc5ed83761a"
AZIMUTH_CHARACTERISTIC_UUID = "00e72748-8854-4107-857c-001140e0f1fc"
INCLINATION_CHARACTERISTIC_UUID = "c501a8ac-5334-4e1d-8d45-5befdc98f923"
ID_CHARACTERISTIC_UUID = "c5a74ffa-c011-480c-83ac-b4601d0455a4"

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        print(d)

async def connect_to_device(address):
    async with BleakClient(address) as client:
        model_number = await client.read_gatt_char(MODEL_NBR_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

        client.start_notify(AZIMUTH_CHARACTERISTIC_UUID, callback)
        client.start_notify(INCLINATION_CHARACTERISTIC_UUID, callback)
        client.start_notify(ID_CHARACTERISTIC_UUID, callback)


def callback(sender: int, data: bytearray):
    print(f"{sender}: {data}")




asyncio.run(main())
asyncio.run(connect_to_device(address))
