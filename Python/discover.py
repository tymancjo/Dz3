import asyncio
import time
from os import device_encoding
from bleak import BleakClient, BleakScanner, cli
from bleak.exc import BleakError

the_device = ""
the_service = ""
client = False

async def BTsearch():
    global the_device
    devices = await BleakScanner.discover()
    for d in devices:
        print(d)
        if "MLT-BT05" in str(d):
            print(f"I have found: {str(d).split(':')}")
            the_device = str(d).split(':')[0]
            
async def print_services(ble_address: str):
    global the_service

    device = await BleakScanner.find_device_by_address(ble_address, timeout=20.0)

    if not device:
        raise BleakError(f"A device with address {ble_address} could not be found.")
    async with BleakClient(device) as client:
        svcs = await client.get_services()
        print("Services:")
        for service in svcs:
            print(service)
            # taking the first string from the last service. 
            # its a hack for the moment - as there must be better way to do it
            # to look for the "Vendor specific"
            if "Vendor specific" in str(service):
                print(service.characteristics)
                for inside in service.characteristics:
                    print(f"Sub info properties: {inside.properties}")
                    print(f"Sub info uuid: {inside.uuid}")
                    the_service = inside.uuid

async def BTconnect():
        global client

        client = BleakClient(the_device,timeout=10)
        await client.connect()
        # Characteristic before it is set
        # print(f'Before: {await client.read_gatt_char(the_service)}')
        print(f"Connection status: {client.is_connected}")
            
async def BTwrite(the_command):
        await client.write_gatt_char(the_service,bytearray(the_command, "utf-8"), response=True)

print("Scanning for BLE devices...")
loop = asyncio.get_event_loop()
loop.run_until_complete(BTsearch())

if the_device:
    print(f"there is Mariola at {the_device}")
    loop.run_until_complete(print_services(the_device))
    if the_service:
        print(f"Found Vendor sercvice at {the_service}")
        # as we found what we were looking for we try to connect
        loop.run_until_complete(BTconnect())
        loop.run_until_complete(BTwrite('<1,0,-720,500>'))
        time.sleep(5)
        loop.run_until_complete(BTwrite('<1,0,720,500>'))

else:
    print("Can't find robot :(")
