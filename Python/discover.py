import asyncio
import time
from bleak import BleakClient, BleakScanner 
from bleak.exc import BleakError

the_device = ""
the_service = ""
client = False

async def BTsearch():
    global the_device
    devices = await BleakScanner.discover()
    for i,d in enumerate(devices):
        print(f"[{i}]\t{d.name}\t{d.address}")
        if "BT05" in d.name:
            print(f"Potenitial robot found @ {d.address}")
            the_device = d.address
            
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

async def BTdisconnect():
        if client.is_connected:
            await client.disconnect()
            print("The robot have been disconnected...")
        else:
            print("No robot to be disconnected!")
            
async def BTwrite(the_command, redial=True):
        if client.is_connected:
            await client.write_gatt_char(the_service,bytearray(the_command, "utf-8"), response=not True)
        else:
            print("No devce connected.")
            if redial and the_service:
                loop.run_until_complete(BTconnect())

print("Scanning for BLE devices...")
loop = asyncio.get_event_loop()
loop.run_until_complete(BTsearch())

if the_device:
    print(f"there is Mariola at {the_device}")
    loop.run_until_complete(print_services(the_device))
    if the_service:
        print(f"Found Vendor sercvice at {the_service}")
        # as we found what we were looking for we try to connect
        time.sleep(2)
        trys = 0
        while trys < 5:
            try:
                loop.run_until_complete(BTconnect())
                break
            except:
                trys += 1
                print("issue....")

        
        loop.run_until_complete(BTwrite('<1,0,30,500>'))


        print("Done...")
        # and we disconnect
        loop.run_until_complete(BTdisconnect())

else:
    print("Can't find robot :(")
