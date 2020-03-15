import board
import busio
import digitalio
import time
from adafruit_pcf8523 import PCF8523
from adafruit_featherwing import minitft_featherwing
from adafruit_onewire.bus import OneWireBus
from adafruit_ds18x20 import DS18X20

# Set up latching relay for lights
lights_relay = LatchingRelay(set_pin=board.D11, unset_pin=board.D12)

# Set up 1-Wire temperature sensors
AIR_ID = ['0x28', '0x11', '0x8d', '0x97', '0xa', '0x0', '0x0', '0x58']
WATER_ID = ['0x28', '0xaa', '0x87', '0xed', '0x37', '0x14', '0x1', '0xee']
ow_bus = OneWireBus(board.D13)
ow_devices = ow_bus.scan()
ow_ids = [[hex(i) for i in ow_device.rom] for ow_device in ow_devices]
air_sensor = DS18X20(ow_bus, ow_devices[ow_ids.index(AIR_ID)])
water_sensor = DS18X20(ow_bus, ow_devices[ow_ids.index(WATER_ID)])

# Set up I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Set up real time clock
rtc = PCF8523(i2c)

# Set up display
minitft = minitft_featherwing.MiniTFTFeatherWing(i2c=i2c)


class LatchingRelay:
    def __init__(self, set_pin, unset_pin):
        # Set up pins
        self._set = digitalio.DigitalInOut(set_pin)
        self._set.direction = digitalio.Direction.OUTPUT
        self._unset = digitalio.DigitalInOut(unset_pin)
        self._unset.direction = digitalio.Direction.OUTPUT

    def enable(self):
        # Pull set pin high for >10ms to set relay
        self._set.value = True
        time.sleep(0.02)
        self._set.value = False

    def disable(self):
        # Pull unset pin high for >10ms to unset relay
        self._unset.value = True
        time.sleep(0.02)
        self._unset.value = False


while True:
    led.value = True
    t = rtc.datetime
    time_string = "Time: %d:%02d:%02d\n" % (t.tm_hour, t.tm_min, t.tm_sec)
    air_string = "Air temp: {0:0.4f}C\n".format(air_sensor.temperature)
    water_string = "Water temp: {0:0.4f}C\n".format(water_sensor.temperature)
    cpu_string = "CPU temp: {0:0.2f}C\n".format(microcontroller.cpu.temperature)
    output = time_string + air_string + water_string + cpu_string
    print(output)
    led.value = False
    time.sleep(1)