import board
import busio
import digitalio
import rtc
import time
from adafruit_pcf8523 import PCF8523
from adafruit_featherwing import minitft_featherwing
from adafruit_onewire.bus import OneWireBus
from adafruit_ds18x20 import DS18X20

# Operation settings
LIGHTS_ON_TIME = (08, 00, 00)  # Time in (HH, mm, ss) format
LIGHTS_OFF_TIME = (20, 00, 00)  # Time in (HH, mm, ss) format

# Hardware settings
HEARTBEAT_PIN = board.RED_LED
HEARTBEAT_DURATION = 0.1
LIGHTS_ENABLE_PIN = board.D11
LIGHTS_DISABLE_PIN = board.D12
OW_PIN = board.D13
WATER_SN = 'aa 87 ed 37 14 01 ee'
WATER_OFFSET = 0.0
AIR_SN = '11 8d 97 0a 00 00 58'
AIR_OFFSET = 0.0


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


class TemperatureSensor:
    def __init__(self, ow_bus, serial_number, temperature_offset):
        sn_bytes = bytes.from_hex(serial_number)
        ow_devices = ow_bus.scan()
        ow_sns = [ow_device.serial_number for ow_devices in ow_devices]
        device_index = ow_sns.index(device_sn)
        self._sensor = DS19X20(ow_bus, ow_devices[device_index]
        self._temp_offset = temperature_offset

    @property
    def temperature:
        return self._sensor.temperature + self._temp_offset


class Aquarium:
    def __init__(self,
                 lights_on_time=LIGHTS_ON_TIME,
                 lights_off_time=LIGHTS_OFF_TIME,
                 heartbeat_pin=HEARTBEAT_PIN,
                 heartbeat_duration=HEARTBEAT_DURATION,
                 lights_enable_pin=LIGHTS_ENABLE_PIN,
                 lights_disable_pin=LIGHTS_DISABLE_PIN,
                 ow_pin=OW_PIN,
                 water_sn=WATER_SN,
                 water_offset=WATER_OFFSET,
                 air_sn=AIR_SN,
                 air_offset=AIR_OFFSET
                 ):
        self._lights_on_secs = self.time_tuple_to_mins(lights_on_time)
        self._lights_off_secs = self.time_tuple_to_mins(lights_off_time)

        # Set up heartbeat output (i.e red LED)
        self._heartbeat = digitalio.DigitalInOut(HEARTBEAT_PIN)
        self._heartbeat.direction = digitalio.Direction.OUTPUT
        self._heartbeat_duration = heartbeat_duration

        # Set up latching relay for lights
        self._lights_relay = LatchingRelay(set_pin=lights_disable_pin, unset_pin=lights_disable_pin)
        self._lights_relay.disable()
        self._lights_on = False

        # Set up I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)

        # Set up real time clock as source for time.time() or time.local_time() calls.
        clock = PCF8523(i2c)
        rtc.set_time_source(clock)

        # Set up 1-Wire temperature sensors
        ow_bus = OneWireBus(board.D13)
        self._water_sensor = TemperatureSensor(ow_bus, water_sn, water_offset)
        self._air_sensor = TemperatureSensor(ow_bus, air_sn, air_offset)

        # Set up display
        minitft = minitft_featherwing.MiniTFTFeatherWing(i2c=i2c)

    def time_tuple_to_mins(self, time_tuple):
        return time_tuple[0] * 3600 + time_tuple[1] * 60 + time_tuple[2]

    def heartbeat(self):
        self._heartbeat.value = True
        time.sleep(self._heartbeat_duration)
        self._heartbeat.value = False

    def update_lights(self, date_time):
        # Note, for now this logic assumes the lights off time is later in the day than lights
        # on time, i.e. lights on period does not span midnight.
        secs = self.time_tuple_to_mins((date_time.tm_hour, date_time.tm_min, date_time.tm_sec))
        if secs > self._lights_on_secs and secs < self._lights_off_secs:
           if not self._lights_on:
               self._lights_relay.enable()
               self._lights_on = True
        if secs < self._lights_on_secs or secs > self._lights_off_secs:
            if self._lights_on:
                self._lights_relay.disable()
                self._lights_on = False

    def update_display(self, date_time, water_temp, air_temp):
        time_string = "Time: %d:%02d:%02d\n" % (date_time.tm_hour, date_time.tm_min, date_time.tm_sec)
        water_string = "Water temp: {0:0.4f}C\n".format(water_temp)
        ai_string = "Water temp: {0:0.4f}C\n".format(air_temp)
        output = time_string + water_string + air_string + "\n"
        print(output)

    def run_once(self):
        self.heartbeat()
        now = time.local_time()
        self.update_lights(now)
        water_temp = self._water_sensor.temperature
        air_temp = self._air_sensor.temperature
        self.update_display(now, water_temp, air_temp)

    def run(self):
        while True:
            self.run_once()


aquarium = Aquarium()
aquarium.run()