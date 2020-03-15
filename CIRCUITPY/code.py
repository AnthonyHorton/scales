import board
import busio
import digitalio
import displayio
import rtc
import storage
import time
from adafruit_ds18x20 import DS18X20
from adafruit_featherwing import minitft_featherwing
from adafruit_onewire.bus import OneWireBus
from adafruit_pcf8523 import PCF8523
from adafruit_sdcard import SDCard

# Operation settings
LIGHTS_ON_TIME = (08, 00, 00)  # Time in (HH, mm, ss) format
LIGHTS_OFF_TIME = (20, 00, 00)  # Time in (HH, mm, ss) format
LOG_DATA = True
LOG_INTERVAL = (00, 05, 00)  # Time in (HH, mm, ss) format

# Hardware settings
HEARTBEAT_PIN = board.RED_LED
HEARTBEAT_DURATION = 0.02  # seconds
LIGHTS_ENABLE_PIN = board.D11
LIGHTS_DISABLE_PIN = board.D12
OW_PIN = board.D13
WATER_SN = b'\x28\xaa\x87\xed\x37\x14\x01\xee'
WATER_OFFSET = 0.0  # Temperature sensor zero point offset in degrees C
AIR_SN = b'\x28\x11\x8d\x97\x0a\x00\x00\x58'
AIR_OFFSET = 0.0  # Temperature sensor zero point offset in degrees C
SD_CS = board.D10


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
        ow_devices = ow_bus.scan()
        ow_sns = [ow_device.rom for ow_device in ow_devices]
        device_index = ow_sns.index(serial_number)
        self._sensor = DS18X20(ow_bus, ow_devices[device_index])
        self._temp_offset = temperature_offset

    @property
    def temperature(self):
        return self._sensor.temperature + self._temp_offset


class Aquarium:
    def __init__(self,
                 lights_on_time=LIGHTS_ON_TIME,
                 lights_off_time=LIGHTS_OFF_TIME,
                 log_data=LOG_DATA,
                 log_interval=LOG_INTERVAL,
                 heartbeat_pin=HEARTBEAT_PIN,
                 heartbeat_duration=HEARTBEAT_DURATION,
                 lights_enable_pin=LIGHTS_ENABLE_PIN,
                 lights_disable_pin=LIGHTS_DISABLE_PIN,
                 ow_pin=OW_PIN,
                 water_sn=WATER_SN,
                 water_offset=WATER_OFFSET,
                 air_sn=AIR_SN,
                 air_offset=AIR_OFFSET):
        # Set up heartbeat output (i.e red LED)
        self._heartbeat = digitalio.DigitalInOut(HEARTBEAT_PIN)
        self._heartbeat.direction = digitalio.Direction.OUTPUT
        self._heartbeat_duration = heartbeat_duration

        # Set up latching relay for lights
        self._lights_on_secs = self.time_tuple_to_secs(lights_on_time)
        self._lights_off_secs = self.time_tuple_to_secs(lights_off_time)
        self._lights_relay = LatchingRelay(set_pin=lights_enable_pin, unset_pin=lights_disable_pin)
        self._lights_relay.disable()
        self._lights_on = False

        # Set up I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)

        # Set up real time clock as source for time.time() or time.localtime() calls.
        clock = PCF8523(i2c)
        rtc.set_time_source(clock)

        # Set up 1-Wire temperature sensors
        ow_bus = OneWireBus(board.D13)
        self._water_sensor = TemperatureSensor(ow_bus, water_sn, water_offset)
        self._air_sensor = TemperatureSensor(ow_bus, air_sn, air_offset)

        # Set up SD card
        displayio.release_displays()
        spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        cs = digitalio.DigitalInOut(SD_CS)
        sdcard = SDCard(spi, cs)
        vfs = storage.VfsFat(sdcard)
        storage.mount(vfs, "/sd")
        self._log_data = log_data
        self._log_interval = self.time_tuple_to_secs(log_interval)
        self._last_log = None

        # Set up display
        minitft = minitft_featherwing.MiniTFTFeatherWing(i2c=i2c, spi=spi)

    def time_tuple_to_secs(self, time_tuple):
        return time_tuple[0] * 3600 + time_tuple[1] * 60 + time_tuple[2]

    def time_struct_to_secs(self, time_struct):
        return time_struct.tm_hour * 3600 + time_struct.tm_min * 60 + time_struct.tm_sec

    def heartbeat(self):
        self._heartbeat.value = True
        time.sleep(self._heartbeat_duration)
        self._heartbeat.value = False

    def update_lights(self, date_time):
        # Note, for now this logic assumes the lights off time is later in the day than lights
        # on time, i.e. lights on period does not span midnight.
        secs = self.time_tuple_to_secs((date_time.tm_hour, date_time.tm_min, date_time.tm_sec))
        if secs > self._lights_on_secs and secs < self._lights_off_secs:
           if not self._lights_on:
               self._lights_relay.enable()
               self._lights_on = True
        if secs < self._lights_on_secs or secs > self._lights_off_secs:
            if self._lights_on:
                self._lights_relay.disable()
                self._lights_on = False

    def update_temps(self):
        self._water_temp = self._water_sensor.temperature
        self._air_temp = self._air_sensor.temperature

    def update_display(self, date_time):
        time_string = "Time:   %d:%02d:%02d\n" % (date_time.tm_hour, date_time.tm_min, date_time.tm_sec)
        lights_string = "Lights: {}\n".format("On" if self._lights_on else "Off")
        water_string = "Water:  {:7.4f}C\n".format(self._water_temp)
        air_string = "Air:    {:7.4f}C\n".format(self._air_temp)
        output = time_string + lights_string + water_string + air_string
        print(output)

    def update_log(self, date_time):
        if not self._log_data:
            return

        if self._last_log:
            last_log_secs = self.time_struct_to_secs(self._last_log)
            current_secs = self.time_struct_to_secs(date_time)
            if current_secs - last_log_secs < self._log_interval:
                return

        print("Updating log:")
        datestamp = "{:04d}-{:02d}-{:02d}".format(date_time.tm_year, date_time.tm_mon, date_time.tm_mday)
        filename = datestamp + ".log"
        print(filename)
        timestamp = datestamp + "T{:02d}:{:02d}:{:02d}".format(date_time.tm_hour, date_time.tm_min, date_time.tm_sec)
        log_line = "{}, {:d}, {:7.4f}, {:7.4f}".format(timestamp, self._lights_on, self._water_temp, self._air_temp)
        print(log_line)
        with open("/sd/scales_logs/" + filename, mode="at", buffering=1) as logfile:
            logfile.write(log_line + "\n")
        self._last_log = date_time
        print("Done.\n")

    def run_once(self):
        self.heartbeat()
        now = time.localtime()
        self.update_lights(now)
        self.update_temps()
        self.update_display(now)
        self.update_log(now)

    def run(self):
        while True:
            self.run_once()


aquarium = Aquarium()
aquarium.run()