from _bleio import BluetoothError
import board
import busio
import digitalio
import displayio
import rtc
import storage
import time
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_ds18x20 import DS18X20
from adafruit_featherwing import minitft_featherwing
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
from adafruit_onewire.bus import OneWireBus
from adafruit_pcf8523 import PCF8523
from adafruit_sdcard import SDCard

__version__ = "0.6.1"

# Operation settings
LIGHTS_ON_TIME = (07, 30, 00)  # Time in (HH, mm, ss) format
LIGHTS_OFF_TIME = (19, 30, 30)  # Time in (HH, mm, ss) format
FEEDING_TIMES = ((08, 30, 00), (18, 00, 00))
PORTIONS_PER_MEAL = 2
DISPLAY_TIMEOUT = 300  # How long the display remains on, in seconds.
LOG_DATA = True
LOG_INTERVAL = (00, 05, 00)  # Time in (HH, mm, ss) format
BLE_NAME = "Scales Aquarium"

# Hardware settings
HEARTBEAT_PIN = board.RED_LED
HEARTBEAT_DURATION = 0.02  # seconds
LIGHTS_ENABLE_PIN = board.D11
LIGHTS_DISABLE_PIN = board.D12
FEEDER_MOTOR = "stepper1"
FEEDER_STEPS_PER_ROTATION = 513
FEEDER_STEP_DELAY = 0.01
FEEDER_STEP_STYLE = stepper.DOUBLE
OW_PIN = board.D13
WATER_SN = b'\x28\xaa\x87\xed\x37\x14\x01\xee'
WATER_OFFSET = 0.0625  # Temperature sensor zero point offset in degrees C
AIR_SN = b'\x28\xaa\x83\x95\x53\x14\x01\x7e'
AIR_OFFSET = -0.125  # Temperature sensor zero point offset in degrees C
SD_CS = board.D10


def time_tuple_to_secs(time_tuple):
    return time_tuple[0] * 3600 + time_tuple[1] * 60 + time_tuple[2]


def time_struct_to_secs(time_struct):
    return time_struct.tm_hour * 3600 + time_struct.tm_min * 60 + time_struct.tm_sec


class LatchingRelay:
    def __init__(self, set_pin, unset_pin):
        # Set up pins
        self._set = digitalio.DigitalInOut(set_pin)
        self._set.direction = digitalio.Direction.OUTPUT
        self._unset = digitalio.DigitalInOut(unset_pin)
        self._unset.direction = digitalio.Direction.OUTPUT
        self.disable()

    def enable(self):
        # Pull set pin high for >10ms to set relay
        self._set.value = True
        time.sleep(0.02)
        self._set.value = False
        self.is_enabled = True

    def disable(self):
        # Pull unset pin high for >10ms to unset relay
        self._unset.value = True
        time.sleep(0.02)
        self._unset.value = False
        self.is_enabled = False


class Lights(LatchingRelay):
    def __init__(self, lights_on_time, lights_off_time, set_pin, unset_pin):
        super().__init__(set_pin, unset_pin)
        self.on_time = lights_on_time
        self.off_time = lights_off_time
        self.update()

    @property
    def on_time(self):
        return self._on_time

    @on_time.setter
    def on_time(self, on_time):
        self._on_time = on_time
        self._on_secs = time_tuple_to_secs(on_time)

    @property
    def off_time(self):
        return self._off_time

    @off_time.setter
    def off_time(self, off_time):
        self._off_time = off_time
        self._off_secs = time_tuple_to_secs(off_time)

    def update(self):
        # Note, for now this logic assumes the lights off time is later in the day than lights
        # on time, i.e. lights on period does not span midnight.
        now = time.localtime()
        secs = time_struct_to_secs(now)
        if secs > self._on_secs and secs < self._off_secs:
            if not self.is_enabled:
                print("Turning lights on.\n\n\n\n")
                self.enable()
        if secs < self._on_secs or secs > self._off_secs:
            if self.is_enabled:
                print("Turning light off.\n\n\n\n")
                self.disable()


class StepperMotor:
    def __init__(self, motor, steps_per_rotation, step_delay, step_style, i2c):
        self._kit = MotorKit(i2c=i2c)
        self._stepper = getattr(self._kit, motor)
        self._stepper.release()
        self._steps_per_rotation = int(steps_per_rotation)
        self._delay = step_delay
        assert step_style in {stepper.SINGLE, stepper.DOUBLE}  # Don't want to microstep this geared motor
        self._style = step_style

    def rotate(self, n_rotations):
        self.move(n_rotations * self._steps_per_rotation)

    def move(self, n_steps):
        n_steps = int(n_steps)
        if n_steps > 0:
            direction = stepper.FORWARD
        elif n_steps < 0:
            direction = stepper.BACKWARD
        else:
            return
        try:
            for i in range(abs(n_steps)):
                self._stepper.onestep(direction=direction, style=self._style)
                time.sleep(self._delay)
        finally:
            # De-energise the coils
            self._stepper.release()


class Feeder(StepperMotor):
    def __init__(self, feeding_times, portions_per_meal, motor, steps_per_rotation, step_delay, step_style, i2c):
        super().__init__(motor, steps_per_rotation, step_delay, step_style, i2c)
        self.feeding_times = feeding_times
        self.portions_per_meal = portions_per_meal
        self._meals_fed_today = 0
        now = time.localtime()
        secs = time_struct_to_secs(now)
        self._last_update = secs
        for fs in self._feeding_secs:
            if secs > fs:
                self._meals_fed_today += 1

    @property
    def feeding_times(self):
        return self._feeding_times

    @feeding_times.setter
    def feeding_times(self, times):
        self._n_times = len(times)
        self._feeding_times = times
        self._feeding_secs = tuple(time_tuple_to_secs(t) for t in times)

    @property
    def portions_per_meal(self):
        return self._portions_per_meal

    @portions_per_meal.setter
    def portions_per_meal(self, n_portions):
        n_portions = int(n_portions)
        assert n_portions > 0
        self._portions_per_meal = n_portions

    def feed(self, n_portions=1):
        n_portions = int(n_portions)
        assert n_portions > 0
        print(f"Feeding {n_portions} portions.\n\n\n\n")
        self.rotate(n_portions)


    def update(self):
        now = time.localtime()
        secs = time_struct_to_secs(now)
        if secs < self._last_update:
            # Midnight has past since last update
            self._meals_fed_today = 0
        self._last_update = secs
        try:
            next_feed = self._feeding_secs[self._meals_fed_today]
        except IndexError:
            # Done with feeding for today
            return
        if secs > next_feed:
            self.feed(self.portions_per_meal)
            self._meals_fed_today += 1
            print(f"Fed meal {self._meals_fed_today} of {self._n_times}.\n\n\n\n")


class TemperatureSensor:
    def __init__(self, ow_bus, serial_number, temperature_offset, retries=3):
        ow_devices = ow_bus.scan()
        ow_sns = [ow_device.rom for ow_device in ow_devices]
        device_index = ow_sns.index(serial_number)
        self._sensor = DS18X20(ow_bus, ow_devices[device_index])
        self._sensor.resolution = 12
        self._temp_offset = temperature_offset
        self._retries = retries

    @property
    def temperature(self):
        for attempt in range(self._retries):
            try:
                t = self._sensor.temperature
            except RuntimeError as err:
                # Sometimes get CRC errors with BLE on?
                print(err)
            else:
                # Got a successful reading. I'm out of here.
                return t + self._temp_offset

        raise RuntimeError("Too many errors attempting to read temperature sensor.")


class Display(minitft_featherwing.MiniTFTFeatherWing):
    def __init__(self, aquarium, display_timeout, i2c, spi):
        super().__init__(i2c=i2c, spi=spi)
        self.aquarium = aquarium
        self.timeout = display_timeout
        self.turned_on = time_struct_to_secs(time.localtime())

    @property
    def is_enabled(self):
        return self.backlight < 1.0

    def enable(self):
        self.backlight = 0.0
        self.turned_on = time_struct_to_secs(time.localtime())

    def disable(self):
        self.backlight = 1.0

    def update(self):
        if any(self.buttons):
            self.enable()
        now = time.localtime()
        secs = time_struct_to_secs(now)
        if self.is_enabled and secs > self.turned_on + self.timeout:
            self.disable()

        time_string = "Time:   %d:%02d:%02d\n" % (now.tm_hour, now.tm_min, now.tm_sec)
        lights_string = "Lights: {}\n".format("On" if self.aquarium.lights.is_enabled else "Off")
        water_string = "Water:  {:7.4f}C\n".format(self.aquarium.water_temp)
        air_string = "Air:    {:7.4f}C\n".format(self.aquarium.air_temp)
        output = time_string + lights_string + water_string + air_string
        print(output)


class Aquarium:
    def __init__(self):
        # Set up heartbeat output (i.e red LED)
        self._heartbeat = digitalio.DigitalInOut(HEARTBEAT_PIN)
        self._heartbeat.direction = digitalio.Direction.OUTPUT
        self._heartbeat_duration = HEARTBEAT_DURATION

        # Set up I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        # Set up SPI bus
        spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

        # Set up real time clock as source for time.time() or time.localtime() calls.
        print("Initialising real time clock.\n\n\n\n")
        clock = PCF8523(i2c)
        rtc.set_time_source(clock)

        print("Initialising display.\n\n\n\n")
        self.display = Display(self, DISPLAY_TIMEOUT, i2c, spi)

        print("Initialising lights.\n\n\n\n")
        self.lights = Lights(LIGHTS_ON_TIME, LIGHTS_OFF_TIME, LIGHTS_ENABLE_PIN, LIGHTS_DISABLE_PIN)

        print("Initialising feeder.\n\n\n\n")
        self.feeder = Feeder(FEEDING_TIMES, PORTIONS_PER_MEAL, FEEDER_MOTOR, FEEDER_STEPS_PER_ROTATION,
                             FEEDER_STEP_DELAY, FEEDER_STEP_STYLE, i2c)

        print("Initialising temperature sensors.\n\n\n\n")
        ow_bus = OneWireBus(OW_PIN)
        self.water_sensor = TemperatureSensor(ow_bus, WATER_SN, WATER_OFFSET)
        self.air_sensor = TemperatureSensor(ow_bus, AIR_SN, AIR_OFFSET)

        # Set up SD card
        print("Setting up logging.\n\n\n\n")
        cs = digitalio.DigitalInOut(SD_CS)
        sdcard = SDCard(spi, cs)
        vfs = storage.VfsFat(sdcard)
        storage.mount(vfs, "/sd")
        self._log_data = LOG_DATA
        self._log_interval = time_tuple_to_secs(LOG_INTERVAL)
        self._last_log = None

        print("Initialising Bluetooth.\n\n\n\n")
        self._ble = BLERadio()
        self._ble._adapter.name = BLE_NAME
        self._ble_uart = UARTService()
        self._ble_ad = ProvideServicesAdvertisement(self._ble_uart)

    def heartbeat(self):
        self._heartbeat.value = True
        time.sleep(self._heartbeat_duration)
        self._heartbeat.value = False

    def update_temps(self):
        self.water_temp = self.water_sensor.temperature
        self.air_temp = self.air_sensor.temperature

    def update_log(self):
        if not self._log_data:
            return

        if self._last_log:
            last_log_secs = time_struct_to_secs(self._last_log)
            current_secs = time_struct_to_secs(self._now)
            if current_secs - last_log_secs < self._log_interval:
                return

        print("Updating log:")
        datestamp, timestamp, log_line = self._get_status_strings()
        filename = datestamp + ".log"
        print(filename)
        print(log_line)
        with open("/sd/scales_logs/" + filename, mode="at", buffering=1) as logfile:
            logfile.write(log_line + "\n")
        self._last_log = self._now
        self._last_log = self._now
        print("Done.\n")

    def blelele(self):
        if not self._ble.connected:
            # Not connected, so make sure we're advertising for connections.
            try:
                self._ble.start_advertising(self._ble_ad)
            except BluetoothError:
                # Already advertising. Probably.
                pass
            return

        if self._ble_uart.in_waiting:
            # There's a command waiting.
            ble_command = self._ble_uart.readline()
            if ble_command:
                # First echo command, then respond.
                self._ble_uart.write(ble_command + b'\n')
                self._ble_respond(ble_command)

    def run_once(self):
        self.heartbeat()
        self._now = time.localtime()
        self.lights.update()
        self.feeder.update()
        self.update_temps()
        self.display.update()
        self.update_log()
        self.blelele()
        time.sleep(1)

    def run(self):
        while True:
            self.run_once()

    def _ble_respond(self, ble_command):
        if ble_command == b"v?":
            response = bytes(f"{BLE_NAME} v{__version__}\n", 'ascii')
        elif ble_command == b"s?":
            _, _, response = self._get_status_strings()
            response = bytes(response, 'ascii')
        elif ble_command == b"f?":
            response = f"{self.feeder.feeding_times}, {self.feeder.portions_per_meal}"
            response = bytes(response, 'ascii')
        elif ble_command == b"ff":
            self.feeder.feed()
            response = bytes("Fed 1 portion.", 'ascii')
        elif len(ble_command) > 2 and ble_command[:2] == b"fp":
            portions = int(str(ble_command[2:], 'ascii'))
            self.feeder.portions_per_meal = portions
            response = bytes(f"Set portions per meal to {portions}.", 'ascii')
        else:
            command = str(ble_command, 'ascii')
            response = bytes("ERROR: Invalid command '{}'\n".format(command), 'ascii')

        self._ble_uart.write(response)

    def _get_status_strings(self):
        datestamp = "{:04d}-{:02d}-{:02d}".format(self._now.tm_year, self._now.tm_mon, self._now.tm_mday)
        timestamp = datestamp + "T{:02d}:{:02d}:{:02d}".format(self._now.tm_hour, self._now.tm_min, self._now.tm_sec)
        status = "{}, {:d}, {:7.4f}, {:7.4f}".format(timestamp, self.lights.is_enabled, self.water_temp, self.air_temp)
        return datestamp, timestamp, status


displayio.release_displays()
aquarium = Aquarium()
aquarium.run()