import alarm
import board
import digitalio
import ssl
import socketpool
import time
import wifi

from adafruit_ds18x20 import DS18X20
from adafruit_io.adafruit_io import IO_MQTT, IO_HTTP
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
from adafruit_onewire.bus import OneWireBus
import adafruit_requests

try:
    from secrets import secrets
except ImportError:
    print("Could not import WiFi and Adafruit IO credentials from secrets.py.")
    raise


__version__ = "0.8.0"

# Time between updates, in seconds.
INTERVAL = 300  # How long the display remains on, in seconds.

# Hardware settings
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


def time_tuple_to_secs(time_tuple):
    return time_tuple[0] * 3600 + time_tuple[1] * 60 + time_tuple[2]


def time_struct_to_secs(time_struct):
    return time_struct.tm_hour * 3600 + time_struct.tm_min * 60 + time_struct.tm_sec


def time_struct_to_hours(time_struct):
    return time_struct_to_secs(time_struct) / 3600.0


class LatchingRelay:
    def __init__(self, set_pin, unset_pin, is_enabled=False):
        # Set up pins
        self._set = digitalio.DigitalInOut(set_pin)
        self._set.direction = digitalio.Direction.OUTPUT
        self._unset = digitalio.DigitalInOut(unset_pin)
        self._unset.direction = digitalio.Direction.OUTPUT
        self.is_enabled = bool(is_enabled)

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
    def __init__(self, lights_on_time, lights_off_time, set_pin, unset_pin, is_enabled):
        super().__init__(set_pin, unset_pin, is_enabled)
        self.on_time = float(lights_on_time)
        self.off_time = float(lights_off_time)

    def update(self, now_struct):
        # Note, for now this logic assumes the lights off time is later in the day than lights
        # on time, i.e. lights on period does not span midnight.
        hours = time_struct_to_hours(now_struct)
        if hours > self.on_time and hours < self.off_time:
            if not self.is_enabled:
                print("Turning lights on.")
                self.enable()
        if hours < self.on_time or hours > self.off_time:
            if self.is_enabled:
                print("Turning light off.")
                self.disable()
        return int(self.is_enabled)


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

    @property
    def feeding_times(self):
        return self._feeding_times

    @feeding_times.setter
    def feeding_times(self, times):
        times = [float(feeding_time) for feeding_time in times]
        self._n_times = len(times)
        self._feeding_times = times

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
        print(f"Feeding {n_portions} portions.")
        self.rotate(n_portions)

    def update(self, now_struct, meals_fed_today):
        hours = time_struct_to_hours(now_struct)
        meals_fed_today = int(meals_fed_today)
        try:
            next_feed = self._feeding_times[meals_fed_today]
        except IndexError:
            # Done with feeding for today
            return
        if hours > next_feed:
            self.feed(self.portions_per_meal)
            meals_fed_today += 1
            print(f"Fed meal {meals_fed_today} of {self._n_times}.")
        return meals_fed_today


class TemperatureSensor:
    def __init__(self, ow_bus, serial_number, temperature_offset, retries=5):
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
                time.sleep(2)
            else:
                # Got a successful reading. I'm out of here.
                return t + self._temp_offset

        raise RuntimeError("Too many errors attempting to read temperature sensor.")


try:
    start_time = time.monotonic()

    # Read temperature sensors before starting WiFi.
    print("Initialising temperature sensors.")
    ow_bus = OneWireBus(OW_PIN)
    try:
        air_sensor = TemperatureSensor(ow_bus, AIR_SN, AIR_OFFSET, retries=20)
        _ = air_sensor.temperature
        air_temp = air_sensor.temperature
        print(f"Air temperature: {air_temp}C")
    except RuntimeError:
        print("Can't read air temperature sensor.")
        air_temp = 0
    try:
        water_sensor = TemperatureSensor(ow_bus, WATER_SN, WATER_OFFSET, retries=20)
        _ = water_sensor.temperature
        water_temp = water_sensor.temperature
        print(f"Water temperature: {water_temp}C")
    except RuntimeError:
        print("Can't read water temperature sensor.")
        water_temp = 0


    aio_username = secrets["aio_username"]
    aio_key = secrets["aio_key"]

    try:
        print("Connecting to {}".format(secrets["ssid"]))
        wifi.radio.connect(secrets["ssid"], secrets["password"])
    except Exception as err:
        print("Failed to connect to WiFi:\n{}\n".format(err))
        raise err
    else:
        print("Connected to {}".format(secrets["ssid"]))

    # Create a socket pool
    pool = socketpool.SocketPool(wifi.radio)

    # Set up an adafruit_requests Session
    requests = adafruit_requests.Session(socket_pool=pool,
                                         ssl_context=ssl.create_default_context())

    # Initialise an Adafruit IO HTTP Client
    io_http = IO_HTTP(aio_username, aio_key, requests)

    # Set up I2C bus
    i2c = board.I2C()

    # Get current time.
    now_struct = io_http.receive_time()
    print("Got current time {:02d}:{:02d}:{:02d}".format(now_struct.tm_hour,
                                                         now_struct.tm_min,
                                                         now_struct.tm_sec))

    io_http.send_data("scales.airtemperature", air_temp)
    io_http.send_data("scales.watertemperature", water_temp)

    print("Initialising lights.")
    lights = Lights(io_http.receive_data("scales.lightson")['value'],
                    io_http.receive_data("scales.lightsoff")['value'],
                    LIGHTS_ENABLE_PIN,
                    LIGHTS_DISABLE_PIN,
                    int(io_http.receive_data("scales.lightsonoff")['value']))
    io_http.send_data("scales.lightsonoff",
                      lights.update(now_struct))

    print("Initialising feeder.")
    feeder = Feeder((io_http.receive_data("scales.feedtime1")['value'],
                     io_http.receive_data("scales.feedtime2")['value']),
                    io_http.receive_data("scales.feedserves")['value'],
                    FEEDER_MOTOR,
                    FEEDER_STEPS_PER_ROTATION,
                    FEEDER_STEP_DELAY,
                    FEEDER_STEP_STYLE,
                    i2c)
    io_http.send_data("scales.fedtoday",
                      feeder.update(now_struct,
                                    io_http.receive_data("scales.fedtoday")['value']))

except Exception as err:
    print(f"Exception:\n{err}")
finally:
    time_alarm = alarm.time.TimeAlarm(monotonic_time=start_time + INTERVAL)
    alarm.exit_and_deep_sleep_until_alarms(time_alarm)
