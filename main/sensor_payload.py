#! /usr/bin/env python
#
# MicroPython USMART Sensor Payload
#
# This file is part of micropython-usmart-sensor-payload.
# https://github.com/bensherlock/micropython-usmart-sensor-payload
#
#
# MIT License
#
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
"""MicroPython USMART Sensor Payload."""

# Standard Interface for Sensor Payloads for Gateway and Sensor Nodes. 
# For nodes with different sensors available, implement this api and provide your data accordingly.

class SensorPayload:
    """SensorPayload standard class. Inherit form this class to provide your own sensor data."""

    def __init__(self):
        """Initialise."""
        super().__init__()

    def __call__(self):
        return self

    def get_est_acquisition_duration(self) -> float:
        """Get estimated acquisition duration in seconds."""
        pass

    def start_acquisition(self):
        """Start an acquisition. Returns True if started successfully."""
        pass

    def process_acquisition(self):
        """Continue processing the acquisition. To be called periodically within the mainloop. 
        This is where the state machine keeps track on progress. """
        pass

    def is_completed(self):
        """Is acquisition completed flag."""
        pass

    def get_latest_data_as_bytes(self) -> bytes:
        """Get the latest data as a bytes."""
        # Format to be determined
        pass

    def get_latest_data_as_json(self):
        """Get the latest data as a json object. Which can then be loaded into json.dump/dumps."""
        pass

import pyb
from machine import I2C
import struct

import pybd_expansion.main.bme280 as bme280
from pybd_expansion.main.bme280 import BME280

import pybd_expansion.main.lsm303agr as lsm303agr
from pybd_expansion.main.lsm303agr import LSM303AGR

from pybd_expansion.main.ms5837 import MS5837
from pybd_expansion.main.tsys01 import TSYS01


class ExternalSensorPayload(SensorPayload):
    """ Provides Pressure, Depth and Temperature readings
     from BlueRobotics Underwater Pressure Sensor MS5837-30BA. """

    def __init__(self):
        """ Initialise """
        SensorPayload.__init__(self)

        self._ldo2 = None
        self._scl = None
        self._sda = None
        self._i2c = None
        self._ms5837 = None
        self._tsys01 = None
        self._acquisition_duration = None
        self._ms5837_awaiting_valid_measurements = False
        self._tsys01_awaiting_valid_measurements = False
        self._ms5837_pressure = None
        self._ms5837_temperature = None
        self._ms5837_depth = None
        self._tsys01_temperature = None

    def __call__(self):
        return self

    def get_est_acquisition_duration(self) -> float:
        """Get estimated acquisition duration in seconds."""
        return self._acquisition_duration

    def stop_acquisition(self) -> None:
        """ Stop an acquisition: disable power to sensor,
        clear i2c bus and other objects to None"""
        self._ldo2.value(0)  # disable ldo2
        self._ldo2 = None
        self._scl.value(0)  # disable pull-up
        self._scl = None
        self._sda.value(0)  # disable pull-up
        self._sda = None
        self._i2c = None
        self._ms5837 = None
        self._tsys01 = None
        self._ms5837_pressure = None
        self._ms5837_temperature = None
        self._ms5837_depth = None
        self._tsys01_temperature = None
        self._acquisition_duration = None
        self._ms5837_awaiting_valid_measurements = False
        self._tsys01_awaiting_valid_measurements = False

    def start_acquisition(self):
        """Start an acquisition. Returns True if started successfully."""
        # Ensure the sensors are powered and the I2C pullups are enabled.
        self._ldo2 = pyb.Pin('Y5', mode=pyb.Pin.OPEN_DRAIN, pull=None, value=1)
        self._scl = pyb.Pin('PULL_SCL', mode=pyb.Pin.OUT, value=1)  # enable 5.6kOhm X9/SCL pull-up
        self._sda = pyb.Pin('PULL_SDA', mode=pyb.Pin.OUT, value=1)  # enable 5.6kOhm X10/SDA pull-up

        # Instantiate and Initialize the i2c-bus X
        self._i2c = I2C('X', freq=400000)

        pyb.delay(10)  # in ms

        # Instantiate and Initialize the Pressure Sensor
        self._ms5837 = MS5837(self._i2c)
        self._tsys01 = TSYS01(self._i2c)

        init_success_1 = self._ms5837.init()
        pyb.delay(10)  # in ms
        init_success_2 = self._tsys01.init()

        if init_success_1 and init_success_2:
            # Clear measurements
            self._ms5837_pressure = None
            self._ms5837_temperature = None
            self._ms5837_depth = None
            self._tsys01_temperature = None
            self._ms5837_awaiting_valid_measurements = True
            self._tsys01_awaiting_valid_measurements = True
            return True
        else:
            return False

    def process_acquisition(self):
        """Continue processing the acquisition. To be called periodically within the mainloop.
            This is where the state machine keeps track on progress. """
        if self._ms5837 and self._tsys01:
            acquisition_duration = 0
            if self._ms5837_awaiting_valid_measurements:
                start_micro = pyb.micros()
                read_success = self._ms5837.read()
                if read_success:
                    self._ms5837_pressure = self._ms5837.pressure()
                    self._ms5837_temperature = self._ms5837.temperature()
                    self._ms5837_awaiting_valid_measurements = False

                acquisition_duration += pyb.elapsed_micros(start_micro)

            pyb.delay(10)  # in ms
            if self._tsys01_awaiting_valid_measurements:
                start_micro = pyb.micros()
                read_success = self._tsys01.read()
                if read_success:
                    self._tsys01_temperature = self._tsys01.temperature()
                    self._tsys01_awaiting_valid_measurements = False

                acquisition_duration += pyb.elapsed_micros(start_micro)

            self._acquisition_duration = acquisition_duration

        else:
            raise Exception("Sensors are not initialized!")

    def is_completed(self) -> bool:
        """Is acquisition completed flag."""
        return ((not self._ms5837_awaiting_valid_measurements)
                and (not self._tsys01_awaiting_valid_measurements))

    def get_latest_pressure_in_mbar(self) -> float:
        """ Get the latest pressure measurement in MBar """
        return self._ms5837_pressure

    def get_latest_temperature_in_celsius(self) -> float:
        """ Get the latest temperature measurement in degree Celsius """
        return self._tsys01_temperature

    def get_latest_depth_as_float(self, fluid_density: float = 1000.0):
        """ Get the Depth (calculated using pressure and fluid_density in kg/m^3) """
        self._ms5837.set_fluid_density(fluid_density)
        return self._ms5837.depth()

    def get_latest_data_as_bytes(self) -> bytes:
        """Get the latest data as a bytes."""
        # Format as packed floats
        # https://docs.python.org/3/library/struct.html
        packed_bytes = struct.pack("ff", self._ms5837_pressure, self._tsys01_temperature)

        # To unpack
        # ms5837_pressure, ms5837_temperature = struct.unpack("ff", packed_bytes)

        return packed_bytes


class PebSensorPayload(SensorPayload):
    """Sensor Payload for the PYBD Expansion Board (PEB).
    Provides readings from: 3-axis accelerometer, 3-axis magnetometer,
    temperature, pressure, humidity within the housing."""

    def __init__(self):
        """Initialise."""
        SensorPayload.__init__(self)

        self._i2c = None

        self._bme280 = None
        self._bme280_awaiting_valid_measurements = False
        self._bme280_pressure = None
        self._bme280_temperature = None
        self._bme280_humidity = None

        self._lsm303agr = None
        self._lsm303agr_awaiting_valid_measurements = False
        self._lsm303agr_temperature = None
        self._lsm303agr_accel = None
        self._lsm303agr_magneto = None

    def __call__(self):
        return self

    def get_est_acquisition_duration(self) -> float:
        """Get estimated acquisition duration in seconds."""
        return 0.0

    def start_acquisition(self):
        """Start an acquisition. Returns True if started successfully."""

        # Ensure the sensors are powered and the I2C pullups are enabled.
        pyb.Pin.board.EN_3V3.on()
        pyb.Pin('PULL_SCL', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X9/SCL pull-up
        pyb.Pin('PULL_SDA', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X10/SDA pull-up
        # i2c = machine.I2C(1, freq=400000)  # machine.I2C
        i2c = pyb.I2C(1)  # pyb.I2C
        i2c.init(pyb.I2C.MASTER, baudrate=400000)  # pyb.I2C

        # delay
        pyb.delay(10)

        self._bme280 = BME280(i2c)
        self._lsm303agr = LSM303AGR(i2c)

        # Clear measurements
        self._bme280_pressure = None
        self._bme280_temperature = None
        self._bme280_humidity = None

        # Setup to take measurements
        self._bme280.set_ctrl_hum_reg(osrs_h=bme280.OSRS_OVERSAMPLE_X_1)
        self._bme280.set_ctrl_meas_reg(mode=bme280.MODE_NORMAL, osrs_p=bme280.OSRS_OVERSAMPLE_X_1, osrs_t=bme280.OSRS_OVERSAMPLE_X_1)
        self._bme280.set_config_reg(filter=bme280.FILTER_COEF_OFF, t_sb=bme280.STANDBY_T_1000_MS)

        self._bme280_awaiting_valid_measurements = True

        # delay
        pyb.delay(10)

        # Clear measurements
        self._lsm303agr_temperature = None
        self._lsm303agr_accel = None
        self._lsm303agr_magneto = None

        # Setup to take measurements
        # Temperature
        self._lsm303agr.set_accel_ctrl_reg4(bdu=1)
        self._lsm303agr.set_temp_cfg_reg(en=1)

        # Accelerometer
        self._lsm303agr.set_accel_ctrl_reg1(accel_odr=lsm303agr.ACCEL_ODR_1HZ)
        self._lsm303agr.set_accel_ctrl_reg4(accel_fs=lsm303agr.ACCEL_FS_2G, hr=0, bdu=1)  # bdu=1 needed for temperature

        # Magnetometer
        self._lsm303agr.set_magneto_cfg_reg_a(magneto_odr=lsm303agr.MAGNETO_ODR_10HZ, magneto_md=lsm303agr.MAGNETO_MD_CONTINUOUS, lp=0, comp_temp_en=1)
        self._lsm303agr.set_magneto_cfg_reg_b(lpf=1)
        self._lsm303agr.set_magneto_cfg_reg_c(bdu=1)

        self._lsm303agr_awaiting_valid_measurements = True

        return True

    def process_acquisition(self):
        """Continue processing the acquisition. To be called periodically within the mainloop.
            This is where the state machine keeps track on progress. """

        if self._bme280_awaiting_valid_measurements:
            reg_val, im_update, measuring = self._bme280.get_status_reg()
            if not measuring:
                pressure, temperature, humidity = self._bme280.get_sensor_values_float()

                if pressure and temperature and humidity:
                    self._bme280_awaiting_valid_measurements = False
                    self._bme280_pressure = pressure
                    self._bme280_temperature = temperature
                    self._bme280_humidity = humidity

                    # Stop acquisitions
                    self._bme280.set_ctrl_meas_reg(mode=bme280.MODE_SLEEP, osrs_p=bme280.OSRS_OVERSAMPLE_X_1,
                                                   osrs_t=bme280.OSRS_OVERSAMPLE_X_1)

        if self._lsm303agr_awaiting_valid_measurements:
            # Check the temperature status register for new data
            status_reg, tda, tor = self._lsm303agr.get_temp_status_reg_aux()
            if tda:
                temperature = self._lsm303agr.get_temp_output_float()
                self._lsm303agr_temperature = temperature

            # Check the status register for new accelerometer data
            status_reg, xda, yda, zda, zyxda, xor, yor, zor, zyxor = self._lsm303agr.get_accel_status_reg()
            if zyxda:
                x_val, y_val, z_val = self._lsm303agr.get_accel_outputs()
                self._lsm303agr_accel = (x_val, y_val, z_val)

            # Check the status register for new magnetometer data
            status_reg, xda, yda, zda, zyxda, xor, yor, zor, zyxor = self._lsm303agr.get_magneto_status_reg()
            if zyxda:
                x_val, y_val, z_val = self._lsm303agr.get_magneto_outputs()
                self._lsm303agr_magneto = (x_val, y_val, z_val)

            if self._lsm303agr_temperature and self._lsm303agr_accel and self._lsm303agr_magneto:
                self._lsm303agr_awaiting_valid_measurements = False

                # Stop acquisitions
                self._lsm303agr.set_accel_ctrl_reg1(accel_odr=lsm303agr.ACCEL_ODR_POWERDOWN)
                self._lsm303agr.set_magneto_cfg_reg_a(magneto_odr=lsm303agr.MAGNETO_ODR_10HZ,
                                                      magneto_md=lsm303agr.MAGNETO_MD_IDLE0, lp=0, comp_temp_en=1)

        return None

    def is_completed(self):
        """Is acquisition completed flag."""
        return (not self._bme280_awaiting_valid_measurements) and (not self._lsm303agr_awaiting_valid_measurements)

    def get_latest_data_as_bytes(self) -> bytes:
        """Get the latest data as a bytes."""
        # Format as packed floats
        # https://docs.python.org/3/library/struct.html
        packed_bytes = struct.pack("ffffffffff", self._bme280_temperature, self._bme280_pressure, self._bme280_humidity,
                                   self._lsm303agr_accel[0], self._lsm303agr_accel[1], self._lsm303agr_accel[2],
                                   self._lsm303agr_magneto[0], self._lsm303agr_magneto[1], self._lsm303agr_magneto[2],
                                   self._lsm303agr_temperature)
        # To unpack
        #bme280_temperature, bme280_pressure, bme280_humidity, \
        #lsm303agr_accel_x, lsm303agr_accel_y, lsm303agr_accel_z, \
        #lsm303agr_magneto_x, lsm303agr_magneto_y, lsm303agr_magneto_z, \
        #lsm303agr_temperature = struct.unpack("ffffffffff", packed_bytes)

        return packed_bytes

    def get_latest_data_as_json(self):
        """Get the latest data as a json object. Which can then be loaded into json.dump/dumps."""
        jason = {"bme280": {"temperature": self._bme280_temperature,
                            "pressure": self._bme280_pressure,
                            "humidity": self._bme280_humidity},
                 "lsm303agr": {"accelerometer": {"x": self._lsm303agr_accel[0],
                                                 "y": self._lsm303agr_accel[1],
                                                 "z": self._lsm303agr_accel[2]},
                               "magnetometer": {"x": self._lsm303agr_magneto[0],
                                                "y": self._lsm303agr_magneto[1],
                                                "z": self._lsm303agr_magneto[2]},
                               "temperature": self._lsm303agr_temperature}}

        return jason


def get_internal_sensor_payload_instance() -> SensorPayload:
    """Get instance of the SensorPayload for this module.
    Override this function in derived modules."""
    return PebSensorPayload()

def get_external_sensor_payload_instance() -> SensorPayload:
    """Get instance of the SensorPayload for this module.
    Override this function in derived modules."""
    return ExternalSensorPayload()
