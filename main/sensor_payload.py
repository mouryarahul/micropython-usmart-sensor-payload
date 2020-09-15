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


def get_sensor_payload_instance():
    """Get instance of the SensorPayload for this module.
    Override this function in derived modules."""
    return SensorPayload()


class SensorPayload:
    """SensorPayload standard class."""

    def __init__(self):
        """Initialise."""
        pass

    def __call__(self):
        return self

    def get_est_acquisition_duration(self) -> float:
        """Get estimated acquisition duration in seconds."""
        return 0.0

    def start_acquisition(self):
        """Start an acquisition. Returns True if started successfully."""
        return True

    def process_acquisition(self):
        """Continue processing the acquisition. To be called periodically within the mainloop. 
        This is where the state machine keeps track on progress. """
        return None

    def is_completed(self):
        """Is acquisition completed flag."""
        return True

    def get_latest_data(self) -> bytes:
        """Get the latest data as a bytes."""
        return None




