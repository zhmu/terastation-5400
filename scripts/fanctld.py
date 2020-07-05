#!/usr/bin/env python3
"""Controls fans in a Buffalo TeraStation 5600

This is a reimplementation of the fanctld.sh script shipped with TeraStation's
with some minor enhancements and bugfixes.

The algorithm is roughly the following:

1. Read the current fan speed settings (stop, slow, fast, full)
2. Obtain the highest temperature in the system
    - If drive temperature can be read (hddtemp), it will be used
    - Otherwise the temperature of the CPU cores will be used
3. Control each fan based on the currently highest temperature
    - If no temperature is available, go to full speed
    - If the temperature exceeds the max threshold, increase the fan speed
    - If the temperature is lower than the min threshold, decrease the fan speed

There are currently no provisions for checking the fan speed or over
temperature.
"""

import argparse
import logging
import os
import subprocess
import time

MONITOR_INTERVAL = 5 # seconds
DEVICE_PATH = "/dev"
DEVICE_TEMPERATURES = [
    '/sys/class/sugi/fan/temp1',
    '/sys/class/sugi/fan/temp2',
]
FANS = [
    {'name': 'fan1', 'device': '/sys/class/sugi/fan/fan1_speed', 'mode_index': 0},
    {'name': 'fan2', 'device': '/sys/class/sugi/fan/fan2_speed', 'mode_index': 0},
]
FAN_MODES = [
    {'name': 'stop', 'threshold_min': 20, 'threshold_max': 30},
    {'name': 'slow', 'threshold_min': 25, 'threshold_max': 40},
    {'name': 'fast', 'threshold_min': 35, 'threshold_max': 50},
    {'name': 'full', 'threshold_min': 45, 'threshold_max': 60},
]

def lookup_fan_mode_index(mode):
    """Given a mode, returns the index in FANS of the mode"""
    for index, fan_mode in enumerate(FAN_MODES):
        if fan_mode['name'] == mode:
            return index
    return 0

def get_disk_devices():
    """Returns a list of disk devices, i.e. [ "/dev/sda", "/dev/sdb" ]"""
    devices = []
    for dev in os.listdir(DEVICE_PATH):
        if not dev.startswith('sd'):
            continue
        if dev[-1].isdigit():
            continue
        devices.append(os.path.join(DEVICE_PATH, dev))
    return devices

def get_disk_temperatures(devices):
    """Given a list of disk devices, returns a list of temperatures"""
    temperatures = []
    for dev in devices:
        try:
            hdd_temp = subprocess.check_output(['/usr/sbin/hddtemp', '-n', dev])
            hdd_temp = hdd_temp.decode('utf-8').strip()
            hdd_temp = int(hdd_temp)
            temperatures.append(hdd_temp)
        except ValueError as _:
            pass
        except subprocess.CalledProcessError as _:
            pass
    return temperatures

def update_current_fan_mode():
    """For all FANS, fills out the current speed"""
    for fan in FANS:
        with open(fan['device'], 'rt') as fan_file:
            mode = fan_file.read().strip()
            fan['mode_index'] = lookup_fan_mode_index(mode)
            logging.info('current fan speed of %s: %s', fan['device'], mode)

def get_device_temperatures():
    """Yields a list of temperature values for all temperature devices"""
    temperatures = []
    for dev in DEVICE_TEMPERATURES:
        with open(dev, "rt") as temp_file:
            temperature = temp_file.read().strip()
            temperature = int(temperature)
            temperatures.append(temperature)
    return temperatures

def get_highest_temperature(devices):
    """Given a list of devices, yields the highest temperature for all devices.

    If no devices given, uses the CPU temperature sensors"""
    temperatures = get_disk_temperatures(devices)
    if not temperatures:
        temperatures = get_device_temperatures()
    if not temperatures:
        return None
    max_temperature = max(temperatures)
    logging.info('current highest temperature: %d', max_temperature)
    return max_temperature

def update_fan_mode(fan, mode):
    """Sets the given fan to a new mode"""
    new_mode_index = lookup_fan_mode_index(mode)
    with open(fan['device'], 'wt') as fan_file:
        fan_file.write(FAN_MODES[new_mode_index]['name'] + '\n')
    fan['mode_index'] = new_mode_index

def control_fans(highest_temperature):
    """Given the current highest temperature, updates the fans"""
    for fan in FANS:
        if highest_temperature is None:
            # no temperature -> go full-speed
            logging.warning('fan %s: no highest temperature, transitioning to "full"', fan['name'])
            update_fan_mode(fan, 'full')
            continue
        current_mode_index = fan['mode_index']
        current_mode = FAN_MODES[current_mode_index]

        next_mode = None
        if highest_temperature > current_mode['threshold_max'] and \
           current_mode_index < len(FAN_MODES) - 1:
            # advance to next mode
            next_mode_index = current_mode_index + 1
            next_mode = FAN_MODES[next_mode_index]['name']
            logging.info(
                'fan %s: temperature %s exceeds threshold %s, increasing speed to "%s"',
                fan['name'], highest_temperature, current_mode['threshold_max'], next_mode)
            update_fan_mode(fan, next_mode)
        elif highest_temperature < current_mode['threshold_min'] and \
             current_mode_index > 0:
            # advance to previous mode
            next_mode_index = current_mode_index - 1
            next_mode = FAN_MODES[next_mode_index]['name']
            logging.warning(
                'fan %s: temperature %s is lower than threshold %s, decreasing speed to "%s"',
                fan['name'], highest_temperature, current_mode['threshold_min'], next_mode)

        if next_mode:
            update_fan_mode(fan, next_mode)

def parse_arguments():
    """Returns the parsed arguments"""
    parser = argparse.ArgumentParser(description='Fan controller')
    parser.add_argument('--verbose', '-v', action='count')
    return parser.parse_args()

if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)-15s %(message)s')

    ARGS = parse_arguments()
    if ARGS.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    update_current_fan_mode()

    while True:
        DISK_DEVICES = get_disk_devices()
        HIGHEST_TEMPERATURE = get_highest_temperature(DISK_DEVICES)
        control_fans(HIGHEST_TEMPERATURE)

        time.sleep(MONITOR_INTERVAL)
