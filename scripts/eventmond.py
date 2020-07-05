#!/usr/bin/env python3
"""Handles events for the Buffalo TeraStation 5600

Currently this only handles SATA hotplug events; it will enable/disable
power to the drive triggering the event and request a bus rescan of the
SATA port so Linux can pick the devices up.
"""

import argparse
import os
import logging
import time

EVENT_FILE = '/proc/buffalo/kernevnt'
POWER_CONTROL_PATH = '/proc/buffalo/gpio/power_control'
POWER_CONTROL_FILE = 'hdd'
POWER_CONTROL_DELAY = 3 # seconds
SATA_RESCAN_PATH = '/sys/class/scsi_host/host'

SATA_POWER = {}

def initiate_sata_rescan(port_num):
    """Requests a bus rescan for a given SATA port"""
    with open(os.path.join(SATA_RESCAN_PATH + str(port_num), 'scan'), 'wt') as rescan_file:
        rescan_file.write('- - -\n')

def set_sata_power(port_num, activate):
    """Enables or disables power to a given SATA port"""
    sata_power_port = SATA_POWER[port_num]
    if sata_power_port['active'] == activate:
        return
    with open(sata_power_port['path'], 'wt') as power_file:
        command = 'off'
        if activate:
            command = 'on'
        power_file.write(command + '\n')
        sata_power_port['active'] = activate
    time.sleep(POWER_CONTROL_DELAY)

def process_event(name):
    """Given a named event, takes the appropriate action"""
    if name.startswith('SATA ') and name.endswith(' plugged'):
        port_id = int(name.split()[1])
        logging.info('enabling power to port %d', port_id)
        set_sata_power(port_id, True)
        initiate_sata_rescan(port_id)
    elif name.startswith('SATA ') and name.endswith(' unplugged'):
        port_id = int(name.split()[1])
        logging.info('disabling power to port %d', port_id)
        set_sata_power(port_id, False)
        initiate_sata_rescan(port_id)

def update_sata_port_power():
    """Obtains the current SATA power settings for all ports"""
    for fname in os.listdir(POWER_CONTROL_PATH):
        if fname.startswith(POWER_CONTROL_FILE):
            port_index = int(fname[len(POWER_CONTROL_FILE):])
            port_path = os.path.join(POWER_CONTROL_PATH, fname)
            with open(port_path, 'rt') as port_file:
                active = port_file.read().strip() == 'on'
            SATA_POWER[port_index] = {'path': port_path, 'active': active}
            logging.info('sata port %d power status: %s', port_index, 'on' if active else 'off')

def parse_arguments():
    """Returns the parsed arguments"""
    parser = argparse.ArgumentParser(description='Event monitor')
    parser.add_argument('--verbose', '-v', action='count')
    return parser.parse_args()

if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)-15s %(message)s')

    ARGS = parse_arguments()
    if ARGS.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    update_sata_port_power()

    with open(EVENT_FILE, 'rt') as event_f:
        while True:
            # events have a \0-byte, so get rid of that
            EVENTS = [s.strip()[:-1] for s in event_f.read().split('\n')]
            for event in EVENTS:
                logging.debug('event "%s"', event)
                process_event(event)
