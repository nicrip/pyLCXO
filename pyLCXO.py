#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import serial
import re
import time
import datetime
import sys
import curses
from .geomag import geomag

class LCXO(object):
    def __init__(self, port, baud, timeout):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        
        self.gnss = 'none'
        self.tracked_sats = -1
        self.visible_sats = -1
        self.latitude = -1
        self.longitude = -1
        self.altitude = -1
        self.speed_over_ground = -1
        self.course_over_ground = -1
        self.gps_receiver_status = 'invalid'

        self.year = 2024
        self.month = 1
        self.day = 1
        self.hours = 0
        self.minutes = 0
        self.seconds = 0
        self.utc_date = -1
        self.utc_time = -1
        self.utc = -1

        self.pps_source_mode = 'none'
        self.pps_source_state = 'none'
        self.pps_lock_status = -1
        self.holdover_state = 'none'
        self.last_holdover_duration = -1
        self.freq_error_estimate = -1
        self.time_interval_diff = -1
        self.health_status = 'none'

        self.pulse_freq = 2
        self.pulse_period_us = 500000

        self.geomag = geomag.GeoMag()
        self.magnetic_declination = -14.42   #declination at MIT

        # initialize serial port
        try:
            self.LCXO = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except:
            print('No serial device detected on port {}!'.format(self.port))
            exit(1)
        print('serial port {} initialized.'.format(self.port))

    def run(self):
        prev_loop_start = time.time()
        console = curses.initscr()
        curses.noecho()
        curses.cbreak()
        console.keypad(True)
        console.nodelay(True)
        input = None
        while True:
            console.clear()
            loop_start = time.time()
            loop_time = loop_start - prev_loop_start
            prev_loop_start = loop_start

            self.readGPSPos()
            self.readGPSTime()
            self.readOCXOSync()

            console.addstr(0,0,'Reading Jackson Labs LC_XO Data')
            console.addstr(1,0,'')
            console.addstr(2,0,'UTC:')
            console.addstr(3,0,'    Seconds: {:10.3f} s'.format(self.utc))
            console.addstr(4,0,'    Date: {:06d}'.format(self.utc_date))
            console.addstr(5,0,'    Time: {:06.3f} s'.format(self.utc_time))
            console.addstr(6,0,'')
            console.addstr(7,0,'Position:')
            console.addstr(8,0,'    Status: {}'.format(self.gps_receiver_status))
            console.addstr(9,0,'    Longitude: {:6.5f} deg'.format(self.longitude))
            console.addstr(10,0,'    Latitude: {:6.5f} deg'.format(self.latitude))
            console.addstr(11,0,'    Altitude: {:6.5f} m'.format(self.altitude))
            console.addstr(12,0,'')
            console.addstr(13,0,'Number of Visible Satellites: {:3d}'.format(self.visible_sats))
            console.addstr(14,0,'Number of Tracked Satellites: {:3d}'.format(self.tracked_sats))
            console.addstr(15,0,'GNSS: {}'.format(self.gnss))
            console.addstr(16,0,'Speed-over-Ground: {:3.2f} knots'.format(self.speed_over_ground))
            console.addstr(17,0,'Course-over-Ground: {:3.2f} deg'.format(self.course_over_ground))
            console.addstr(18,0,'Magnetic Declination: {:3.2f} deg'.format(self.magnetic_declination))
            console.addstr(19,0,'')
            console.addstr(20,0,'OCXO Status:')
            console.addstr(21,0,'    PPS Source Mode: {}'.format(self.pps_source_mode))
            console.addstr(22,0,'    PPS Source State: {}'.format(self.pps_source_state))
            console.addstr(23,0,'    PPS Lock Status: {:1d}'.format(self.pps_lock_status))
            console.addstr(24,0,'    Holdover State: {}'.format(self.holdover_state))
            console.addstr(25,0,'    Last Holdover Duration: {:d}'.format(self.last_holdover_duration))
            console.addstr(26,0,'    Freq Error Estimate: {:.2e}'.format(self.freq_error_estimate))
            console.addstr(27,0,'    Time Interval Difference: {:.2e}'.format(self.time_interval_diff))
            console.addstr(28,0,'    Health Status: {}'.format(self.health_status))
            console.addstr(29,0,'')
            console.addstr(30,0,'Loop Cycle Time: {:4.2f} ms'.format(loop_time*1e3))
            console.addstr(31,0,'')
            console.addstr(32,0,'Press `q` to quit.')
            console.refresh()

            time.sleep(0.05)

            try:
                input = console.getkey()
            except:
                pass

            if input == 'q':
                curses.nocbreak()
                console.nodelay(False)
                console.keypad(False)
                curses.echo()
                curses.endwin()
                print('Quitting Jackson Labs LC_XO program.')
                sys.exit(0)

    def readGPSPos(self):
        self.LCXO.write(b'cGPS?\r')
        while True:
            b = self.LCXO.readline()
            s = b.decode('utf-8')
            # print(s)
            if 'Unknown command' in s:
                break
            if not b:
                break
            if 'GNSS' in s:
                sa = re.split(': | \r', s)
                self.gnss = sa[1]
                # print(self.gnss)
            if 'TRACKED SATS' in s:
                sa = re.split(':|\r', s)
                self.tracked_sats = int(sa[1])
                # print(self.tracked_sats)
            if 'VISIBLE SATS' in s:
                sa = re.split(':|\r', s)
                self.visible_sats = int(sa[1])
                # print(self.visible_sats)
            if 'ACTUAL POSITION' in s:
                b = self.LCXO.readline()
                s = b.decode('utf-8')
                sa = re.split(',|\r', s)
                latitutde_hem = sa[0]
                latitude = float(sa[1])
                latitude_d = int(latitude/100.0)
                latitude_m = latitude - latitude_d*100.0
                latitude = latitude_d + latitude_m/60.0
                if latitutde_hem == 'S':
                    latitude = -latitude
                self.latitude = latitude
                # print(self.latitude)
                b = self.LCXO.readline()
                s = b.decode('utf-8')
                sa = re.split(',|\r', s)
                longitude_hem = sa[0]
                longitude = float(sa[1])
                longitude_d = int(longitude/100.0)
                longitude_m = longitude - longitude_d*100.0
                longitude = longitude_d + longitude_m/60.0
                if longitude_hem == 'W':
                    longitude = -longitude
                self.longitude = longitude
                # print(self.longitude)
                b = self.LCXO.readline()
                s = b.decode('utf-8')
                sa = re.split(' m\r', s)
                self.altitude = float(sa[0])
                # print(self.altitude)
                b = self.LCXO.readline()
                s = b.decode('utf-8')
                sa = re.split(' Knots\r', s)
                self.speed_over_ground = float(sa[0])
                # print(self.speed_over_ground)
                b = self.LCXO.readline()
                s = b.decode('utf-8')
                sa = re.split(' Degrees\r', s)
                self.course_over_ground = float(sa[0])
                # print(self.course_over_ground)
                mag = self.geomag.GeoMag(self.latitude, self.longitude)
                self.magnetic_declination = mag.dec
                # print(self.magnetic_declination)
            if 'GPS Receiver Status' in s:
                sa = re.split(': |\r', s)
                self.gps_receiver_status = sa[1]
                break
                # print(self.gps_receiver_status)

    def readGPSTime(self):
        self.LCXO.write(b'cPTIME?\r')
        while True:
            b = self.LCXO.readline()
            s = b.decode('utf-8')
            # print(s)
            if 'Unknown command' in s:
                break
            if not b:
                break
            if 'DATE :' in s:
                sa = re.split(' :|\r|,', s)
                self.year = int(sa[1])
                self.month = int(sa[2])
                self.day = int(sa[3])
                self.utc_date = self.day*10000 + self.month*100 + (self.year-2000)
                # print(self.year, self.month, self.day)
                # print(self.utc_date)
            if 'TIME :' in s:
                sa = re.split(' :|\r|:', s)
                self.hours = int(sa[1])
                self.minutes = int(sa[2])
                self.seconds = int(sa[3])
                self.utc_time = self.hours*10000 + self.minutes*100 + self.seconds
                # print(self.hours, self.minutes, self.seconds)
                # print(self.utc_time)
                break
        dt = datetime.datetime(self.year, self.month, self.day, self.hours, self.minutes, self.seconds, 0)
        self.utc = (dt - datetime.datetime(1970, 1, 1)).total_seconds()
        # print(dt)
        # print(self.utc)

    def setPulseFreq(self, freq=None):
        if freq is None:
            a = 'p\r'
        else:
            a = 'p{}\r'.format(int(freq))
        self.LCXO.write(a.encode('utf-8'))
        while True:
            b = self.LCXO.readline()
            s = b.decode('utf-8')
            # print(s)
            if 'Unknown command' in s:
                break
            if not b:
                break
            if 'Pulse number' in s:
                sa = re.split(': |\r', s)
                self.pulse_freq = int(sa[1])
                # print(self.pulse_freq)
            if 'Trigger period' in s:
                sa = re.split(': |\r', s)
                self.pulse_period_us = int(sa[1])
                # print(self.pulse_period_us)
                break

    def readOCXOSync(self):
        self.LCXO.write(b'cSYNC?\r')
        while True:
            b = self.LCXO.readline()
            s = b.decode('utf-8')
            # print(s)
            if 'Unknown command' in s:
                break
            if not b:
                break
            if '1PPS SOURCE MODE' in s:
                sa = re.split(': |\r', s)
                self.pps_source_mode = sa[1]
                # print(self.pps_source_mode)
            if '1PPS SOURCE STATE' in s:
                sa = re.split(': |\r', s)
                self.pps_source_state = sa[1]
                # print(self.pps_source_state)
            if '1PPS LOCK STATUS' in s:
                sa = re.split(': |\r', s)
                self.pps_lock_status = int(sa[1])
                # print(self.pps_lock_status)
            if 'HOLDOVER STATE' in s:
                sa = re.split(': |\r', s)
                self.holdover_state = sa[1]
                # print(self.holdover_state)
            if 'LAST HOLDOVER DURATION' in s:
                sa = re.split(' |,|\r', s)
                self.last_holdover_duration = int(sa[3])
                # print(self.last_holdover_duration)
            if 'FREQ ERROR ESTIMATE' in s:
                sa = re.split(' |\r', s)
                self.freq_error_estimate = float(sa[3])
                # print(self.freq_error_estimate)
            if 'TIME INTERVAL DIFFERENCE' in s:
                sa = re.split(' |\r', s)
                self.time_interval_diff = float(sa[3])
                # print(self.time_interval_diff)
            if 'HEALTH STATUS' in s:
                sa = re.split(': |\r', s)
                self.health_status = sa[1]
                # print(self.health_status)
                break

if __name__ == "__main__":
    lc_xo = LCXO('/dev/ttyACM0', 115200, 1)
    lc_xo.run()